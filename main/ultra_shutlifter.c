/*
 * ultra shutlifter cmd groups:
 *
 * cmds:        action:                 duration:   effect:
 *
 * UP           press UP key            500ms       move shutter up
 * DOWN         press DOWN key          500ms       move shutter down
 * SET          press SET key           500ms       none
 * CLK          press CLK key           2s          toggle timer on/off (if any)
 * SUN          press SUN key           2s          toggle sun trig on/off
 *
 * SUNTRIG      press CLK + SUN         500ms       set sun trig
 * DIR          press SET               12s         reverse up/down direction
 * UP_TIMER     press CLK + UP          200+500ms   program up timer
 * DOWN_TIMER   press CLK + DOWN        200+500ms   program down timer
 * UP_STOP      press SET + UP          @autostop   move upper endpoint for @autostop ms
 * DOWN_STOP    press SET + DOWN        @autostop   move lower endpoint for @autostop ms
 * FACRESET     press CLK + DOWN + UP   200ms+6s    factory reset hardware
 *
 * RESTART      send restart cmd                    reboot
 * NOOP         send noop cmd                       return status alone
 * OTA          send OTA cmd                        flash firmware if avail
 * @autostop=XX set autostop                        set duration for moving stop point away from current position (ms)
 * @autostop    read autostop                       return status alone
 *
 *
 * status fields returned after getting a cmd (example for CLK cmd):
 *
 * #[CLK]#[0]#[0]#[0]#[-67/1/0/200]#[0]
 *   ^                 ^   ^ ^  ^                          
 *   cmd            RSSI   |  \  \                                                             
 *                         |   \  `------- autostop (ms)
 *                   !CLK_LED   \                           
 *                               \                         
 *                            time since last movement (s)
 * 
 *
 * all cmds in one fell swoop:
 *
for i in FACRESET NOOP RESTART DIR UP_STOP UP_TIMER DOWN_STOP DOWN_TIMER SUNTRIG CLK SUN @autostop @autostop=222
do
echo --- $i ---;
echo $i ^esp32-16 | netcat host7 x-pq-sync-x
done
 */

// ---vvv--- standard includes ---vvv---
#include "sdkconfig.h"
#include "mnta.h"

// regular lifters
// ESP32-16 ultra_shutlifter/      esp-living-l
// ESP32-17 ultra_shutlifter/      esp-living-r
// ESP32-18 ultra_shutlifter/      esp-eating
// ESP32-19 ultra_shutlifter/      esp-working
// ESP32-20 ultra_shutlifter/      esp-sleeping

#define WIFI_INITIATOR

#if ESP32_(16) || \
    ESP32_(17) || \
    ESP32_(18) || \
    ESP32_(19) || \
    ESP32_(20)

#   define DEBUG 2
#   define MYSERVICE_PORT 8888         // enable/disable function
#   define STD_TARGET_HOST "host2.example.com"  // all lifters are at host13: mcfg_local.h's host7 default is unreachable there
#   define STD_TARGET_PORT 8888
#elif ESP32_(2)              // pre crafted development board
#   define DEBUG 2 
#   define MYSERVICE_PORT 8888         // enable/disable function
#   define REEDREC_PORT 8889           // enable/disable function: stream every PIN_REED edge (tools/reedrec)
#else
#   this may not happen
#endif

#include "mlcf.h"
#ifdef MCFG_LOCAL
#include "mcfg_local.h"
#else
#include "mcfg.h"
#endif
#include "mcom.h"
// ---^^^--- standard includes ---^^^---

#define PIN_UP 4                    // 5 hardware buttons output pin defs
#define PIN_DOWN 19
#define PIN_SET 18
#define PIN_CLK 17
#define PIN_SUN 16
#define PIN_REED 21                  // reed input sense

#define SRV_CMDS "^(UP|DOWN|SET|CLK|SUNTRIG|SUN|DIR|UP_TIMER|DOWN_TIMER|UP_STOP|DOWN_STOP|FACRESET|RESTART|NOOP)$"

#define TIMER_BASE_FREQ 1000000        // 1MHz tick (gptimer resolution, as arduino-esp32 3.x took it)
#define TIMER_INTERVAL 16               // pulse distance (ms) between reed_intrs (~61Hz) -> reed_pinlevel sampled after 1/2 TIMER_INTERVAL
#define MOVD_MIN_INTERVAL 3000          // movement report signaling max. frequ
#define ONE_SEC_IN_MILLIS 1000
#define ONE_MILLISEC_IN_USECS 1000
#define PINLEVEL_GUARD_TRIGGER 5        // exactly timed intrs in a row required to trigger setting of new val
                                        // -> TIMER_INTERVAL * PINLEVEL_GUARD_TRIGGER == 80ms
#define MIN_GAP_TO_DETECT_MOVEMENT 100  // minimum gap length between pulse bursts to detect movement (ms)

/*
 * the millisecond clock of this file - NOT tstamp().
 *
 * tstamp() is esp_log_timestamp(): a base plus FreeRTOS ticks, so it steps in 1ms under arduino's
 * FREERTOS_HZ 1000 but in 10ms at the idf reference's HZ 100. reed_intrs() sorts pulse distances
 * into a 16..17ms window, and no multiple of 10 can ever land in it: the sampling timer would
 * never be armed, reed_pinlevel_guarded would never be adopted, and every single pulse would be
 * counted as an error. so the timestamps this file compares come off esp_timer (1us hardware
 * time) and are floored to the ms arduino measured in - same type and same resolution as tstamp()
 * had there. mcom.h keeps printing its own lines with tstamp()
 */
#define mstamp() ((__u32)(esp_timer_get_time() / 1000))

gptimer_handle_t mytimer;               // was hw_timer_t *
volatile bool timer_armed;             // arduino-esp32 3.x has no timerAlarmEnabled()
_u32 reed_intr;                         // counting intrs_off since last intrs_off()
_u32 reed_intr_consecutives;
_u32 reed_pinlevel;                     // momentary value
_u32 reed_pinlevel_guarded;             // adopted after PINLEVEL_GUARD_TRIGGER grace period

_u32 stat_last;                         // time of last status reported
_u32 intr_last;                         // time of last reed_intrs()
_u32 movd_last;                         // time of last movement reported

_u32 error_cnt;                         // error memory for diagnosis
_u8  error_buf[32];

_u32 new_movement;                      // timestamp of last (still unreported) movement registered

/*
 * some theory of operation - how PIN_REED works (the numbers as the reed recorder measured them on
 * the bench, see README.md and recordings/):
 *
 * - PIN_REED is connected via a voltage divider to the output of the reed relais.
 * - the reed relais input is connected to rail_2
 * - so if the reed relais closes contact we read the voltage present on rail_2. with the contact
 *   open the divider holds PIN_REED at 0V
 * - rail_2 basically controls the CLK LED cathode voltage meaning that: 5V on rail_2 - CLK LED OFF/ 0V on rail_2 - CLK LED ON
 * - additionally voltage on rail_2 is continuously overlaid by pulses at frequency 61Hz (every 16.35ms). with the CLK LED
 *   off a pulse is a dip to 0V, 14us wide - 17us while the GW60 sees one of its keys pressed. with the LED on it is a
 *   spike to 5V, 18us or 19us wide, and while the motor runs every few spikes is a wide one: 77.8us going up, 78.8us or
 *   79.3us going down (from 0.25s after the start on). whenever the motor stops, the pulses pause for ~44ms
 * - with the CLK LED off, PIN_REED follows the reed contact itself between the pulses: the contact closing is a rising
 *   edge at any phase, followed by ~70us of bounce - rising edges for reed_intrs() too, at no 16ms distance
 * - the small magnetic wheel (with 2 magnets baked in) causes ~280ms in length reed contact closures being spaced at
 *   about 700ms, the contact being open for ~415ms in between
 * - so each reed contact closure contains: 0.280s / (1 / 61)s = 17.1 pulses (per burst), 14 to 18 counted
 * - the larger magnetic wheel repeats every 5 closures: it stretches or shortens them (232ms to 300ms), and where its
 *   magnet passes the open contact it closes it once more, briefly (47ms to 109ms, a burst of a few pulses) - on the bench
 *   after closures 2, 7, 12, 17, 22 and 27, counted from the lower end. its relation to the small wheel drifts slowly
 *
 * |                           |
 * |closed reed pulse          |
 * |             burst distance|
 * |                           |
 *
 * <------------700ms---------->
 * |||||||______________________|||||||______________________|||||||______________________
 * <----->              ...---->
 * |280ms|               ~415ms silence
 * |closed                 movd trigger: the first rising edge after more than MIN_GAP_TO_DETECT_MOVEMENT of silence
 * |reed |
 * |burst|
 * |length|
 */

/*
 * client configurable variables
 */
_u32 cfg_autostop = 200;                // auto stop adjustment travel default duration (ms)

void
record_err(_u8 err)
{
    if (error_cnt < _SZ(error_buf)) {
        error_buf[error_cnt] = err;
    }
    ++error_cnt;
}

/*
 * protect against intrs!
 */
void
dump_err()
{
    _u32 i;
    _u32 tpr = min(error_cnt, _SZ(error_buf));

    if (tpr) PR00("[ ");
    for (i = 0; i < tpr; ++i) {
        PR00("%02x ", error_buf[i]);
    }
    if (tpr) PR00("]\n");
    error_cnt = 0;
}

// about every other 16ms (~61Hz)
void
IRAM_ATTR reed_intrs(void *arg)
{
    _u32 intr_now = mstamp();

    ++reed_intr;
    if (intr_last) {
        _u32 timer_interval = intr_now - intr_last;

        if (TIMER_INTERVAL <= timer_interval && timer_interval <= TIMER_INTERVAL + 1) {
            if (reed_intr_consecutives >= PINLEVEL_GUARD_TRIGGER) { // wait until signal settled
                reed_pinlevel_guarded = reed_pinlevel;              // then take over last sample
            }
            gptimer_set_raw_count(mytimer, 0);              // was timerWrite(mytimer, 0)
            if (!timer_armed) {
                /*
                 * was timerStart() + timerAlarm(mytimer, .., false, 0) - which is exactly this
                 * pair of calls in exactly this order, minus arduino's NULL checks. the alarm is
                 * one shot: the hardware disables it when it fires, and timer_intrs() clears the
                 * flag below. the counter keeps running from here on, so every further edge only
                 * rewinds it to 0 above
                 */
                gptimer_alarm_config_t alarm = {
                    .alarm_count = (TIMER_INTERVAL >> 1) * ONE_MILLISEC_IN_USECS,
                    .reload_count = 0,
                    .flags.auto_reload_on_alarm = false,
                };

                gptimer_start(mytimer);                     // stopped by setup / intrs_off()
                gptimer_set_alarm_action(mytimer, &alarm);
                timer_armed = 1;
            } else {
#if DEBUG > 1
                record_err(~0);
#endif
            }
            ++reed_intr_consecutives;
        } else if (timer_interval > MIN_GAP_TO_DETECT_MOVEMENT) {
            // movement reporting triggers on new pulses after more than MIN_GAP_TO_DETECT_MOVEMENT of silence
            // the minimum limit for the absence of pulses additionally defines
            // the max rate of reported movements
            new_movement = intr_now;
            reed_intr_consecutives = 0;
        } else {
#if DEBUG > 1
            record_err(timer_interval);
#endif
            reed_intr_consecutives = 0;
        }
    }
    intr_last = intr_now;
}

bool
IRAM_ATTR timer_intrs(gptimer_handle_t timer, const gptimer_alarm_event_data_t *edata, void *arg)
{
    reed_pinlevel = gpio_get_level(PIN_REED);   // was digitalRead()
    timer_armed = 0;                       // single shot: the alarm disarms itself
    return false;                          // no task woken, as arduino's timerFnWrapper() returned
}

void
intrs_off()
{
TP05
    gpio_isr_handler_remove(PIN_REED);                  // was detachInterrupt(PIN_REED): drops the
    gpio_set_intr_type(PIN_REED, GPIO_INTR_DISABLE);    // handler and disables the pin interrupt
    if (timer_armed) { gptimer_stop(mytimer); timer_armed = 0; }
}

void
intrs_on()
{
TP05
    reed_intr = 0;
    intr_last = 0;                                  // avoid false movement detection
    error_cnt = 0;                                  // clear errs possibly not processed yet

    /*
     * what attachInterrupt(PIN_REED, reed_intrs, RISING) called. the isr service is installed with
     * flags 0 exactly as arduino did it (CONFIG_ARDUINO_ISR_IRAM is off in both references), so
     * the handler is not IRAM bound and the interrupt is simply held off while the flash cache is
     * down - which is what lets reed_intrs() call the gptimer functions, none of which are in IRAM
     * either. a failing isr service leaves the device without reed counting but still answering
     * cmds, as it did on arduino, rather than aborting into a boot loop
     */
    esp_err_t err = gpio_install_isr_service(0);

    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {    // INVALID_STATE: already installed
        PR00("PIN %u ISR service failed to start: 0x%x\n", PIN_REED, err);
        return;
    }
    gpio_set_intr_type(PIN_REED, GPIO_INTR_POSEDGE);    // RISING
    gpio_isr_handler_add(PIN_REED, reed_intrs, 0);      // associate PIN and trigger-cond to fct (and enables it)
}

// ======================================== reed recorder ========================================
#if defined(REEDREC_PORT)
/*
 * streams every edge PIN_REED makes to whoever connects to REEDREC_PORT, for as long as the
 * connection lasts - the raw material for working out what the lifter's reed line carries (see
 * README.md and tools/). one line per event, t in us since the connect:
 *
 *      <t> 0|1         an edge, and the level it went to
 *      <t> = 0|1       the level sampled at the connect, then after every REC_QUIET_MS of silence
 *      <t> cmd <CMD>   the service port starts executing <CMD>, i.e. its key press begins
 *      <t> done <CMD>  ... and is through with it
 *      <t> lost <n>    the ring ran over: n events are missing right before this line
 *
 * the edges are timestamped by hardware, not by an interrupt handler. the line carries pulses
 * only 14us wide, and an ISR that reads a clock or the pin level can be late by more than that
 * whenever wifi or the flash holds the cpu - it would see the two edges of such a pulse as one.
 * two mcpwm capture channels watch the pin instead, one per edge direction, each latching the
 * 80MHz capture timer at the edge itself: the value stays exact however late it is read, and the
 * two edges of a pulse land in two different registers, so neither overwrites the other. what
 * still overwrites is a second edge of the same direction within the interrupt latency - reed
 * bounce, a few us apart: the recording then shows one level twice in a row, an edge pair lost
 * in between and the level after it right.
 * all of it runs beside reed_intrs(), which is left alone: the capture driver only routes the
 * pin's input into the mcpwm through the gpio matrix and never touches the pin's interrupt type.
 * the capture interrupt and the stream task live on core 1, away from wifi
 */
#include "driver/mcpwm_cap.h"

#define REC_RING 2048                   // events between capture interrupt and stream task, power of 2
#define REC_CHUNK 64                    // events taken out of the ring per pass
#define REC_POLL 50                     // ms between passes over an empty ring
#define REC_QUIET_MS 500                // a "=" line after this long without any other line
#define REC_SEND_TIMEOUT 10             // x 1s, a client that stops reading ends the recording

enum { REC_FALL, REC_RISE, REC_CMD, REC_DONE, REC_LOST };   // FALL/RISE == the level an edge goes to

typedef struct {
    _i64 t;                             // capture timer ticks since the connect
    _u32 kind;
    _u32 arg;                           // REC_CMD/REC_DONE: slot in rec_names[], REC_LOST: count
} rec_ev_t;

rec_ev_t rec_ring[REC_RING];
_u32 rec_head, rec_tail;                // free running, masked on access
_u32 rec_lost;                          // events dropped since the ring last had room
portMUX_TYPE rec_mux = portMUX_INITIALIZER_UNLOCKED;   // guards all of the above, the ref and the names
volatile bool rec_on;                   // a client is connected
_u32 rec_tpu;                           // capture timer ticks per us (80: APB, fixed on the esp32)
_i64 rec_t0;                            // esp_timer_get_time() at the connect
_i64 rec_ref_us;                        // esp_timer_get_time() at the first edge of a recording, 0 before it
_u32 rec_ref_cap;                       // ... and that edge's capture value

/*
 * a cmd name lives here from rec_mark() until its line is out. cmds run one at a time and each
 * holds the service port for 500ms at least, so a slot is not wanted again for seconds
 */
_i8 rec_names[8][16];
_u32 rec_name_seq;

/*
 * rec_mux held. one slot is always kept back, so that the first event after an overrun can
 * close the gap with a REC_LOST
 */
void
IRAM_ATTR rec_push(_i64 t, _u32 kind, _u32 arg)
{
    if (rec_head - rec_tail > REC_RING - 2) {
        ++rec_lost;
        return;
    }
    if (rec_lost) {
        rec_ring[rec_head++ & (REC_RING - 1)] = (rec_ev_t){ t, REC_LOST, rec_lost };
        rec_lost = 0;
    }
    rec_ring[rec_head++ & (REC_RING - 1)] = (rec_ev_t){ t, kind, arg };
}

/*
 * one call per captured edge, arg is the level the edge went to. the capture timer is 32 bits at
 * 80MHz and wraps every 53.7s - shorter than the reed line may stay silent - so the whole wraps
 * since the recording's first edge are counted off esp_timer. that clock is read late by the
 * interrupt latency, a few us, against a wrap of 2^32 ticks
 */
bool
IRAM_ATTR rec_cap(mcpwm_cap_channel_handle_t chan, const mcpwm_capture_event_data_t *edata, void *arg)
{
    _i64 now = esp_timer_get_time();

    if (rec_on) {
        portENTER_CRITICAL_ISR(&rec_mux);
        if (!rec_ref_us) {
            rec_ref_us = now;
            rec_ref_cap = edata->cap_value;
        }
        _u32 d = edata->cap_value - rec_ref_cap;                       // since the first edge, mod 2^32
        _i64 wraps = ((now - rec_ref_us) * rec_tpu - d + ((_i64)1 << 31)) >> 32;

        rec_push((rec_ref_us - rec_t0) * rec_tpu + d + wraps * ((_i64)1 << 32), (_u32)arg, 0);
        portEXIT_CRITICAL_ISR(&rec_mux);
    }
    return false;
}

void
rec_mark(_u32 kind, _i8cp name)
{
    _i64 now = esp_timer_get_time();

    portENTER_CRITICAL(&rec_mux);
    if (rec_on) {
        _u32 slot = rec_name_seq++ % _NE(rec_names);

        *rec_names[slot] = 0;
        strncat(rec_names[slot], name, _SZ(rec_names[slot]) - 1);
        rec_push((now - rec_t0) * rec_tpu, kind, slot);
    }
    portEXIT_CRITICAL(&rec_mux);
}

/*
 * rising edges on one channel, falling ones on the other. the rising one is registered first
 * because a shared interrupt calls its handlers newest first: when both edges of a dip - the
 * pulse shape seen with the CLK LED off - are pending at once, they then reach the ring in order
 */
esp_err_t
rec_capture_init(void)
{
    esp_err_t err;
    mcpwm_cap_timer_handle_t timer;
    mcpwm_capture_timer_config_t timer_config = {
        .group_id = 0,
        .clk_src = MCPWM_CAPTURE_CLK_SRC_DEFAULT,
    };
    __u32 res;                          // uint32_t, which is unsigned long here
    _u32 level;

    if ((err = mcpwm_new_capture_timer(&timer_config, &timer)) != ESP_OK ||
        (err = mcpwm_capture_timer_get_resolution(timer, &res)) != ESP_OK) {
        return err;
    }
    rec_tpu = res / 1000000;
    for (level = 2; level--; ) {
        mcpwm_cap_channel_handle_t chan;
        mcpwm_capture_channel_config_t chan_config = {
            .gpio_num = PIN_REED,
            .prescale = 1,
            .flags.neg_edge = !level,
            .flags.pos_edge = level,
        };
        mcpwm_capture_event_callbacks_t cbs = { .on_cap = rec_cap };

        if ((err = mcpwm_new_capture_channel(timer, &chan_config, &chan)) != ESP_OK ||
            (err = mcpwm_capture_channel_register_event_callbacks(chan, &cbs, (void *)level)) != ESP_OK ||
            (err = mcpwm_capture_channel_enable(chan)) != ESP_OK) {
            return err;
        }
    }
    if ((err = mcpwm_capture_timer_enable(timer)) != ESP_OK ||
        (err = mcpwm_capture_timer_start(timer)) != ESP_OK) {
        return err;
    }
    PR05("reed recorder: capture at %u ticks/us on PIN %u\n", rec_tpu, PIN_REED);
    return ESP_OK;
}

// us with two decimals, the capture resolution is 12.5ns
_i32
rec_fmt_t(_i8p str, _u32 siz, _i64 t)
{
    if (t < 0) t = 0;   // only an edge a few us ahead of the connect, swapped behind the first one
    return snprintf(str, siz, "%lld.%02lld", t / rec_tpu, t % rec_tpu * 100 / rec_tpu);
}

/*
 * one client at a time, a second one waits in the backlog until the first is gone. the stream
 * ends when a send fails, i.e. once the client has closed - nothing is ever read from it
 */
void
rec_task(void *arg)
{
TP05
    _i32 listen_sock, client_sock;
    struct sockaddr_in server_addr = {
        .sin_family = AF_INET,
        .sin_port = htons(REEDREC_PORT),
        .sin_addr.s_addr = htonl(INADDR_ANY),
    };
    esp_err_t err;
    rec_ev_t ev[REC_CHUNK + 1];         // +1: the carry, see below
    _u32 n, i, j, carry;
    _i8 out[1536];
    _i8 tstr[40];                       // what gcc proves rec_fmt_t() can need, 33
    _i32 len;
    bool ok;
    _i64 now, last_line;

    if ((err = rec_capture_init()) != ESP_OK) {      // here, so that the capture interrupt lands on core 1
        PR00("reed recorder: capture setup failed: 0x%x\n", err);
        vTaskDelete(0);
    }
    while (1) {
        listen_sock = socket(AF_INET, SOCK_STREAM, IPPROTO_IP);
        if (listen_sock < 0
         || bind(listen_sock, (struct sockaddr *)&server_addr, _SZ(server_addr)) < 0
         || listen(listen_sock, 1) < 0) {
            if (listen_sock >= 0) close(listen_sock);
            PR05("Error: reed recorder has no listen socket\n");
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }
        PR05("reed recorder listening on port %d\n", REEDREC_PORT);
        while ((client_sock = accept(listen_sock, 0, 0)) >= 0) {
            struct timeval tv = { .tv_sec = REC_SEND_TIMEOUT, .tv_usec = 0 };
            setsockopt(client_sock, SOL_SOCKET, SO_SNDTIMEO, &tv, _SZ(tv));

            portENTER_CRITICAL(&rec_mux);
            rec_head = rec_tail = rec_lost = 0;
            rec_ref_us = 0;
            rec_t0 = esp_timer_get_time();
            rec_on = 1;
            portEXIT_CRITICAL(&rec_mux);
            PR05("reed recorder: client connected\n");

            len = snprintf(out, _SZ(out), "# reedrec %s pin %u\n0.00 = %d\n",
                                                    DEVICE_FW, PIN_REED, gpio_get_level(PIN_REED));
            last_line = esp_timer_get_time();
            n = 0;
            ok = 1;
            while (ok) {
                portENTER_CRITICAL(&rec_mux);
                while (n < REC_CHUNK + 1 && rec_tail != rec_head) {
                    ev[n++] = rec_ring[rec_tail++ & (REC_RING - 1)];
                }
                portEXIT_CRITICAL(&rec_mux);

                /*
                 * events can reach the ring a few us out of order: a positive pulse's two edges
                 * when both are pending (the fall is served first), or a cmd mark against an edge
                 * still waiting for its interrupt. sort what was taken, and while the ring may
                 * still hold more, keep the latest event back for the next pass - so a swapped
                 * pair split by the chunk boundary still comes out in order
                 */
                for (i = 1; i < n; ++i) {
                    rec_ev_t e = ev[i];

                    for (j = i; j && ev[j - 1].t > e.t; --j) {
                        ev[j] = ev[j - 1];
                    }
                    ev[j] = e;
                }
                carry = n == REC_CHUNK + 1;

                for (i = 0; ok && i < n - carry; ++i) {
                    if (len > _SZ(out) - 64) {                  // room for the longest line, 55 chars
                        ok = send(client_sock, out, len, 0) == len;
                        len = 0;
                    }
                    rec_fmt_t(tstr, _SZ(tstr), ev[i].t);
                    switch (ev[i].kind) {
                    case REC_FALL:
                    case REC_RISE:
                        len += snprintf(out + len, _SZ(out) - len, "%s %u\n", tstr, ev[i].kind);
                        break;
                    case REC_CMD:
                    case REC_DONE:
                        len += snprintf(out + len, _SZ(out) - len, "%s %s %s\n", tstr,
                                        ev[i].kind == REC_CMD ? "cmd" : "done", rec_names[ev[i].arg]);
                        break;
                    case REC_LOST:
                        len += snprintf(out + len, _SZ(out) - len, "%s lost %u\n", tstr, ev[i].arg);
                        break;
                    }
                }
                if (carry) {
                    ev[0] = ev[n - 1];
                }
                n = carry;

                now = esp_timer_get_time();
                if (len) {
                    last_line = now;
                } else if (now - last_line >= REC_QUIET_MS * 1000) {
                    rec_fmt_t(tstr, _SZ(tstr), (now - rec_t0) * rec_tpu);
                    len = snprintf(out, _SZ(out), "%s = %d\n", tstr, gpio_get_level(PIN_REED));
                    last_line = now;
                }
                if (ok && len) {
                    ok = send(client_sock, out, len, 0) == len;
                    len = 0;
                }
                if (!carry) {
                    vTaskDelay(pdMS_TO_TICKS(REC_POLL));
                }
            }
            rec_on = 0;
            close(client_sock);
            PR05("reed recorder: client gone\n");
        }
        close(listen_sock);
        PR05("Error: accept failed\n");
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
#else
#define rec_mark(kind, name)
#endif

// ======================================== provide a service control port ========================================
#if defined(MYSERVICE_PORT)

#define RESTART_GRACE_PERIOD 700        // allow sending status et.al.

#define SET_INDX 1
#define GET_INDX 3
#define OTA_INDX 4
#define CMD_INDX 5

#define PMATCH_INDX (CMD_INDX + 1)                      // use last indx +1
#define MATCHED(indx) (pmatch[(indx)].rm_so != -1)

_i32 regerr;
_i8 regbuf[128];
_i8cp CMD_MATCH =
     \
    "^@([^ =]+)=([^ =]+)$" "|" \
    "^@([^ =]+)$"          "|" \
    "^(OTA)$"              "|" \
    SRV_CMDS
    ;
regex_t regex;
regmatch_t pmatch[PMATCH_INDX]; // nr of parenthesized subexprs + 1

/*
 * bounded by cv as well as by the match: the regex caps no field and a line may run to BUF_SIZE,
 * so a long @name or value would otherwise be copied past the end of cmd[]/val[] on the stack
 */
#define GET_CV_STR(cv, indx) \
    *cv = 0; strncat(cv, buf + pmatch[(indx)].rm_so, min((size_t)(pmatch[(indx)].rm_eo - pmatch[(indx)].rm_so), _SZ(cv) - 1)); \

#define GET_MISC_STR(str, a1, a2, a3) \
    snprintf(str, _SZ(str), "%d/%u/%lu/%u", wifi_rssi(), (a1), (a2), (a3))

#define BUF_SIZE 256    // for buf[], must also hold statusStr (cmd + misc + some)
#define RECV_TIMEOUT 1  // x 1s, the stream timeout arduino's readStringUntil() gave up after

#define CONTINUE(str) \
    PR05("Error: " str); \
    vTaskDelay(pdMS_TO_TICKS(1000)); \
    continue;

// WiFi.RSSI(): 0 while not associated
_i32
wifi_rssi(void)
{
    wifi_ap_record_t ap_info;

    return esp_wifi_sta_get_ap_info(&ap_info) == ESP_OK ? ap_info.rssi : 0;
}

/*
 * arduino's delay(): vTaskDelay(ms) at FREERTOS_HZ 1000, i.e. ms-1 .. ms of wall time. every use
 * below is the length of a key press into the lifter, and pdMS_TO_TICKS() alone would cut each of
 * them to the 10ms tick of this reference - up to 9ms short of what the lifter has been operated
 * with for years, and @autostop is settable to any ms at all. so sleep the whole ticks and sit out
 * the remainder, which is under one tick
 */
void
delay_ms(_u32 ms)
{
    _i64 end = esp_timer_get_time() + (_i64)ms * 1000;
    _i64 left;

    vTaskDelay(pdMS_TO_TICKS(ms));
    if ((left = end - esp_timer_get_time()) > 0) {
        esp_rom_delay_us(left);
    }
}

/*
 * was myserv(), polled from _loop() through arduino's WiFiServer - where a client that connected
 * and never sent held the whole loop. as a task of its own it blocks in accept() and drops such a
 * client after RECV_TIMEOUT. the cmd is still executed only after the status has gone out, and
 * still on this one task, so cmds stay serialized exactly as they were. shaped after
 * ultra_laundry.c / ultra_stairs.c
 */
void
myserv_task(void *arg)
{
TP05
    _i32 listen_sock, client_sock;
    struct sockaddr_in server_addr, client_addr;
    socklen_t addr_len;
    _i8 buf[BUF_SIZE];

    while (1) {
        listen_sock = socket(AF_INET, SOCK_STREAM, IPPROTO_IP);
        if (listen_sock < 0) {
            CONTINUE("unable to create socket\n");
        }
        server_addr.sin_family = AF_INET;
        server_addr.sin_port = htons(MYSERVICE_PORT);
        server_addr.sin_addr.s_addr = htonl(INADDR_ANY);
        if (bind(listen_sock, (struct sockaddr *)&server_addr, _SZ(server_addr)) < 0) {
            close(listen_sock);
            CONTINUE("socket bind failed\n");
        }
        /*
         * 4 as WiFiServer's max_clients: a cmd holds this task for up to 12s (DIR), and the next
         * caller has to be able to sit in the backlog for that long instead of being refused
         */
        if (listen(listen_sock, 4) < 0) {
            close(listen_sock);
            CONTINUE("listen failed\n");
        }
        PR05("listening on port %d\n", MYSERVICE_PORT);
        while (1) {
            addr_len = _SZ(client_addr);
            client_sock = accept(listen_sock, (struct sockaddr *)&client_addr, &addr_len);
            if (client_sock < 0) {
                close(listen_sock);
                PR05("Error: accept failed\n");
                vTaskDelay(pdMS_TO_TICKS(1000));
                break;      // to a fresh listen socket - a CONTINUE here would accept() on the closed one forever
            }
            PR05("new client connected\n");
            struct timeval tv = { .tv_sec = RECV_TIMEOUT, .tv_usec = 0 };   // a client that never sends must not hold the port
            setsockopt(client_sock, SOL_SOCKET, SO_RCVTIMEO, &tv, _SZ(tv));

            _i32 total = 0;
            while (1) {
                _i32 len = recv(client_sock, buf + total, 1, 0);
                if (len <= 0) { // gets terminated per \0
                    PR05("client disconnected prematurely\n");
                    break;
                }
                if (buf[total] == '\n') { // gets over patched per \0 / only \n (not \r) work
                    break;
                }
                total++;
                if (total >= BUF_SIZE - 1) { // gets terminated per \0
                    break;
                }
            }
            buf[total] = 0;

            _i8 cmd[64];
            _i8 val[64];
            _i8 misc[96];       // was 256, which cannot fit beside cmd[] in a BUF_SIZE reply
            _u8 stat = 0;       // (-Werror=format-truncation). the four fields run to 45 chars

            sprintf(cmd, "???");
            PR05("<%s>\n", buf);
            if (regerr = regexec(&regex, buf, _NE(pmatch), pmatch, 0)) {
                // no match
                regerror(regerr, &regex, regbuf, _SZ(regbuf));
PR05("%s\n", regbuf);
                sprintf(cmd, "ILLEG regex");
                stat = 1;
            } else if (MATCHED(OTA_INDX)) {
                GET_CV_STR(cmd, OTA_INDX);
            } else if (MATCHED(GET_INDX)) {
                GET_CV_STR(cmd, GET_INDX);
                if (!strcmp(cmd, "autostop")) {
                } else if (!strcmp(cmd, "autostop1")) {
                } else {
PR05("[ %s ] does not exist\n", cmd);
                    sprintf(cmd, "ERR get");
                    stat = 1;
                }
            } else if (MATCHED(SET_INDX)) {
                GET_CV_STR(cmd, SET_INDX);
                GET_CV_STR(val, SET_INDX + 1);
                if (!strcmp(cmd, "autostop")) {
                    cfg_autostop = atoi(val);
                } else {
PR05("[ %s ] does not exist\n", cmd);
                    sprintf(cmd, "ERR set");
                    stat = 1;
                }
            } else if (MATCHED(CMD_INDX)) {
                GET_CV_STR(cmd, CMD_INDX);
            } else {
PR05("illegal cmd\n");
                sprintf(cmd, "ILLEG cmd");
                stat = 1;
            }
            GET_MISC_STR(misc,
                                reed_pinlevel_guarded,
                                movd_last ? (mstamp() - movd_last) / ONE_SEC_IN_MILLIS : 0,
                                cfg_autostop);
            statusStr(cmd, stat, misc, buf, _SZ(buf));  // appends the \n itself
            send(client_sock, buf, strlen(buf), 0);
            close(client_sock);
            PR05("client disconnected\n");
            /*
             * delay execution to here to allow sending status instantly in advance
             */
#define SHORT_PRESS_DURATION 500            // standard key press duration
#define ACTIVATE_PRE_DURATION 200           // decouple multi key presses
#define ACTIVATE_CLK_DURATION 2000          // duration acc to manual
#define ACTIVATE_SUN_DURATION 2000          // duration acc to manual
#define ACTIVATE_DIR_DURATION 12000         // duration acc to manual
#define ACTIVATE_FACRESET_DURATION 6000     // duration acc to manual

            PR05("--- starting ---\n");
            rec_mark(REC_CMD, cmd);

// handle this separately for your safety
            if (!strcmp(cmd, "OTA")) {
                intrs_off();
                myota();
                vTaskDelay(pdMS_TO_TICKS(RESTART_GRACE_PERIOD));         // allow sending status
                esp_restart();      // in the event myota() fails we must reset anyway to restore interrupts et.al.

// single key cmds
            } else if (!strcmp(cmd, "UP")) {
                gpio_set_level(PIN_UP, 1);
                delay_ms(SHORT_PRESS_DURATION);
                gpio_set_level(PIN_UP, 0);
            } else if (!strcmp(cmd, "DOWN")) {
                gpio_set_level(PIN_DOWN, 1);
                delay_ms(SHORT_PRESS_DURATION);
                gpio_set_level(PIN_DOWN, 0);
            } else if (!strcmp(cmd, "SET")) {
                gpio_set_level(PIN_SET, 1);
                delay_ms(SHORT_PRESS_DURATION);
                gpio_set_level(PIN_SET, 0);
            } else if (!strcmp(cmd, "CLK")) {
                gpio_set_level(PIN_CLK, 1);
                delay_ms(ACTIVATE_CLK_DURATION);
                gpio_set_level(PIN_CLK, 0);
            } else if (!strcmp(cmd, "SUN")) {
                gpio_set_level(PIN_SUN, 1);
                delay_ms(ACTIVATE_SUN_DURATION);
                gpio_set_level(PIN_SUN, 0);
            } else if (!strcmp(cmd, "DIR")) {
                gpio_set_level(PIN_SET, 1);
                delay_ms(ACTIVATE_DIR_DURATION);
                gpio_set_level(PIN_SET, 0);

// composed key cmds
            } else if (!strcmp(cmd, "SUNTRIG")) {
                gpio_set_level(PIN_CLK, 1);
                gpio_set_level(PIN_SUN, 1);
                delay_ms(SHORT_PRESS_DURATION);
                gpio_set_level(PIN_CLK, 0);
                gpio_set_level(PIN_SUN, 0);

            } else if (!strcmp(cmd, "UP_TIMER")) {
                gpio_set_level(PIN_CLK, 1);
                delay_ms(ACTIVATE_PRE_DURATION);                     // avoid motor move
                gpio_set_level(PIN_UP, 1);
                delay_ms(SHORT_PRESS_DURATION);
                gpio_set_level(PIN_CLK, 0);
                gpio_set_level(PIN_UP, 0);
            } else if (!strcmp(cmd, "DOWN_TIMER")) {
                gpio_set_level(PIN_CLK, 1);
                delay_ms(ACTIVATE_PRE_DURATION);                     // avoid motor move
                gpio_set_level(PIN_DOWN, 1);
                delay_ms(SHORT_PRESS_DURATION);
                gpio_set_level(PIN_CLK, 0);
                gpio_set_level(PIN_DOWN, 0);

            } else if (!strcmp(cmd, "UP_STOP")) {
                gpio_set_level(PIN_SET, 1);
                gpio_set_level(PIN_UP, 1);
                delay_ms(cfg_autostop);
                gpio_set_level(PIN_SET, 0);
                gpio_set_level(PIN_UP, 0);
            } else if (!strcmp(cmd, "DOWN_STOP")) {
                gpio_set_level(PIN_SET, 1);
                gpio_set_level(PIN_DOWN, 1);
                delay_ms(cfg_autostop);
                gpio_set_level(PIN_SET, 0);
                gpio_set_level(PIN_DOWN, 0);

            } else if (!strcmp(cmd, "FACRESET")) {
                gpio_set_level(PIN_CLK, 1);
                delay_ms(ACTIVATE_PRE_DURATION);                     // avoid motor move
                gpio_set_level(PIN_DOWN, 1);
                gpio_set_level(PIN_UP, 1);
                delay_ms(ACTIVATE_FACRESET_DURATION);
                gpio_set_level(PIN_CLK, 0);
                gpio_set_level(PIN_DOWN, 0);
                gpio_set_level(PIN_UP, 0);

// other
            } else if (!strcmp(cmd, "RESTART")) {
                vTaskDelay(pdMS_TO_TICKS(RESTART_GRACE_PERIOD));         // allow sending status
                esp_restart();

            } else if (!strcmp(cmd, "NOOP")) {
                // nothing to do
            }
            PR05("--- completed ---\n");
            rec_mark(REC_DONE, cmd);
            delay_ms(500);                      // allow the opto to settle/ maintain a gap to next cmd
        }
    }
}
#endif

void
_setup(void)
{
TP05
    init_1st();
    init_2nd();

#if defined(MYSERVICE_PORT)
    PR05("MYSERVICE_PORT:  %u\n", MYSERVICE_PORT);
#endif
    PR05("PIN DEFINITIONS: UP: %u DOWN: %u SET: %u CLK: %u SUN: %u REED: %u\n",
                        PIN_UP, PIN_DOWN, PIN_SET, PIN_CLK, PIN_SUN, PIN_REED);
    /*
     * the pinMode()/digitalWrite() pairs, one gpio_config() for all five outputs. arduino's OUTPUT
     * is GPIO_MODE_INPUT_OUTPUT, nothing ever reads these back. the out register is 0 out of any
     * reset, so no opto sees a pulse from configuring them
     */
    // outputs
    {
        gpio_config_t io_conf = { 0 };
        io_conf.mode = GPIO_MODE_OUTPUT;
        io_conf.pin_bit_mask = (_u64)1 << PIN_UP
                             | (_u64)1 << PIN_DOWN
                             | (_u64)1 << PIN_SET
                             | (_u64)1 << PIN_CLK
                             | (_u64)1 << PIN_SUN;
        gpio_config(&io_conf);
    }
    gpio_set_level(PIN_UP, 0);
    gpio_set_level(PIN_DOWN, 0);
    gpio_set_level(PIN_SET, 0);
    gpio_set_level(PIN_CLK, 0);
    gpio_set_level(PIN_SUN, 0);

    // inputs
    {
        gpio_config_t io_conf = { 0 };
        io_conf.mode = GPIO_MODE_INPUT;     // no pull-up/-down, as pinMode(INPUT)
        io_conf.pin_bit_mask = (_u64)1 << PIN_REED;
        gpio_config(&io_conf);
    }
#if defined(MYSERVICE_PORT)
    if (regerr = regcomp(&regex, CMD_MATCH, REG_EXTENDED)) {
        regerror(regerr, &regex, regbuf, _SZ(regbuf));
        PR05("%s\n", regbuf);
    }
    xTaskCreate(myserv_task, "myserv_task", 8192, 0, 5, 0);  // 8192 as ultra_laundry.c/ultra_stairs.c, it runs myota()
    PR05("myservice started on port: %u\n", MYSERVICE_PORT);
#endif
    /*
     * what timerBegin(TIMER_BASE_FREQ) + timerAttachInterrupt() + timerStop() left behind: a
     * gptimer enabled but stopped, counting up at 1MHz once an edge starts it, with a shared
     * interrupt. arduino picked the first of SOC_GPTIMER_CLKS reaching a divider of 2..65536,
     * which on the esp32 is APB - GPTIMER_CLK_SRC_DEFAULT
     */
    {                                                                       // set timeout to 1/2 of pulse distance,
        gptimer_config_t timer_config = {                                   // single shot (see reed_intrs())
            .clk_src = GPTIMER_CLK_SRC_DEFAULT,
            .direction = GPTIMER_COUNT_UP,
            .resolution_hz = TIMER_BASE_FREQ,
            .flags.intr_shared = true,
        };
        gptimer_event_callbacks_t cbs = { .on_alarm = timer_intrs };

        ESP_ERROR_CHECK(gptimer_new_timer(&timer_config, &mytimer));
        ESP_ERROR_CHECK(gptimer_register_event_callbacks(mytimer, &cbs, 0));
        ESP_ERROR_CHECK(gptimer_enable(mytimer));                   // armed per reed edge, not here
    }
    intrs_on();
#if defined(REEDREC_PORT)
    xTaskCreatePinnedToCore(rec_task, "rec_task", 8192, 0, 5, 0, 1);   // core 1, see the reed recorder
#endif

    /*
     * init_3rd() only started the connect, and mysend() then waited up to 6s per cmd for the link
     * (wait4wifi); myconn_check() in _loop() re-issued the connect every 10s while it was down.
     * the idf handlers reconnect on every disconnect by themselves, so ur_connect() replaces both.
     * WIFI_CONN_WAIT because nothing this device does happens without the link - the movement
     * report as much as every single cmd - and the idf mysend() would otherwise block without any
     * timeout before the first IP of a boot. WIFI_PS_NONE: the device never sleeps, and
     * arduino's WIFI_PS_MIN_MODEM (WiFiGenericClass::_sleepEnabled), which this port first kept,
     * made the AP hold every packet to it until the next DTIM - ping measured 10..230ms, avg ~120ms
     * (2026_10_09). it comes last so the reed interrupt is already counting while we wait for the
     * accesspoint
     */
#if ESP32_(2)
    if (ur_connect(ROTA2K_SSID, WIFI_CONN_WAIT, WIFI_CONN_SLOW_FAIL, WIFI_PS_NONE)) {
#else
    if (ur_connect(UFIRE_SSID, WIFI_CONN_WAIT, WIFI_CONN_SLOW_FAIL, WIFI_PS_NONE)) {
#endif
        PR05("can't ur_connect, rebooting...\n");
        esp_restart();
    }

    /*
     * the lifters were first flashed with arduino, whose bootloader has app rollback enabled: the
     * first boot of an image written by the old espota/ArduinoOTA stays PENDING_VERIFY until the
     * app confirms it, and the next reset marks it ABORTED and boots the old image again. arduino's
     * initArduino() confirmed implicitly, this port has no arduino left, so confirm here - once the
     * link is up, which is what keeps the rollback for an image that crashes or never connects.
     * deliberately NOT in mcom.h: only the lifters are ever updated across that bootloader.
     * harmless where the bootloader has no rollback (a no-op on an image that is not pending)
     */
    esp_ota_mark_app_valid_cancel_rollback();
}

void _loop(void)
{
//TP05
    _u32 time_now = mstamp();
    _i8 cmd[64];

    if (new_movement) {
        PR05("%u: new_movement recorded at %u\n", time_now, new_movement);
        new_movement = 0;
        if (time_now - movd_last > MOVD_MIN_INTERVAL) {
#if ESP32_(2)
            snprintf(cmd, _SZ(cmd), "@beep= f:600 c:3 t:.07 p:.25 g:-20 ^");
#else
            snprintf(cmd, _SZ(cmd), "@beep=" HOST);
#endif
            mysend(cmd, STD_TARGET_HOST, STD_TARGET_PORT, 0);
            movd_last = time_now;
        }
    }
#if DEBUG > 1
    if (time_now - stat_last > ONE_SEC_IN_MILLIS) {                  // status msg each sec
        PR00("%u: reed_intr: %u pinlevel: %u errcnt: %u\n",
                                    time_now, reed_intr, reed_pinlevel_guarded, error_cnt);
        dump_err();
        stat_last = time_now;
    }
#endif
    // no myserv() and no myconn_check() any more, see myserv_task() and ur_connect() in _setup()
    fflush(stdout);
    vTaskDelay(pdMS_TO_TICKS(100));     // 10 ticks at FREERTOS_HZ 100, so still 100ms
}

void
app_main(void)
{
#if DEBUG
    PR00("TP00: %lu\n", tstamp());
#endif

    // no initArduino(): its nvs_flash_init() is in init_1st(), rollback is off in both sdkconfig references
    _setup();
    while (1) {
        _loop();
    }
}
