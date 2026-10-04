ultra-shutlifter
================

work in progress... stay tuned

what is it
----------

- an ESP32 roller-shutter lifter with reed-pulse position sensing
- it sits in a RADEMACHER Superrollo GW60 belt winder and works it through the GW60's own front
  panel keys: UP (GPIO 4), DOWN (GPIO 19), SET (GPIO 18), CLK (GPIO 17) and SUN (GPIO 16)
- GPIO 21 listens to the reed contact the GW60 senses its gears with, through a divider from the
  GW60's 5V down to the ESP32's 3.3V
- TCP port 8888 takes one command per connection, answers with one status line and only then
  presses the key, e.g.

        echo DOWN | nc $LIFTER 8888

  `$LIFTER` stands for the device's address, here and below. the commands are `UP` `DOWN` `SET`
  `CLK` `SUN` (one key each), `SUNTRIG` `DIR` `UP_TIMER` `DOWN_TIMER` `UP_STOP` `DOWN_STOP`
  `FACRESET` (key combinations, see `main/ultra_shutlifter.c`), `RESTART`, `NOOP`, `OTA` and
  `@autostop` / `@autostop=<ms>`. the fifth field of the status line holds the RSSI, the reed
  level (see below), the seconds since the last movement and the autostop time

software installation
---------------------

- install [ESP-IDF](https://docs.espressif.com/projects/esp-idf/) v5.5 or later
- fill in your own access points, gateway MACs and target hosts in `main/mcfg.h`
- the values shipped there are placeholders, not working credentials

how to use it
-------------

- the sources want three defines: `ENTITY` picks the per-device block (16 to 20 are the author's
  lifters, 2 a bare test board), `PROJECT` and `SERNO` only name the firmware. the author's build
  driver generates them into a header - without it, write one

        cat > build_id.h << !
        #pragma once
        #define PROJECT "ultra-shutlifter"
        #define ENTITY  16
        #define SERNO   "0000"
        !

- then build and flash with

        export OPTS_="-include $PWD/build_id.h"
        idf.py set-target esp32
        idf.py -p /dev/ttyUSB0 flash monitor

- `OPTS_` must be exported before the first configure run, which `idf.py set-target` is; changing it
  afterwards needs an `idf.py fullclean`. passing the defines directly as `-DSERNO="0000"` does not
  work - the quotes are lost on the way through CMake and `SERNO` arrives as a number
- an `ENTITY` the sources do not know stops the build at `this may not happen`
- `buildit.cfg` holds the per-device settings used by the author's build driver

recording the reed line
-----------------------

the firmware itself takes little from the reed line: it counts the GW60's scan pulses, samples the
line level halfway between two of them - the reed level in the status line, which shows the CLK
LED, see below - and reports a movement whenever the pulses come back after a silence. to see all
the line does, there is a recorder:

- define `REEDREC_PORT` for an entity - the sources do it for entity 2, the bare test board, only -
  and connect to that port, 8889. every edge on GPIO 21 is streamed as one line, for as long as
  the connection stays open

        <t> 0|1         an edge, and the level it went to
        <t> = 0|1       the level sampled at the connect, then after every 500ms of silence
        <t> cmd <CMD>   port 8888 starts executing <CMD>, i.e. its key press begins
        <t> done <CMD>  ... and is through with it
        <t> lost <n>    the device's buffer ran over: n events are missing right before this line

  `<t>` is in microseconds since the connect, with two decimals
- the edges are timestamped in hardware. two MCPWM capture channels, one per edge direction, latch
  an 80MHz timer at the edge itself, so a timestamp is exact to 12.5ns however late the interrupt
  gets to read it - the GW60's scan pulses are only 14us wide. all of it runs on the second core,
  beside the firmware's own reed interrupt, which it leaves alone
- the one limit: a second edge of the same direction within a few microseconds - reed bounce -
  overwrites the first. the recording then shows one level twice in a row, the level after it is
  right. 16 of the 10008 edges in `recordings/` are such pairs, all of them inside a bounce
- one client at a time, a second one waits until the first has gone

two scripts in `tools/` drive it. they need gawk and the OpenBSD netcat (`nc -d`):

- `reedrec` records, and can work the lifter meanwhile: it waits a second, sends the first
  command to port 8888, waits until the reed has been quiet for 5 seconds - the motor has
  stopped - sends the next one, and ends once the last one has gone quiet too. `CMD/secs` does
  not wait: the next command follows secs after this one started, so a key can be pressed while
  the motor runs. its timing comes from the device's clock in the stream, never from the network

        sh tools/reedrec $LIFTER > idle.rec              # until interrupted
        sh tools/reedrec $LIFTER UP DOWN > updown.rec    # a full travel up, then down
        sh tools/reedrec $LIFTER UP/4 UP > stop.rec      # up, stopped again after 4 seconds

- `reedana` tells what a recording holds: the scan pulses and their widths, every reed closure
  and, per command, the cycles of the movement it caused, its wide pulses and any pause in the
  scan pulses. `-c` lists every cycle, `-t` lays full travels of several recordings side by side

        sh tools/reedana recordings/2026-10-04-c-up-down.rec

        == recordings/2026-10-04-c-up-down.rec  ultra-shutlifter-2-000a  50.2 s  2453 edges  0 lost  3 merged  0 glitches
        scan pulses   966 dips, 0 spikes, period 16.350 ms (61.16 Hz), 0 pauses
                      dip    13.98 us    953
                      dip    16.98 us      1
                      dip    16.98 us     12   while a key is pressed
        reed          62 closures, from the line level (exact)
        cmd UP        at 1.057 s, switching 390 ms later for 18.98 s
                      28 cycles, median 693.4 ms: closed 279.8 ms, open 413.0 ms, duty 40.3%
                      3 extra closures, each inside the open phase after a cycle:
                       cycle   closed   after cycle   before next
                          17     53.6         271.0          90.1   late
                          22     70.7         251.5          96.3   late
                          27     82.9         237.4         104.2   late
        cmd DOWN      at 25.505 s, switching 409 ms later for 19.20 s
                      28 cycles, median 698.8 ms: closed 284.2 ms, open 418.4 ms, duty 40.7%
                      3 extra closures, each inside the open phase after a cycle:
                       cycle   closed   after cycle   before next
                           1     94.3         106.6         230.7   early
                           6     80.9          99.7         246.6   early
                          11     67.3          95.5         259.6   early

`recordings/` holds what this README quotes from, all of entity 2 on 2026-10-04, on a GW60 on the
bench that moves no shutter, its end positions set about 19 seconds of travel apart: `a` to `d`
are eight full travels with the CLK LED off, `e` to `g` came an hour later with the LED on - `g`
is the stop and reverse test below - and `h` is one more travel up and down with the LED off
again, three hours after `a`

what the reed line carries
--------------------------

measured in `recordings/`:

- the GW60 tests its reed with a scan pulse every 16.350ms (61.16Hz, jitter about 9us). an open
  reed lets nothing through, and the divider holds GPIO 21 low. a closed reed connects it to the
  CLK LED's line. the reed level in the status line is that line's idle level, sampled halfway
  between two pulses
- with the LED off the line is high, and the scan pulse is a dip to low, 13.976us wide (sd
  0.05us). while the GW60 sees one of its keys pressed, the dip is 16.98us - one such dip still
  came about 0.13ms after the ESP32 had let go of UP
- with the LED on the line is low, and the scan pulse is a spike to high, 17.95 or 18.96us wide,
  a key press makes no difference. but while the motor runs, every few spikes is a wide one,
  74-79us in steps of 0.5us - see the direction below
- whenever the motor stops, the scan pulses pause: two are left out, one comes 43.7-43.9ms after
  the last, and 5.1-5.4ms later they are back on their grid. all five stops that happened with the
  reed closed showed it, by a key as much as at the end position - with the reed open there is
  nothing to see. the motor starts 80-100ms after its key press begins and leaves no such mark
- with the LED off, the line follows the reed contact itself between the scan pulses, so the
  recording shows the reed switching to the microsecond: closing, it bounces for 67us on average
  (mostly 50-75us, 243us at most, about 6 extra edges), opening is a single clean edge in 242 of
  254 cases
- the reed starts switching 0.32-0.45s after the key press begins, then cycles every 0.69-0.70s:
  about 280ms closed, 410-420ms open (duty about 40.5%), for 18.9-19.3s. every full travel recorded,
  6 up and 6 down, had exactly 28 of these cycles
- in each up/down pair, going down took 0.6-0.9% longer per cycle (median 694.7-701.5ms against
  689.3-697.2ms) - on this bench, with no shutter to lift
- the second, larger gear shows every 5th cycle. when its magnet comes by in the reed's open
  phase, it closes the reed a second time, briefly (47-109ms): after cycles 2, 7, 12, 17, 22 and
  27, counted from the bottom, wherever it happens at all. and it stretches or shortens the
  regular closures (232-300ms)

`reedana -t` lays all eight travels side by side, the DOWN ones reversed, so that a row is one
shutter position - closed time per cycle in ms, `xNN` an extra closure in the open phase after it:

        sh tools/reedana -t recordings/2026-10-04-[a-d]-*.rec

        closed ms per cycle, one row per shutter position (UP as recorded, DOWN reversed), xNN: an extra closure
               1  recordings/2026-10-04-a-up.rec UP@2
               2  recordings/2026-10-04-b-down.rec DOWN@2
               3  recordings/2026-10-04-c-up-down.rec UP@1
               4  recordings/2026-10-04-c-up-down.rec DOWN@26
               5  recordings/2026-10-04-d-up-down-up-down.rec UP@1
               6  recordings/2026-10-04-d-up-down-up-down.rec DOWN@26
               7  recordings/2026-10-04-d-up-down-up-down.rec UP@50
               8  recordings/2026-10-04-d-up-down-up-down.rec DOWN@75
        cycle         1         2         3         4         5         6         7         8
            1   286       287       283       284       280       280       278       278
            2   284       289       284       287       281       284 x62   282 x59   282 x96
            3   250       261       261       270       272       277       280       281
            4   288       286       287       286       288       285       285       285
            5   267       267       252       254       242       244       244       259
            6   279       278       276       276       276       277       276       278
            7   282       293       280       287       277       284 x66   273 x66   279 x96
            8   251       260       261       267       269       274       274       278
            9   292       298       293       296       293       295       289       291
           10   265       263       253       253       243       244       255       260
           11   284       288       283       285       281       284       279       283
           12   279       284       281       283       281 x65   281 x86   281 x92  279 x104
           13   255       262       263       269       268       274       270       275
           14   285       291       284       290       283       288       281       287
           15   265       269       253       258       253       253       270       271
           16   293       289       289       287       285       285       284       287
           17   281       288       281 x54   286 x67   280 x84   284 x96   279 x99  282 x106
           18   258       261       269       271       275       278       280       281
           19   277       287       273       284       271       282       269       281
           20   256       261       244       249       246       251       270       275
           21   294       294       289       289       286       288       284       286
           22   283       289       279 x71   284 x81   279 x93  285 x102  281 x105  285 x109
           23   267       272       272       277       275       281       279       284
           24   280       286       279       285       279       286       281       288
           25   242       256       232       244       241       251       261       272
           26   287       291       284       288       283       287       283       287
           27   292 x47   300 x55   287 x83   293 x94   284 x96  287 x105  281 x105  283 x108
           28   276       282       282       287       282       287       282       287

stopping and reversing in mid travel
------------------------------------

`recordings/2026-10-04-g-led-on-stops-reversals.rec`, made with

        sh tools/reedrec $LIFTER UP/4 UP DOWN/4 DOWN DOWN/3 DOWN UP/3 UP DOWN

from halfway up, the reed open just above cycle 10 (cycles counted from the bottom, as above).
pressing the key of the running direction again stops the motor - 0.62s after that 0.5s key
press began, about 0.12s after it ended:

        run               starts                   cycles     extra closure       stops
        UP, continued     open, just above 10      11 to 16   between 12 and 13   closed, at 16
        DOWN, reversed    closed, at 16            15 to 10   between 13 and 12   closed, at 10
        DOWN, continued   closed, at 10            9 to 6     between 8 and 7     open, just below 6
        UP, reversed      open, just below 6       6 to 10    between 7 and 8     closed, at 10
        DOWN, to the end  closed, at 10            9 to 1     between 8/7, 3/2    the lower end

- the reed pattern stays with the shutter position through every stop and reversal: each run
  went on from exactly where the one before had stopped, and every extra closure sat where the
  full travels have it
- a reversal starts with the closure last passed: going up again from just below cycle 6, the
  reed closed 135ms after the key press - for cycle 6 once more. continuing upwards from just
  above cycle 10, it took 414ms through the rest of the open phase to cycle 11

can the direction be told?
--------------------------

with the CLK LED on, yes - from the scan pulses themselves. while the motor runs the GW60 widens
every few of them, and how wide depends on the direction. from the test above (lines left out):

        sh tools/reedana recordings/2026-10-04-g-led-on-stops-reversals.rec

        cmd UP        at 1.075 s, switching 414 ms later for 3.48 s
                      23 wide pulses, median 77.84 us
        cmd DOWN      at 10.802 s, switching 710 ms later for 3.53 s
                      31 wide pulses, median 79.33 us
        cmd DOWN      at 20.753 s, switching 652 ms later for 2.39 s
                      31 wide pulses, median 78.84 us
        cmd UP        at 28.905 s, switching 135 ms later for 2.78 s
                      21 wide pulses, median 77.84 us
        cmd DOWN      at 37.604 s, switching 723 ms later for 6.94 s
                      59 wide pulses, median 78.84 us

- over the three LED-on recordings, from 0.25s after the motor started: going up, 88 of 92 wide
  pulses were 77.84us; going down, 258 of 261 were 78.84 or 79.34us. at rest, 3 of 1565 pulses
  were wide, all of them narrower than 77.4us
- in the first 0.25s after a start the width is still on its way - 76.84, 77.34, 77.84us, once
  78.33us - and only going down does it move on to 78.84 or 79.34us. so it looks less like a
  direction flag than like a value of the motor control that settles differently each way
- a closure lets through about 17 pulses, a few of them wide: going by the majority of its wide
  pulses, 84 of the 85 closures in motion that had any told the right direction. the one miss was
  the last closure, as the motor stopped at the lower end
- with the LED off there is nothing of the kind: no scan pulse in `a` to `d` or in `h` is wider
  than 17us, and width and period are the same in either direction, to a few
  nanoseconds and a microsecond

from the reed pattern, with the LED off or on:

- over one up/down pair, the reed pattern is a function of the shutter position. a run down is
  the run up before it played backwards: the same 28 cycles, the closed times row by row within
  15ms, the extra closures in the same rows - where one shows in only one run of a pair, it is a
  weak one (62 and 66ms), close to not happening at all
- so the direction shows in the order things come in. the clearest case is where an extra closure
  sits inside its open phase: in `a` to `d`, going up, all 14 came late (164-293ms after the
  closure before, 90-155ms before the next), going down, all 16 came early (96-159ms after,
  160-284ms before)
- but the two gears do not keep their relation. over the eight travels, within 14 minutes, the
  extra closures moved from late towards the middle of their open phase, grew from 47 to 109ms
  and showed at more positions, while the shortest regular closures moved from cycles 3, 8,
  13, ... to 5, 10, 15, ... (columns 1 to 8 above). an hour later, with the LED on, the extra
  closures after cycles 12 and 27 came late going down and early going up - the other way round
  (with the LED on these times are good to one scan period, 16ms). each one still sat at the same
  place from either side, so it is a matter of the position, and of how far the drift has got,
  not of the direction. in `h`, with the LED off again, all six extra closures showed in both
  directions, 64-113ms long and most of them near the middle of their open phase: going up, those
  after cycles 2 and 7 came late and the other four early, going down all six came late
- what does not depend on any of it: a movement that starts at an end position can only go one
  way, and from there the firmware could count cycles

still open:

- a real shutter on the belt: its weight changes the speed both ways, perhaps the wide pulses too
- whether the pause at a motor stop differs with the direction: 43.68-43.72ms the four times
  going down, 43.80 and 43.93ms the two times going up - too few to tell
- longer series: does the relation between the gears keep drifting, and why does it drift at all?

notes
-----

- `main/mcom.h`, `main/mlcf.h` and `main/mnta.h` are shared with the author's other ESP32
  projects and are vendored here rather than referenced
- host names, MAC addresses and IP addresses throughout this repository are placeholders
