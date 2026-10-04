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
  stopped - sends the next one, and ends once the last one has gone quiet too. its timing comes
  from the device's clock in the stream, never from the network

        sh tools/reedrec $LIFTER > idle.rec              # until interrupted
        sh tools/reedrec $LIFTER UP DOWN > updown.rec    # a full travel up, then down

- `reedana` tells what a recording holds: the scan pulses, every reed closure and, per command,
  the cycles of the movement it caused. `-c` lists every cycle, `-t` lays the runs of several
  recordings side by side

        sh tools/reedana recordings/2026-10-04-c-up-down.rec

        == recordings/2026-10-04-c-up-down.rec  ultra-shutlifter-2-000a  50.2 s  2453 edges  0 lost  3 merged  0 glitches
        scan pulses   966 dips, 0 spikes, period 16.350 ms (61.16 Hz)
                      dip   14 us    953
                      dip   17 us      1
                      dip   17 us     12   while a key is pressed
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

`recordings/` holds what this README quotes from: entity 2 on 2026-10-04, on a GW60 on the bench
that moves no shutter, its CLK LED off, the end positions set about 19 seconds of travel apart

what the reed line carries
--------------------------

measured in `recordings/`:

- the GW60 tests its reed with a scan pulse every 16.350ms (61.16Hz, jitter about 9us). an open
  reed lets nothing through, and the divider holds GPIO 21 low. a closed reed connects it to the
  CLK LED's line: with the LED off that line is high, and the scan pulse is a dip to low, 13.976us
  wide (sd 0.05us). while the GW60 sees one of its keys pressed, the dip is 16.98us - one such
  dip still came about 0.13ms after the ESP32 had let go of UP. with the LED on, the line should
  idle low and the pulse be a short spike instead - not recorded yet. the reed level in the
  status line is this idle level, sampled halfway between two pulses
- with the LED off, the line follows the reed contact itself between the scan pulses, so the
  recording shows the reed switching to the microsecond: closing, it bounces for 67us on average
  (mostly 50-75us, 243us at most, about 6 extra edges), opening is a single clean edge in 242 of
  254 cases
- the reed starts switching 0.32-0.45s after the key press begins, then cycles every 0.69-0.70s:
  about 280ms closed, 410-420ms open (duty about 40.5%), for 18.9-19.3s. every travel recorded,
  4 up and 4 down, had exactly 28 of these cycles
- in each up/down pair, going down took 0.6-0.9% longer per cycle (median 694.7-701.5ms against
  689.3-697.2ms) - on this bench, with no shutter to lift
- the second, larger gear shows every 5th cycle. when its magnet comes by in the reed's open
  phase, it closes the reed a second time, briefly (47-109ms): after cycles 2, 7, 12, 17, 22 and
  27, counted from the bottom, wherever it happens at all. and it stretches or shortens the
  regular closures (232-300ms)

`reedana -t` lays all eight travels side by side, the DOWN ones reversed, so that a row is one
shutter position - closed time per cycle in ms, `xNN` an extra closure in the open phase after it:

        sh tools/reedana -t recordings/*.rec

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

can the direction be told?
--------------------------

what these recordings say:

- not from the scan pulses: their width and period are the same in either direction, to a few
  nanoseconds and a microsecond
- over one up/down pair, the reed pattern is a function of the shutter position. a run down is
  the run up before it played backwards: the same 28 cycles, the closed times row by row within
  15ms, the extra closures in the same rows - where one shows in only one run of a pair, it is a
  weak one (62 and 66ms), close to not happening at all
- so the direction shows in the order things come in. the clearest case is where an extra closure
  sits inside its open phase: going up, all 14 came late (164-293ms after the closure before,
  90-155ms before the next), going down, all 16 came early (96-159ms after, 160-284ms before)
- but the two gears do not keep their relation. over the eight travels, within 14 minutes, the
  extra closures moved from late towards the middle of their open phase, grew from 47 to 109ms
  and showed at more positions, while the shortest regular closures moved from cycles 3, 8,
  13, ... to 5, 10, 15, ... (columns 1 to 8 above). at the end, the margin between late and early
  was down to 164/155ms against 159/160ms. the indicator holds for these recordings, but on its
  own it cannot be relied on for long
- what does not depend on any of it: a movement that starts at an end position can only go one
  way, and from there the firmware could count cycles

still open:

- recordings with the CLK LED on (`reedana` then falls back to the pulse trains, good to one scan
  period)
- a stop in mid travel, then a restart the same way and the other way
- longer series: does the relation between the gears keep drifting, and why does it drift at all?
- a real shutter on the belt, whose weight changes the speed both ways

notes
-----

- `main/mcom.h`, `main/mlcf.h` and `main/mnta.h` are shared with the author's other ESP32
  projects and are vendored here rather than referenced
- host names, MAC addresses and IP addresses throughout this repository are placeholders
