ultra-shutlifter
================

work in progress... stay tuned

what is it
----------

- an ESP32 roller-shutter lifter with reed-pulse position sensing

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

notes
-----

- `main/mcom.h`, `main/mlcf.h` and `main/mnta.h` are shared with the author's other ESP32
  projects and are vendored here rather than referenced
- host names, MAC addresses and IP addresses throughout this repository are placeholders
