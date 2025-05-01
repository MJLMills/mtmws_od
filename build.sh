cp -rf src/boot.py micropython/ports/rp2/modules
cp -rf src/main.py micropython/ports/rp2/modules

# to prevent the startup bug, add:

#ifndef PICO_XOSC_STARTUP_DELAY_MULTIPLIER
#define PICO_XOSC_STARTUP_DELAY_MULTIPLIER 64
#endif

# (including the #) to the following file:
# micropython/lib/pico-sdk/src/boards/include/boards/pico.h

cd micropython
make -C mpy-cross

cd ports/rp2
make BOARD=RPI_PICO submodules
make clean
make -j 8 BOARD=RPI_PICO FROZEN_MANIFEST=/Users/mjohnmills/PycharmProjects/mtmws_od/manifest.py

mkdir -p ../../../dist
cp -rf build-RPI_PICO/firmware.uf2 ../../../dist
