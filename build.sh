cp -rf src/boot.py micropython/ports/rp2/modules
cp -rf src/main.py micropython/ports/rp2/modules

cd micropython
make -C mpy-cross

cd ports/rp2
make BOARD=RPI_PICO submodules
make clean
make -j 8 BOARD=RPI_PICO FROZEN_MANIFEST=/Users/mjohnmills/PycharmProjects/mtmws_od/manifest.py

mkdir -p ../../../dist
cp -rf build-RPI_PICO/firmware.uf2 ../../../dist
