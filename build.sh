cd modules/micropython/ports/rp2
make submodules

cd ../
make -C mpy-cross

cd modules/micropython/ports/rp2
make BOARD=RPI_PICO submodules
make -j 8 BOARD=RPI_PICO FROZEN_MANIFEST=/Users/mjohnmills/PycharmProjects/mtmws_od/manifest.py

ls build-RPI_PICO/firmware.uf2