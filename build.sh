curl -L https://github.com/micropython/micropython/releases/download/v1.24.1/micropython-1.24.1.tar.xz > micropython.tar.xz
tar -xvf micropython.tar.xz

cd micropython-1.24.1/ports/rp2
make submodules

cd ../
make -C mpy-cross

cd micropython-1.24.1/ports/rp2
make BOARD=RPI_PICO submodules
make -j 8 BOARD=RPI_PICO FROZEN_MANIFEST=/Users/mjohnmills/PycharmProjects/mtmws_od/manifest.py

ls build-RPI_PICO/firmware.uf2