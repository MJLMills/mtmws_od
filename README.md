# Od
mcclintic.sphere.fx

Od is a program for the Music Thing Modular Workshop System's Computer
module that simulates the Lorenz system in order to produce loop-able 
control voltage and pulse signals that display sensitivity to initial 
conditions. "Od" is Cymraeg for "odd" or "strange". For detailed
documentation please see the [included PDF](docs/od.pdf).

The program can be flashed to the Computer module following the instructions at
the [Computer](https://www.musicthing.co.uk/Computer_Program_Cards/) page
after downloading the .uf2 binary for the latest version from the [releases
page](https://github.com/MJLMills/mtmws_od/releases) of the program repo.

The program is created using the 
[pyworkshopsystem](https://github.com/MJLMills/pyworkshopsystem) package, a
reusable tool for building programs for the Computer module using 
MicroPython.

### Building the .uf2 Binary

*NB: This has only been tested on Mac OS Sequoia 15.4. The process should work 
similarly for other operating systems but the installation of the prerequisites
will differ. This procedure is only necessary if customizing the program.*

The source code can be frozen into the micropython code within a .uf2 file, 
allowing for flashing of Computer modules with both micropython and
the extensions in the package. This process is not mature or well-tested,
so per-release, pre-built .uf2 files are provided in the release page of the 
git repo (see above)

Building requires cmake and the appropriate GNU embedded toolchain. On MacOS
these can be installed using [homebrew](https://brew.sh/):

`brew install cmake`

`brew install gcc-arm-embedded`

The bash script `clone_mp.sh` clones the micropython repo, and initializes the
submodules needed to build the rp2 port of micropython. The `build.sh` script
then builds the cross-compiler, the board/port submodules and finally the
`firmware.uf2` file, which is copied to the root dist directory. This file can
be copied to the Computer module.
