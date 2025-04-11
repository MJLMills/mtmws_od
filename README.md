# Od
mcclintic.sphere.fx

Od is a program for the Music Thing Modular Workshop System Computer
module that simulates the Lorenz system in order to produce loop-able 
control voltage and pulse signals that display sensitivity to initial 
conditions. "Od" is Cymraeg for "odd" or "strange".

The majority of the core module code is in the 
[pyworkshopsystem](https://github.com/MJLMills/pyworkshopsystem) repo, a
reusable package for building programs for the Computer module with 
MicroPython.

A .uf2 file will be provided once features are finalized and tested and all 
packaging is complete. For now it is possible to try the module out using the 
provided main.py file (which contains the complete module code).

### Building the .uf2

This has only been tested on Mac OS Sequoia 15.4.

Building the module into a .uf2 file for distribution requires
cmake and gcc-arm-embedded, installed via homebrew. With these 
installed, the bash script `clone_mp.sh` can be executed to retrieve
the micropython source and the submodules needed to build the rp2 port.
After this, `build.sh` can be executed to build the port using the 
manifest file `manifest.py` at the root of this repo.