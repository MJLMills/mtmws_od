"""
The "manifest" is a python file that will be interpreted by the RP2 build process.
Typically this would be written as part of a board definition (qq wtf is that?),
but you can also write a standalone manifest file and use it with an existing board
definition.

A manifest file is a Python file containing a series of function calls.
The following variables are also available:

$(MPY_DIR) - path to the micropython repo
$(MPY_LIB_DIR) - path to the micropython-lib submodule. Prefer to use require()
$(PORT_DIR) - path to the current port (e.g. ports/stm32, we want this to be ports/rp2)
$(BOARD_DIR) - path to the current board (e.g. ports/stm32/boards/PYBV11)


Custom manifest files (like this one) should be kept under version control with the rest of the project.

ultimately will run
$ make BOARD=MYBOARD FROZEN_MANIFEST=/path/to/my/project/manifest.py

probably this will need to use the module() function in the manifest, which lets you include a single
python file as a module, e.g. module("foo.py", base_path="src/drivers")

probably also need to include the port manifest using the include() method.
the port manifest is at /Users/mjohnmills/tmp/manifest_test/micropython/ports/rp2/boards/manifest.py
which makes sense, that is why if you want to freeze code you can just put your files straight into
the rp2/modules directory which is included in the rp2/boards/manifest.py file through the line

freeze("$(PORT_DIR)/modules")
"""
# include("$(PORT_DIR)/boards/manifest.py")

"""
This is equivalent to copying the “package_path” directory to the device (except as frozen code).

If the package isn’t in the same directory as the manifest file, use base_path:
package("foo", base_path="path/to/libraries")
Will recursively include all .py files in foo, and will be frozen as foo/**/*.py.
"""
include("./modules/pyworkshopsystem/manifest.py")

module("looper.py", base_path="src")
module("model.py", base_path="src")
module("lorenz_system.py", base_path="src")
