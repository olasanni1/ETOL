=====
Build
=====

After completing the instructions in the :ref:`setup` sections, the build tools and software dependencies should be installed. The following build instructions use the terminal on Ubuntu or the MSYS2 shell on Windows.

1. Download ETOL ::

    git clone https://github.com/olasanni1/ETOL.git

#. Change directory to ETOL ::

    cd ETOL

#. Create the build folder ::

    mkdir build && cd build

#. The build files are generated with CMake. This program finds the header files and library files that ETOL's components need. If cmake is unable to find the required files for a ETOL component, it skips it. CMake also displays information that provides insight as to why a ETOL component is skipped. ::

    cmake ../src -G "Unix Makefiles"

#. By default the build files for the ETOL documentation are not generated, but this can be changed with cmake. ::

    cmake ../src -G "Unix Makefiles" -DBUILD_DOCS=ON

#. Build ETOL ::

    make all

#. If the build documentation feature is enabled, the documentation can be built ::

    make docs
