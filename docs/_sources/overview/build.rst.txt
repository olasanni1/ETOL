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

#. The build files are generated with CMake. This program finds the header files and library files that ETOL's components need. If CMake is unable to find the required files for a ETOL component, it skips it. CMake also displays information that provides insight as to why a ETOL component is skipped. ::

    cmake ../src -G "Unix Makefiles" -DCMAKE_BUILD_TYPE=Release

#. If an eSolver requires Python3, such as eDymos, the filepath of the Python3 interpreter should be specified. For example,

   a. On an Ubuntu 18.04 machine use ::

        cmake ../src -G "Unix Makefiles" -DCMAKE_BUILD_TYPE=Release -DPYTHON_EXECUTABLE=/usr/bin/python3

   #. Whereas, on a Windows machine with MSYS2 use ::

        cmake ../src -G "Unix Makefiles" -DCMAKE_BUILD_TYPE=Release -DPYTHON_EXECUTABLE=/mingw64/bin/python3

   .. Note:: The PYTHONHOME environment variable may need to be set. If so, set it to the directory of the Python3 binary. For example, on Windows with MSYS2, this directory should be ``C:\msys64\mingw64``.

#. By default the build files for the ETOL documentation are not generated, but this can be changed with cmake. ::

    cmake ../src -G "Unix Makefiles" -DCMAKE_BUILD_TYPE=Release -DBUILD_DOCS=ON

#. Build ETOL

   .. code-block:: none

      make all

#. If the build documentation feature is enabled, the documentation can be built ::

    make docs
