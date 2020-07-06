.. _windows:

Windows Setup
=============

1. ETOL was originally developed in Ubuntu 18.04. However, with `MSYS2`_, ETOL and its prerequisite software packages can be installed on Windows because MSYS2 provides a Unix-like environment for Windows. Follow the MSYS2 installation instructions that are provided at https://www.msys2.org/

#. The MSYS2 shell uses a package manager called pacman. This `MSYS2 package manager`_ is able to install the prerequisite software packages for ETOL. Use pacman to install the default build tools ::

     pacman -S binutils diffutils git grep make patch pkg-config

#. Install the mingw64 build tools by using the following command and choose select all, when prompted ::

     pacman -S mingw-w64-x86_64-toolchain

#. After the install completes, the following command block installs additional dependencies ::

     pacman -S mingw-w64-x86_64-libtool mingw-w64-x86_64-cmake \
     mingw-w64-x86_64-dlfcn mingw-w64-x86_64-f2c mingw-w64-x86_64-python \
     mingw-w64-x86_64-python-pip mingw-w64-x86_64-python-numpy \
     mingw-w64-x86_64-glog mingw-w64-x86_64-gflags mingw-w64-x86_64-gdb \
     mingw-w64-x86_64-intel-tbb mingw-w64-x86_64-glog mingw-w64-x86_64-pkg-config \
     mingw-w64-x86_64-qt5  mingw-w64-x86_64-boost \
     mingw-w64-x86_64-tools-git mingw-w64-x86_64-gnuplot mingw-w64-x86_64-openmp \
     mingw-w64-x86_64-libxml2 mingw-w64-x86_64-ffmpeg \
     mingw-w64-x86_64-doxygen mingw-w64-x86_64-python-sphinx  \
     mingw-w64-x86_64-python-sphinx_rtd_theme mingw-w64-x86_64-python-pip \
     mingw-w64-x86_64-graphviz mingw-w64-x86_64-cgal mingw-w64-x86_64-re2

#. Note that the aforementioned packages were selected for a 64-bit operating system. The package name may need to change for a 32-bit system. After installing the mingw64 packages, a /mingw64/bin folder will exist and this path is relative to MSYS2 installation directory. This folder should be added to the user's PATH environment variable. Although the MSYS2 Shell uses the system's search path, its search can be modified by the /home/<user>/.bashrc file. For a 64-bit system, the following line was added to the end of the .bashrc file. ::

     export PATH=/mingw64/bin:${PATH}
     export LD_LIBRARY_PATH=/mingw64/lib:${LD_LIBRARY_PATH}
     export CPATH=/mingw64/include:${CPATH}


#. Since ETOL relies on gnuplot-iostream, which is not available through pacman, the `gnuplot-iostream.h`_ needs to be copied into the /mingw64/include folder.

   a. In the MSYS2 shell, navigate to the mingw64/include folder ::

        cd /mingw64/include

   b. Download the gnuplot-iostream. ::

        wget https://raw.githubusercontent.com/dstahlke/gnuplot-iostream/master/gnuplot-iostream.h

#. Since the Python package sphinx-git is not available through pacman, use pip to install it ::

     pip install sphinx-git

.. _MSYS2 package manager : https://packages.msys2.org/search

.. _gnuplot-iostream.h : https://github.com/dstahlke/gnuplot-iostream/blob/master/gnuplot-iostream.h

.. _MSYS2 : https://www.msys2.org/
