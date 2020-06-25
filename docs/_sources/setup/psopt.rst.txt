.. _psopt_setup:

ePSOPT Setup
============

Similar to ETOL, `PSOPT`_ is an open source software package that relies on open source packages. The library can be obtained at https://github.com/PSOPT/psopt. It comes with an Ubuntu 18.04 install script, which also installs the prerequisite software packages. The library does not provide an install script for other operating systems, such as Windows. However, PSOPT has been successfully used on Windows with ePSOPT.

Windows Setup
-------------

Many packages need to be built from source. The primary packages are `IPOPT`_ and `ADOL-C`_. These two packages require additional open source packages too. The following steps provide instructions for building PSOPT and its dependencies on a Windows 10 machine.



1. If MSYS2 is not installed, complete the instructions in the :ref:`windows` section.

#. Install additional prerequisite software ::

    pacman -S unzip mingw-w64-x86_64-openblas mingw-w64-x86_64-lapack \
    mingw-w64-x86_64-suitesparse mingw-w64-x86_64-metis mingw-w64-x86_64-swig \
    mingw-w64-x86_64-readline mingw-w64-x86_64-libgd

#. Install ColPack with CMake.

   a. Open a MSYS2 shell

   #. Download Colpack ::

        git clone https://github.com/CSCsw/ColPack.git

   #. Edit the CMakeLists.txt file in ./ColPack/build/cmake. Replace "ColPack_headers" with "ColPack". Replace "ColPack_libs" with "cmake/ColPack". Delete "/shared_archive", "/shared_library", "/shared_runtime", "/archive", "/library", and "/runtime". Change "WIN32" to "MSVC". Change the following lines ::

        101    ${COLPACK_ROOT_DIR}/src/Recovery/*.cpp

        170    $target_link_libraries(ColPack_shared PRIVATE ${OpenMP_C_LIBRARIES})

   #. Change directory to ColPack's build directory ::

        cd ColPack/build

   #. Create a "mywork" directory ::

        mkdir mywork && cd mywork

   #. Generate the build files ::

        cmake ../cmake -DCMAKE_INSTALL_PREFIX:PATH=/mingw64 -DENABLE_OPENMP=ON -DENABLE_EXAMPLES=OFF -G "Unix Makefiles"

   #. Build the files ::

        make && make install

#. Install `ADOL-C`_

   a. Open a MSYS Shell

   #. Download ADOL-C ::

        git clone https://github.com/coin-or/ADOL-C.git

   #. Change directory to ADOL-C ::

        cd ADOL-C

   #. Edit ./ADOL-C/swig/adolc-numpy-for.i. Change all "unsigned long" to "unsigned long long"

   #. Edit ./configure.ac. On line 95, change "_lib=lib64" to "_lib=lib".

   #. Edit ./ADOL-C/swig/setup.py. Replace "lib64" to "lib". Change line 193 to ::

        libraries=['adolc','ColPack','gomp'],

   #. Run the following ::

        autoreconf -vfi

   #. Configure the build ::

        ./configure --prefix=/mingw64 --with-colpack=/mingw64 \
        --with-cflags="-O3 -fPIC -std=c99 -Wall" \
        --with-cxxflags="-O3 -std=c++11 -Wall -fPIC" --enable-sparse \
        --with-boost-libdir=/mingw64/lib --enable-ulong --enable-static=yes --enable-shared=no --with-openmp-flag="-fopenmp"

   #. Build and install ::

        make && make install

#. Install a `IPOPT Linear Solver`_. For example the `Harwell Subroutines Library`_ (HSL) was successfully used on Windows.

#. Install IPOPT such that it dynamically loads a linear solver

   a. Open a MSYS2 Shell

   #. Download Ipopt ::

        git clone https://github.com/coin-or/Ipopt.git

   #. Change directory to Ipopt ::

        cd Ipopt

   #. Modify it load such that it loads the installed linear solver's dll. For example, if HSL is used, modify ./src/contrib/LinearSolverLoader/HSLLoader.c, by changing '#define HSLLIBNAME SHAREDEXT' to '#define HSLLIBNAME "libhsl.dll"'

   #. Make a build folder ::

        mkdir build && cd build

   #. Configure the build ::

        ../configure --enable-static=yes --enable-shared=no coin_skip_warn_cxxflags=yes \
        --with-mumps=no --with-asl=no --with-hsl=no --prefix=/mingw64 \
        LDFLAGS="-Wl,--no-as-needed -ldl"

   #. Build Ipopt ::

        make -j $(($(nproc) - 1))

   #. Install Ipopt ::

        make install

#. Install a PDFLib-Lite shared library based on `MinGW32 Linux Distro Notes`_

   a. Open a MSYS2 shell

   #. Download PDFLib-Lite ::

        git clone https://github.com/Distrotech/PDFlib-Lite.git

   #. Change directory to PDFlib-Lite ::

        cd PDFlib-Lite

   #. Edit ./libs/pdcore/pc_util.h by adding the following to line 25 ::

        #undef isfinite

   #. Change line 23 in ./config/mkcommon.inc to ::

        EXE		=

   #. Configure the build ::

        ./configure --prefix=/mingw64 CFLAGS="-DPDFLIB_EXPORTS" --enable-64-bit

   #. Build and install ::

        make && make install

   #. Change directory to /mingw64/lib ::

        cd /mingw64/lib

   #. Create a shared Library ::

        gcc -shared -o libpdf.dll libpdf.a

   #. Create a gendef file

      .. code-block:: none

         gendef - libpdf.dll > libpdf.def

   #. Create a dll.a file

      .. code-block:: none

         dlltool -d libpdf.def -l libpdf.dll.a

   #. Move the dll file to the mingw64/bin directory

      .. code-block:: none

         mv ./libpdf.dll ../bin

#. Install PSOPT

   a. Open a MSYS2 shell

   #. Download PSOPT ::

        git clone https://github.com/PSOPT/psopt.git

   #. Change directory to PSOPT ::

        cd psopt

   #. Download lusol into the PSOPT directory ::

        wget --continue http://www.stanford.edu/group/SOL/software/lusol/lusol.zip

   #. Extract the lusol files ::

        unzip ./lusol.zip

   #. Edit ./Makefile.

      I. Delete lines 194 and 186, which are for $(CXSPARSE).

      #. Change the following lines ::

           9     prefix = /mingw64
           179   all: $(DMATRIX_LIBS) $(LUSOL_LIBS) $(PSOPT_LIBS)

   #. Edit ./dmatrix/lib/Makefile. Change the following lines ::

        18    IPOPTINCDIR = ${prefix}/include/coin-or
        27    CXX           = g++
        28    CC            = gcc
        29    CXXFLAGS      = -O0 -g -I$(SNOPTDIR)/cppsrc -I$(DMATRIXDIR)/include -I$(SNOPTDIR)/cppexamples -I$(PSOPTSRCDIR) -DLAPACK -DUNIX -DSPARSE_MATRIX -DUSE_SNOPT -DUSE_IPOPT -I$(CXSPARSE)/Include -I$(LUSOL) -I$(CXSPARSE)/../SuiteSparse_config -I$(IPOPTINCDIR) -fomit-frame-pointer -pipe -DNDEBUG -fPIC -DHAVE_MALLOC

   #. Edit ./PSOPT/lib/Makefile. Change the following lines ::

        16    prefix = /mingw64
        18    IPOPTINCDIR = -I${prefix}/include/coin-or
        27    CXX           = g++
        28    CC            = gcc
        29    CXXFLAGS      = -O0 -g -I${prefix}/include -I${prefix}/include/adolc -I$(DMATRIXDIR)/include -I$(SNOPTDIR)/cppexamples -I$(PSOPTSRCDIR) -DLAPACK -DWIN32 -DSPARSE_MATRIX -DUSE_IPOPT -I$(LUSOL) $(IPOPTINCDIR) -fomit-frame-pointer -pipe -DNDEBUG -fPIC -DHAVE_MALLOC -std=c++11  -DPSOPT_EXPORT

   #. Edit plot.cxx and psopt.h in ./PSOPT/src, by replacing all instances of "const string& title" to "const string title".

   #. Edit ./PSOPT/src/psopt.h.

      1. Add to beginning of lines 1111 and 1113 ::

           __declspec(dllexport)

      #. Replace lines 39 to 43 with

         .. code-block:: c

            #ifndef PSOPT_API
            #ifdef WIN32
            #ifdef PSOPT_EXPORT
            #define PSOPT_API __declspec(dllexport)
            #else
            #define PSOPT_API __declspec(dllimport)
            #endif
            #endif
            #endif

   #. Edit ./PSOPT/src/plot.h. Delete or comment out lines 61 to 65

   #. Edit ./PSOPT/src/IPOPT_interface.cxx. Replace "WIN32" with "MSVC".

   #. Edit ./PSOPT/src/NLP_interface.cxx to use the installed IPOPT linear solver. A list of supported linear solver values are provided at https://coin-or.github.io/Ipopt/OPTIONS.html#OPT_Linear_Solver. For example for HSL ma57, add the following to line 559 ::

        app->Options()->SetStringValue("linear_solver","ma57");

   #. Edit ./dmatrix/include/dmatrixv.h.

      1. Change line 116 to ::

           #define DEC_THREAD __thread

      #. First delete any instance of "=NULL" and "= Null".

      #. Replace any instance of "ntype=0"  with "ntype".

      #. Between lines 1725 to 1832 replace:

         a. "[]" with "[] = NULL"

         #. "* U" with "* U = NULL"

         #. "* V" with "* V = NULL"

         #. "* rindx" with "* rindx = NULL"

         #. "* cindx" with "* cindx = NULL"

         #. "int ntype" with "int ntype = 0"

      #. Between lines 2443 to 2484, replace:

         a. "[]" with "[] = NULL"

         #. "* U" with "* U = NULL"

         #. "* V" with "* V = NULL"

   #. Build PSOPT

      .. code-block:: none

         make all

   #. Create a PSOPT_HOME environment variable that point points the PSOPT root directory. For example, in bashrc, add the following line at the end of the file ::

        export PSOPT_HOME=/home/${USER}/psopt


.. _PSOPT : http://www.psopt.org/

.. _IPOPT : https://coin-or.github.io/Ipopt/index.html

.. _ADOL-C : https://github.com/coin-or/ADOL-C

.. _MinGW32 Linux Distro Notes : http://www.davidgis.fr/documentation/Build_Prebuilt_Toolchain_MinGW-w64_for_Linux-32bits_GCC-7.2.0_Testing/#pdflib-lite-7-0-5p3

.. _IPOPT Linear Solver : https://coin-or.github.io/Ipopt/INSTALL.html

.. _Harwell Subroutines Library : http://www.hsl.rl.ac.uk/ipopt/
