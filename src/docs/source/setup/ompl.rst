.. _ompl:

eOMPL Setup
===========

The `Open Motion Planning Library (OMPL)`_ is an open-source library that contains many sampling-based motion planning libraries. Sampling-based algorithms are able to solve VGPs. eOMPL provides an interface between ETOL and GLPK for the purpose of solving a VGP.

.. _Open Motion Planning Library (OMPL) : https://ompl.kavrakilab.org/

The OMPL website provides an `Ubuntu OMPL installation script`_. Although this same website provides Microsoft Windows installation instructions, these instructions will not be used because they do not use MSYS2.

.. _Ubuntu OMPL installation script : https://ompl.kavrakilab.org/installation.html

Windows Setup
-------------

The following steps are modified version of the Linux (generic) installation instructions at the OMPL website.

1. If MSYS2 is not installed, complete the instructions in the :ref:`windows` section.

#. Install prerequisite software with pacman. ::

    pacman -S mingw-w64-x86_64-eigen3 mingw-w64-x86_64-libyaml mingw-w64-x86_64-yaml-cpp

#. Install prerequisite python packages with pip. ::

     pip install -vU https://github.com/CastXML/pygccxml/archive/develop.zip pyplusplus

#. Install libccd

   a. Open a MSYS Shell

   #. Download libccd ::

        git clone https://github.com/danfis/libccd.git

   #. Change directory to libccd ::

        cd libccd

   #. Generate build files ::

        cmake -G "Unix Makefiles" -DENABLE_DOUBLE_PRECISION=ON \
        -DBUILD_SHARED_LIBS=ON -DCMAKE_INSTALL_PREFIX=/c/msys64/mingw64

   #. Make and install ::

        make && make install

#. Install Open Dynamics Engine (ODE)

   a. Open a MSYS Shell

   #. Download ODE. For example for ode-0.16.1 ::

        wget https://bitbucket.org/odedevs/ode/downloads/ode-0.16.1.tar.gz

   #. Extract the files ::

        tar -xzvf ode-.16.1.tar.gz

   #. Change directory to ODE ::

        cd ode-0.16.1

   #. Create a build folder ::

        mkdir build && cd build

   #. Generate build files ::

        cmake .. -G "Unix Makefiles" -DCMAKE_INSTALL_PREFIX=/c/msys64/mingw64 \
        -DODE_WITH_LIBCCD=ON -DBUILD_SHARED_LIBS=ON -DODE_DOUBLE_PRECISION=ON \
        -DODE_WITH_DEMOS=OFF -DODE_WITH_TESTS=OFF

   #. Make and install ::

        make && make install

   #. Change the name of the "libode_double" file in the /mingw64/bin and /mingw64/lib folder to "libode".

#. Install OMPL

   a. Open a MSYS shell

   #. Download OMPL ::

        git clone https://github.com/ompl/ompl.git

   #. Change directory to ompl ::

        cd ompl

   #. Generate build folder ::

        mkdir -p build/Release

   #. Change directory to the build folder ::

        cd build/Release

   #. Configure the build ::

        cmake ../.. -G "Unix Makefiles" -DCMAKE_INSTALL_PREFIX=/c/msys64/mingw64 -DOMPL_BUILD_DEMOS=OFF

   #. Build and install ::

        make && make install
