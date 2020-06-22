.. _ubuntu:

Ubuntu Setup
============

ETOL was initially developed on an Ubuntu 18.04 system. The following command installs its build tools ::

   sudo apt-get update
   sudo apt-get install gcc g++ gfortran git patch wget pkg-config liblapack-dev


Although ETOL was initially built on Ubuntu 18.04, it was developed with cross-platform support in-mind. Consequently, `CMake`_ is used as the build generator.

.. _CMake : https://cmake.org/

After installing cmake, the prerequisite software packages can be installed with ::

   sudo apt-get update
   sudo apt-get install liblapack-dev libboost-all-dev gnuplot libgnuplot-iostream-dev libcgal-dev libre2-dev ffmpeg
