.. _glpk:

eGLPK Setup
===========

`GNU Linear Programming Kit (GLPK)`_ is an open-source library that can solve mixed-integer linear programming (MILP) problems. This package is able to solve a VGP if it is formulated with linear equations. eGLPK provides an interface between ETOL and GLPK for the purpose of solving a VGP.

.. _GNU Linear Programming Kit (GLPK) : https://www.gnu.org/software/glpk/

On Ubuntu 18.04, GLPK can be installed from the terminal. ::

    sudo apt-get update
    sudo apt-get install glpk-utils

On Windows 10, GLPK can be installed from the MSYS2 shell. ::

  pacman -S mingw-w64-x86_64-glpk

If cmake is able to find the installed GLPK package, it will generate the build files eGLPK.
