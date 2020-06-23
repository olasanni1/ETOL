.. _gurobi:

eGurobi Setup
=============

`Gurobi Optimizer`_ is a commercial software package that can solve many types of mathematical programming problems. eGurobi provides an interface between ETOL and Gurobi for the purpose of solving a VGP.

.. _Gurobi Optimizer : https://www.gurobi.com/


Windows Setup
-------------

After running the Gurobi installer, a C++ API interface needs to be built with the MSYS2 GNU g++ compiler.

1. If MSYS2 is not installed, complete the instructions in the :ref:`windows` section.

#. Create libgurobi_c++.a by using the solution at https://www.quora.com/How-can-I-use-Gurobi-in-Windows-with-a-G-compiler

#. Copy the libgurobi_c++.a file to the Gurobi "lib" directory.

After installing Gurobi, the ETOL CMake project attempts to find Gurobi in the default install paths. If Gurobi is not in the default install path, CMake is configured to search the path that is specified by the environment variable GUROBI_HOME. If cmake still does not find the package, then the FindGUROBI.cmake file may require modification. For example, a modification may be necessary if the name of the library in FindGUROBI.cmake is different from the name of the installed library.
