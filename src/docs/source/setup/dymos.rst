.. _dymos_setup:

eDymos Setup
============

`Dymos`_ solves optimal control problems. It is an open-source Python package and it was created by the National Aeronautics and Space Administration (NASA).

.. _Dymos : https://openmdao.github.io/dymos/

The following instructions provides Ubuntu and Windows instructions. The instructions enumerated with "a." are for Ubuntu and "b." are for Windows. The Windows instruction in the :ref:`windows` section should be completed before completing these following setup instructions.

1. Install `OpenMDAO`_.

   a. On Ubuntu, OpenMDAO can be installed with pip ::

        pip install openmdao[all]

   #. Whereas on Windows, OpenMDAO can be installed with pacman ::

        mingw-w64-x86_64-python-openmdao

.. _OpenMDAO : https://github.com/OpenMDAO/OpenMDAO

2. Install Dymos. For both Ubuntu and Windows, Dymos can be installed with pip ::

     pip install git+https://github.com/OpenMDAO/dymos.git

3. Since Dymos is a Python package and ETOL is a C++ library, eDymos uses `Pybind11`_ to seamlessly integrate Python and C++ code.

   a. On Ubuntu, Pybind11 is installed with pip ::

        pip install pybind11

   #. Whereas on Windows, pacman is used to install Pybind11 ::

        pacman -S mingw-w64-x86_64-pybind11

.. _Pybind11 : https://pybind11.readthedocs.io/en/stable/intro.html
