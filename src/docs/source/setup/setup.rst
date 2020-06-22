.. _setup:

==========
Setup
==========

ETOL demarcates between a VGP and a solver. If the prerequisite software for the operating system is installed, then ETOL can be compiled without any errors. However, the library requires an eSolver to solve the problem. ETOL does not innately include any optimizers or algorithms. Rather, it relies on commercial or open-source optimization or path-planning software packages to solve a problem. The interface between ETOL and a third-party packages is classified as an eSolver and an eSolver is identified by the naming convention where the first letter is a lower case e and the second letter is any capital letter.

----------------

.. toctree::
   :maxdepth: 2

   overview.rst
   eSolver.rst
