==========
Overview
==========

The Extensible Trajectory Optimization Library (ETOL) enunciation is e-tol. It is an open-source software package that solves vehicle guidance problems (VGPs). A vehicle guidance problem specifies an objective criteria with an initial state, goal state, dynamics, collision avoidance and other pertinent constraints for a vehicle. The library provides an abstraction layer between a VGP and its solver. With this library, control researchers can spend less time developing software and more time studying problem formulations or assessing trajectory optimization algorithms.

The primary motivation behind the development of ETOL is modularity and code reuse. By defining a common structure for VGPs, a swath of solvers can be utilized to solve a VGP. Conversely, a swath of VGPs can be solved by a solver. In addition, ancillary functions are defined for loading, saving, plotting and animating data.

ETOL is able to achieve this high-level of modularity and interoperability by using the object-oriented programming features in C++17. In particular, the std::any and std::function features allow generic functions that define a VGP to be passed to and used by extensible solvers, which are called eSolvers.
