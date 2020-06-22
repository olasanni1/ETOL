.. _tutorial_esolver:

Creating an eSolver
===================

An extensible solver (eSolver) performs two primary functions. It setups a VGP with parameters and functions that are loaded into the TrajectoryOptimizer object. It also interfaces with a trajectory optimizer or path-planning algorithm that can solve the VGP. An eSolver is a TrajectoryOptimizer child class. Since the TrajectoryOptimizer is a abstract class, the eSolver must implement the TrajectoryOptimizer's pure virtual functions: setup(), solve(), debug(), and close().

The setup() function uses the loaded settings to construct the VGP in a format that an algorithm can solve. The solve() function uses an algorithm to solve the VGP that is constructed by the setup() function. In addition, the solve() function also extracts the solution from the algorithm and passes it into the TrajectoryOptimizer object.

The debug() and close() functions are ancillary debug functions. The debug() function is used to print or log debug information. Whereas, the close() function forces the deallocation of objects. Typically, these two functions are not needed, but they are helpful when debugging.

ETOL is packaged with many eSolvers. These eSolvers support node-based algorithms and mathematical programming algorithms. Review the packaged  eSolvers and the API's Doxygen documentation, to understand how to create a new eSolver.
