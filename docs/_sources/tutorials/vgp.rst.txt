.. _tutorial_vgp:

Creating a VGP
==============

In this tutorial, a two-dimensional single integrator vehicle guidance problem (VGP) is considered.

.. image:: ../images/vgp.png

The VGP is expressed as an optimal control problem,

.. math ::

   \underset{\textbf{u}(\cdot)}{\min}\:\mathcal{J}(\textbf{u}(\cdot)) &= \int_{0}^{t_f}\left(u_{x}^2(t) + u_{y}^2(t)\right)\\
   \text{subject to}\hspace{0.6cm}&\\
   \dot{x}(t) &= u_x(t),\: \forall t \in [0, t_f]\\
   \dot{y}(t) &= u_y(t),\: \forall t \in [0, t_f]\\
   x(0) &= x_0 \\
   y(0) &= y_0 \\
   x(t_f) &= x_f \pm \epsilon_x\\
   y(t_f) &= y_f \pm \epsilon_y \\
   \langle x(t), y(t) \rangle &\notin \text{ static obstacle,}\: \forall t \in [0, t_f]\\
   \langle x(t), y(t) \rangle &\notin \text{ moving obstacle,}\: \forall t \in [0, t_f]\\
   \langle x(t), y(t) \rangle &\in \mathcal{D},\: \forall t \in [0, t_f]\\
   \langle u_x(t), u_y(t) \rangle &\in U,\: \forall t \in [0, t_f]

where :math:`x` and :math:`y` are state variables, :math:`x_0` and :math:`y_0` are initial states for their respective variables, :math:`x_f` and :math:`y_f` are goal states, :math:`\epsilon_x` and :math:`\epsilon_y` are error tolerances, :math:`u_x(\cdot)` and :math:`u_y(\cdot)` are control functions, :math:`\mathcal{D}` and :math:`U` are compact subsets of :math:`\mathbb{R}^2`.

ETOL's TrajectoryOptimizer application programming interface (API) provides methods that set each component of the VGP. In addition, many components can be specified in an extensible markup language (XML) file and loaded at runtime. The XML file can be used to specify the parameters for the state variables, control variables, static obstacles, and moving obstacles. Let's examine an example XML configuration file that defines the parameters for the VGP.

.. code-block:: XML

   <?xml version="1.0" encoding="UTF-8"?>
   <etol nsteps="32" dt="0.50">
     <states nstates="2" rhorizon="0">
       <state name="x0" vartype="C" lower="0.00" upper="7.00" initial="1.00" terminal="5.00" tolerance="0.01"/>
       <state name="x1" vartype="C" lower="0.00" upper="7.00" initial="2.00" terminal="4.00" tolerance="0.01"/>
     </states>
     <controls ncontrols="2" rhorizon="0">
       <control name="u0" vartype="C" lower="-0.50" upper="0.50"/>
       <control name="u1" vartype="C" lower="-0.50" upper="0.50"/>
     </controls>
     <exzones nzones="2">
       <border name="exz0" ncorners="5">
         <corner x="3.20" y="2.50" z="0.00"/>
         <corner x="3.40" y="2.60" z="0.00"/>
         <corner x="3.50" y="3.40" z="0.00"/>
         <corner x="3.30" y="3.00" z="0.00"/>
         <corner x="3.10" y="3.50" z="0.00"/>
       </border>
       <border name="exz1" ncorners= "4">
         <corner x="2.20" y="2.50" z="0.00"/>
         <corner x="2.40" y="2.60" z="0.00"/>
         <corner x="2.50" y="3.40" z="0.00"/>
         <corner x="2.10" y="3.50" z="0.00"/>
      </border>
     </exzones>
   <mexzones nzones="2">
     <track name="mexz0" radius="0.50" nwaypoints="2">
       <waypoint name="pt0" t="0.00" ndatums="2">
         <datum>2.00</datum>
         <datum>2.00</datum>
       </waypoint>
       <waypoint name="pt1" t="32.00" ndatums="2">
         <datum>2.50</datum>
         <datum>2.00</datum>
       </waypoint>
     </track>
     <track name="mexz1" radius="0.50" nwaypoints="2">
       <waypoint name="pt0" t="0.00" ndatums="2">
         <datum>1.00</datum>
         <datum>4.00</datum>
       </waypoint>
       <waypoint name="pt1" t="32.00" ndatums="2">
         <datum>1.00</datum>
         <datum>3.00</datum>
       </waypoint>
     </track>
     </mexzones>
   </etol>

Let's examine each part of the problem, starting with the discretization parameters.

.. code-block:: XML

   <etol nsteps="32" dt="0.50">

The nsteps parameter is the number of time steps that occur after the initial state. The dt parameter is time step size. From these two parameters, the final time :math:`t_f` is computed with :math:`t_f = nsteps \times dt`

.. code-block:: XML

   <states nstates="2" rhorizon="0">
     <state name="x0" vartype="C" lower="0.00" upper="7.00" initial="1.00" terminal="5.00" tolerance="0.01"/>
     <state name="x1" vartype="C" lower="0.00" upper="7.00" initial="2.00" terminal="4.00" tolerance="0.01"/>
   </states>

The nstates parameters is an upper limit on the number of states that are loaded into memory. The rhorizon parameter means reverse horizon and it specifies how many prior time steps are needed when computing the derivatives. There are two states with names x0 and x1. The vartypes means variable types and a value 'C' means continuous, 'B' means binary, 'I' means integer. Each state also has a lower bound, upper bound, initial value, terminal value, and error tolerance.

.. code-block:: XML

   <controls ncontrols="2" rhorizon="0">
     <control name="u0" vartype="C" lower="-0.50" upper="0.50"/>
     <control name="u1" vartype="C" lower="-0.50" upper="0.50"/>
   </controls>

The control requires fewer parameters. The only new parameter is ncontrols. This parameter is an upper limit on the number of controls that are loaded into memory.

.. code-block:: XML

   <exzones nzones="2">
     <border name="exz0" ncorners="5">
       <corner x="3.20" y="2.50" z="0.00"/>
       <corner x="3.40" y="2.60" z="0.00"/>
       <corner x="3.50" y="3.40" z="0.00"/>
       <corner x="3.30" y="3.00" z="0.00"/>
       <corner x="3.10" y="3.50" z="0.00"/>
     </border>
     <border name="exz1" ncorners= "4">
       <corner x="2.20" y="2.50" z="0.00"/>
       <corner x="2.40" y="2.60" z="0.00"/>
       <corner x="2.50" y="3.40" z="0.00"/>
       <corner x="2.10" y="3.50" z="0.00"/>
     </border>
   </exzones>

The static obstacles are defined by exzones, which stands for exclusion zones. The nzones parameter is an upper limit on the number of exzones to load. Each exzone is defined by its border and a border is defined by a ordered list of corners that are connected to form polygon. To form this polygon, the last and first corners are connected. The ncorners parameters is an upper limit on the number of corners to load for a exzone. The exzone is mainly used to define the location of static physical obstacles relative to an arbitrary inertial frame.

.. code-block:: XML

   <mexzones nzones="2">
     <track name="mexz0" radius="0.50" nwaypoints="2">
       <waypoint name="pt0" t="0.00" ndatums="2">
         <datum>2.00</datum>
         <datum>2.00</datum>
       </waypoint>
       <waypoint name="pt1" t="32.00" ndatums="2">
         <datum>2.50</datum>
         <datum>2.00</datum>
       </waypoint>
     </track>
     <track name="mexz1" radius="0.50" nwaypoints="2">
       <waypoint name="pt0" t="0.00" ndatums="2">
         <datum>1.00</datum>
         <datum>4.00</datum>
       </waypoint>
       <waypoint name="pt1" t="32.00" ndatums="2">
         <datum>1.00</datum>
         <datum>3.00</datum>
       </waypoint>
     </track>
   </mexzones>

The moving obstacles are defined by mexzones, which stands for moving exclusion zones. A mexzone is a closed ball with a fixed radius and a center that follows a track. This track is defined by a time interpolation of waypoints. the nwaypoints parameter is an upper limit on the number of waypoints for a track. Each waypoint is defined by an ordered list of scalars and each scalar is called datum. The ndatums is an upper limit on the number of datum to load. A key benefit of the mexzones is that it is not limited to spatial obstacles. This feature can be used to specify obstacles to n-dimensional state state space.

The VGP in this tutorial also has an objective function, time derivative functions, and constraint functions. These functions should be formed from the parameters that are loaded into a TrajectoryOptimizer object. In addition, these functions are passed to eSolvers through an TrajectoryOptimizer object. Consequently, these functions must be created with the eSolver datatypes. The ETOL examples demonstrate how these functions should be tailored for a specific eSolver.
