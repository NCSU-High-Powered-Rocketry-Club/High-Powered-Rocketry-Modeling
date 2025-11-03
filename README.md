# Tools for simulating and modeling rockets.

A Python library written in Rust for quick and efficient modeling and analysis of rocket data.

Currently it has a 1D-1Dof, and 2D-3Dof model formats, with a 3D-6Dof model format planned. The model parameters are input conditions; planned functionality is to be able to train a model to data.
The long-term vision of this project is to be a toolbox for testing out different rocket models with data-fitted and uncertainty-estimated parameters. The primary intended use case is to be a means to do on-the-ground modeling work relavent to past and future rockets; but, it will be performant enough that it could be used in-the-loop in some launch vehicle applications.


## ~Current~ Outdated Structure / Organization

### main.rs: 
  - Input / Control script, thing are defined here but nothing is really happening

### simulation.rs
  - The Simulation struct stores case information and will also eventually hold simulation output information
  - The run() method carries out the simulation: iterating the state, checking for convergence, and printing command line outputs

### rocket.rs
  - The Rocket struct contains specific information about the rocket. 
  - Current just mass, CD, refference area

### state.rs
  - Contains the State structure which manages the state space of the physical system (height, velocity, ...etc).
  - It is where the state derivatives are defined and the state space is updated at the end of numerical iterations.
  - Implimented as a parent module with submodules for each state(struct) implimented for the state(enum)
  - Additional submodule for state vector operations, implimented as a type MathVector defined in math.rs

### math.rs
  - Module file for mathmatical methods, currently mostly related to ODE, ordinary, Differential, Equation solvers.
  - These methods are what use the state derivatives to define have the state changes from one iteration to the next.
  - The Euler method is the most familiar and most basic. The RK3 TVD method is also included as an example of other ways this step can be done.
  - Implimented as a parent module with submodules for different types of maths
  - ode submodule for diff-eq related functions (Euler/RK3)
  - vec_ops submodule for defining MathVectors and vector operations.

### physics.rs
  - Module file for physics related properties and equations. This will likely be rearranged in the future into different categories.
  - Things like density, gravity, aerodynamic forces and mooments.
