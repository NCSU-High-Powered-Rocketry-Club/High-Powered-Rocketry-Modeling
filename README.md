# Tools for simulating and modeling rockets.

Currently has a 3Dof and 1Dof model, with a 6Dof model planned. Currently this code is just the mathematical solving of a given model (with given parameters). Finding optimal parameters from given data, or implimenting some predictive capability is intended for the future.


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
