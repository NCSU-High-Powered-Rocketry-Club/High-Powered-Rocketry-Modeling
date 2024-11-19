# Tools for simulating and modeling rockets.

Currently a dinky litle 1DoF model. Biggest priority now is getting a good framework for managing models, simulations, data down.


## Current Structure / Organization

### main.rs: 
  - Input / Control script, thing are defined here but nothing is really happening

### simulation_mod.rs
  - The Simulation struct stores case information and will also eventually hold simulation output information
  - The run() method carries out the simulation: iterating the state, checking for convergence, and printing command line outputs

### rocket_mod.rs
  - The Rocket struct contains specific information about the rocket. 
  - Current just mass, CD, refference area

### state_mod.rs
  - Contains the State structure which manages the state space of the physical system (height, velocity, ...etc).
  - It is where the state derivatives are defined and the state space is updated at the end of numerical iterations.

### math_mod.rs
  - Module file for mathmatical methods, currently mostly related to ODE, ordinary, Differential, Equation solvers.
  - These methods are what use the state derivatives to define have the state changes from one iteration to the next.
  - The Euler method is the most familiar and most basic. The RK3 TVD method is also included as an example of other ways this step can be done.

### physics_mod.rs
  - Module file for physics related properties and equations. This will likely be rearranged in the future into different categories.
  - Currently it defines density, dravity, and a function for calculating aerodynamic drag.
