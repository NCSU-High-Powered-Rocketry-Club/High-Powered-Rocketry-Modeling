# Development Roadmap
This roadmap is meant to serve as a place to jot down and prioritize features to be added to the code.

## Stuff that can be worked on now

### Higher Priority
- [ ] Rename all module files to just *_mod instead of the mix that it currently is
- [ ] Add basic output
- Add additional state Spaces
    - [ ] 2 Degree of Freedom (horizontal and vertical motion)
        - Assume rocket is pointing along velocity vector
        - Add additional velocity component (incl. initial condition)
        - Sum of Forces: Drag, in opposite direction of velocity _+ Thrust if Rocket has motor burning_
    - [ ] 3 DoF (Add rotation)
        - Rotation routines to the math module
        - Rocket now had dimensional stability margin
        - Rocket now has moment of inertia
        - Add normal force and turning moment to physics module
        - cd is now a function of angle of attack
        - Sum of Forces: Drag + Lift
     
### Lower Priority
- [ ] Adaptive step size ODE methods to the math module
- [ ] Make a Checkpoint functionality: Mechanism that enforces the simulation hits certain points in time. The goal to to be able to have some parity between the simulation output and measured data without comprimising the simulation accuracy with an overly large timestep.
- [ ] Different options for calculating density and gravity
- [ ] Add a better UI during simulations
- [ ] Add ability to read in rocket motor thrust curves
- [ ] Add Motor Burn effects (changes in mass and acceleration)
- [ ] Add launch rail constraint


## Stuff for Later on
- [ ] Figure out communication with python
- [ ] Full 6 DoF state space
- [ ] 4 DoF state space (no rotation dynamics)
- [ ] Read in data for processing and comparison
- [ ] Options for modeling aerodynamic coefficients as a function of speed, angle of attack, etc...
- [ ] Auxiliary state laws: Add-Ons which are compatible with certain types of state spaces. These could be something like airbrake or flap control laws. The idea is that they are treated as separate equations (possibly also ODEs if you want to do higher fidelity electro-mechanical system modeling) which need to be updated along with the state, and can affect the state.
- [ ] Model fitting / Parameter Estimation / Uncertainty Analysis





