import math
import numpy as np
import matplotlib.pyplot as plt
import hprm


def main():
    print("Testing out the High Powered Rocket Modeling Program")
    print("First Run: both tolerances are set at E-8")

    id = hprm.PyID()

    # Define the Test Vehicle
    test_vehicle = hprm.Rocket(
        10.0,   # mass kg
        0.3,    # drag coefficient
        0.005,  # cross-sectional refference area
        0.05,   # lifting-surface refference area
        5.0,    # Moment of Inertia (for a 3DoF rocket)
        0.5,    # Dimensional stability margin (distance between cp and cg)
        0.2     # Derivative of lift coefficient with alpha(angle of attack)
    )

    #ode = hprm.OdeMethod.Euler(1e-2)

    ats = hprm.AdaptiveTimeStep()

    ats.absolute_error_tolerance = 1.0e-8
    ats.relative_error_tolerance = 1.0e-8
    ode = hprm.OdeMethod.RK45(ats)

    state_info = hprm.PyState(id.PS_1_DOF) # 3DoF

    # Note: It's hard to make the model imputs general / textual because
    #           they change with different models. For not intended use case
    #           is to have a translation table with the different configs
    state_info.u1 = [0.0, 100.0]
    state_info.u3 = [0.0, 0.0, math.pi/2.0,
                     0.0, 100.0, 0.0]
    

    # Run the simulation
    simdata = hprm.sim_apogee(test_vehicle, state_info, ode)




    print("Second Run: both tolerances are set at E-9")
    # Run the simulation
    ats.absolute_error_tolerance = 1.0e-9
    ats.relative_error_tolerance = 1.0e-9
    ode = hprm.OdeMethod.RK45(ats)
    state_info.set_new_model(id.PS_1_DOF) # 3DoF
    state_info.u1 = [0.0, 100.0]
    simdata = hprm.sim_apogee(test_vehicle, state_info, ode)
    

    print("Third Run: both tolerances are set at E-10")
    # Run the simulation
    ats.absolute_error_tolerance = 1.0e-10
    ats.relative_error_tolerance = 1.0e-10
    ode = hprm.OdeMethod.RK45(ats)
    state_info.set_new_model(id.PS_1_DOF) # 3DoF
    state_info.u1 = [0.0, 100.0]
    simdata = hprm.sim_apogee(test_vehicle, state_info, ode)

main()
