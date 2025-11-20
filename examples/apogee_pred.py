import math
import numpy as np
import matplotlib.pyplot as plt
import time
import hprm


def main():
    print("Testing out the High Powered Rocket Modeling Program")
    print("\n\nFirst Run: Error tol = 1.0")

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

    ats.absolute_error_tolerance = 1.0
    ats.relative_error_tolerance = 1.0
    ode = hprm.OdeMethod.RK45(ats)

    state_info = hprm.PyState(id.PS_1_DOF) # 3DoF

    # Note: It's hard to make the model imputs general / textual because
    #           they change with different models. For not intended use case
    #           is to have a translation table with the different configs
    state_info.u1 = [0.0, 100.0]
    state_info.u3 = [0.0, 0.0, 0.0,
                     0.0, 100.0, 0.0]
    
    # Run the simulation
    start = time.perf_counter()
    apogee_1 = hprm.sim_apogee(test_vehicle, state_info, ode)
    end = time.perf_counter()
    print(f"Apogee: {apogee_1:.3f}")
    print(f'Elapsed time: {end - start:.2e} seconds')




    print("\n\nSecond Run: Error tol = 0.1")
    # Run the simulation
    ats.absolute_error_tolerance = 0.1
    ats.relative_error_tolerance = 0.1
    ode = hprm.OdeMethod.RK45(ats)
    state_info.set_new_model(id.PS_1_DOF) # 3DoF
    state_info.u1 = [0.0, 100.0]
    start = time.perf_counter()
    apogee_2 = hprm.sim_apogee(test_vehicle, state_info, ode)
    end = time.perf_counter()
    print(f"Apogee: {apogee_2:.3f}")
    print(f'Elapsed time: {end - start:.2e} seconds')
    

    print("\n\nThird Run: Error tol = 0.01")
    # Run the simulation
    ats.absolute_error_tolerance = 0.01
    ats.relative_error_tolerance = 0.01
    ode = hprm.OdeMethod.RK45(ats)
    state_info.set_new_model(id.PS_1_DOF) # 3DoF
    state_info.u1 = [0.0, 100.0]
    start = time.perf_counter()
    apogee_3 = hprm.sim_apogee(test_vehicle, state_info, ode)
    end = time.perf_counter()
    print(f"Apogee: {apogee_3:.3f}")
    print(f'Elapsed time: {end - start:.2e} seconds')

main()
