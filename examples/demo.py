import math
import numpy as np
import matplotlib.pyplot as plt
import hprm


def main():
    print("Testing out the High Powered Rocket Modeling Program")

    id = hprm.PyID()

    # Define the Test Vehicle
    test_vehicle = hprm.Rocket(
        10.0,   # mass kg
        0.3,    # drag coefficient
        0.005,  # cross-sectional reference area
        0.05,   # lifting-surface reference area
        5.0,    # Moment of Inertia (for a 3DoF rocket)
        0.5,    # Dimensional stability margin (distance between cp and cg)
        0.2     # Derivative of lift coefficient with alpha(angle of attack)
    )

    #ode = hprm.OdeMethod.Euler(1e-2)

    ats = hprm.AdaptiveTimeStep()
    ats.absolute_error_tolerance = 0.1
    ats.relative_error_tolerance = 0.1

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




    # Run the simulation
    state_info.set_new_model(id.PS_3_DOF) # 3DoF
    simdata2 = hprm.sim_apogee(test_vehicle, state_info, ode)
    

    # Extract data and put in np array
    nrow = simdata.get_len()
    ncol = state_info.nlog
    data = np.zeros((nrow, ncol), dtype=float)
    for icol in range(ncol):
        for irow in range(nrow):
            data[irow, icol]  = simdata.get_val(irow, icol)

    nrow = simdata2.get_len()
    ncol = state_info.nlog
    data2 = np.zeros((nrow, ncol), dtype=float)
    for icol in range(ncol):
        for irow in range(nrow):
            data2[irow, icol]  = simdata2.get_val(irow, icol)

    plt.plot(data[:, 0], data[:, 1])
    plt.plot(data2[:, 0], data2[:, 2])
    plt.show()

main()