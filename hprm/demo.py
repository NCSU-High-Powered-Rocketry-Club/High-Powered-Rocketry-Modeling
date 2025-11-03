import math
import numpy as np
import matplotlib.pyplot as plt
import hprm


def main():
    print("Testing out the High Powered Rocket Modeling Program")

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

    ode = hprm.OdeMethod.Euler(1e-2)

    state_info = hprm.PyState(1) # 3DoF
    state_info.u1 = [0.0, 100.0]
    state_info.u3 = [0.0, 0.0, math.pi/2.0,
                     0.0, 100.0, 0.0]
    
    #simdata = hprm.SimulationData()

    # Run the simulation
    simdata = hprm.main(test_vehicle, state_info, ode)
    
    # Run the simulation
    #state_info.ndof = 3 # 3DoF
    #simdata = hprm.main(test_vehicle, state_info, ode)
    
    print("Time at iter = 500:", simdata.get_val(500, 0))

    # Extract data and put in np array
    nrow = simdata.get_len()
    ncol = state_info.nlog
    data = np.zeros((nrow, ncol), dtype=float)
    for icol in range(ncol):
        for irow in range(nrow):
            data[irow, icol] = simdata.get_val(irow, icol)

    plt.plot(data[:, 0], data[:, 1])
    plt.show()

main()
