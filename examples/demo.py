import math
import numpy as np
import matplotlib.pyplot as plt
from hprm import Rocket, ModelType, OdeMethod, AdaptiveTimeStep

def main():
    print("Testing out the High Powered Rocket Modeling Program")

    # Define the Test Vehicle
    test_vehicle = Rocket(
        10.0,   # mass kg
        0.3,    # drag coefficient
        0.005,  # cross-sectional reference area
        0.05,   # lifting-surface reference area
        5.0,    # Moment of Inertia (for a 3DoF rocket)
        0.5,    # Dimensional stability margin (distance between cp and cg)
        0.2     # Derivative of lift coefficient with alpha(angle of attack)
    )

    initial_height = 0.0
    initial_velocity = 100.0

    # Run the simulation
    # This would get you the entire flight data
    # simdata = test_vehicle.simulate_flight(
    #     initial_height, 
    #     initial_velocity, 
    #     ModelType.OneDOF, 
    #     IntegrationMethod.RK45
    # )
    ats = AdaptiveTimeStep()
    ats.absolute_error_tolerance = 1.0
    test_vehicle.simulate_flight(initial_height, initial_velocity, ModelType.OneDOF, OdeMethod.RK45, ats)


    # # Extract data and put in np array
    # nrow = simdata.get_len()
    # ncol = 3 # NLOG for 1DOF
    # data = np.zeros((nrow, ncol), dtype=float)
    # for icol in range(ncol):
    #     for irow in range(nrow):
    #         data[irow, icol]  = simdata.get_val(irow, icol)

    # plt.plot(data[:, 0], data[:, 1])
    # plt.show()

if __name__ == "__main__":
    main()
