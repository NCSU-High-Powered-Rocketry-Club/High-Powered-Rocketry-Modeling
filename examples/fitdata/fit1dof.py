import math
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from hprm import Rocket, ModelType, OdeMethod, AdaptiveTimeStep

def main():
    
    dataf1 = pd.read_csv("./data/genesis_launch_1.csv")
    #
    df1a = dataf1.dropna(subset=["current_altitude"]).copy()
    df1a = df1a.dropna(axis=1, how='all')
    #
    id_coast = df1a["state_letter"] == 'C'
    df1a = df1a[id_coast]
    # 
    df1a["timestamp"] -= df1a["timestamp"].iloc[0]
    df1a["timestamp"] *= 1e-9
    initial_height = df1a["current_altitude"].iloc[0]
    initial_velocity = df1a["vertical_velocity"].iloc[0]
    initial_angle = None #math.pi - .1
    print(f"Initial Altitude: {initial_height}")
    print(f"Initial Velocity: {initial_velocity}")
    # TODO: 
    #    Calculate error
    #    solve optimization problem (scipy)
    #    coplot the fit

    # Run the rocket sim
    test_vehicle = Rocket(
        10.0,   # mass kg
        0.3,    # drag coefficient
        0.005,  # cross-sectional reference area
        0.05,   # lifting-surface reference area
        5.0,    # Moment of Inertia (for a 3DoF rocket)
        0.5,    # Dimensional stability margin (distance between cp and cg)
        0.2     # Derivative of lift coefficient with alpha(angle of attack)
    )


    ats = AdaptiveTimeStep()
    ats.absolute_error_tolerance = 1.0e-10
    ats.relative_error_tolerance = 1.0e-10
    simdata_rust = test_vehicle.simulate_flight(initial_height, initial_velocity, ModelType.OneDOF, OdeMethod.RK45, ats, initial_angle)
    # Extract data and put in np array
    nrow = simdata_rust.get_len()
    ncol = 4 # NLOG+1 for 1DOF
    simdata = np.zeros((nrow, ncol), dtype=float)
    for icol in range(ncol):
        for irow in range(nrow):
            simdata[irow, icol]  = simdata_rust.get_val(irow, icol)
    sdf = pd.DataFrame(data=simdata, columns=['time','height', 'speed', 'acceleration'])

    ax = sdf.plot(x='time', y=['height', 'speed'], subplots=True)
    df1a.plot(x="timestamp", y=["current_altitude","vertical_velocity"],subplots=True, ax=ax)
    plt.tight_layout()
    plt.show()

if __name__ == "__main__":
    main()
