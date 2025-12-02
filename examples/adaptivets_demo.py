import math
import time
import numpy as np
import matplotlib.pyplot as plt
from hprm import Rocket, ModelType, OdeMethod, AdaptiveTimeStep

def main():
    print("Testing out the High Powered Rocket Modeling Program")


    # Define the Test Vehicle
    test_vehicle = Rocket(
        10.0,   # mass kg
        0.3,    # drag coefficient
        0.005,  # cross-sectional refference area
        0.05,   # lifting-surface refference area
        5.0,    # Moment of Inertia (for a 3DoF rocket)
        0.5,    # Dimensional stability margin (distance between cp and cg)
        0.2     # Derivative of lift coefficient with alpha(angle of attack)
    )

    initial_height = 0.0
    initial_velocity = 100.0

    ats = AdaptiveTimeStep()
    ats.absolute_error_tolerance = 1.0e-6
    ats.relative_error_tolerance = 1.0e-6
    start = time.perf_counter()
    test_vehicle.simulate_flight(initial_height, initial_velocity, ModelType.OneDOF, OdeMethod.RK45, ats)
    
    end = time.perf_counter()
    print("First Run: both tolerances are set at E-6")
    print(f'Elapsed time: {end - start:.2e} seconds\n')
    # Run the simulation

    # Run the simulation
    ats.absolute_error_tolerance = 1.0e-8
    ats.relative_error_tolerance = 1.0e-8
    start = time.perf_counter()
    test_vehicle.simulate_flight(initial_height, initial_velocity, ModelType.OneDOF, OdeMethod.RK45, ats)
    end = time.perf_counter()
    print("Second Run: both tolerances are set at E-8")
    print(f'Elapsed time: {end - start:.2e} seconds\n')

    # Run the simulation
    ats.absolute_error_tolerance = 1.0e-10
    ats.relative_error_tolerance = 1.0e-10
    start = time.perf_counter()
    test_vehicle.simulate_flight(initial_height, initial_velocity, ModelType.OneDOF, OdeMethod.RK45, ats)
    end = time.perf_counter()
    print("Third Run: both tolerances are set at E-10")
    print(f'Elapsed time: {end - start:.2e} seconds\n')

main()
