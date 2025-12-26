import math
import numpy as np
import matplotlib.pyplot as plt
from time import perf_counter
from hprm import Rocket, AdaptiveTimeStep, ModelType, OdeMethod

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
    initial_angle = math.pi - .1

    ats = AdaptiveTimeStep.default()
    iout = False

    # Run the simulation
    ats.absolute_error_tolerance = 1.0e-1
    ats.relative_error_tolerance = 1.0e-1
    tstart = perf_counter()
    simdata = test_vehicle.simulate_flight(initial_height, initial_velocity, ModelType.ThreeDOF, OdeMethod.RK45, ats, initial_angle, iout)
    tend = perf_counter()
    t1 = tend-tstart

    # Run the simulation
    ats.absolute_error_tolerance = 1.0e-4
    ats.relative_error_tolerance = 1.0e-4
    tstart = perf_counter()
    simdata = test_vehicle.simulate_flight(initial_height, initial_velocity, ModelType.ThreeDOF, OdeMethod.RK45, ats, initial_angle, iout)
    tend = perf_counter()
    t2 = tend-tstart
    

    # Run the simulation
    ats.absolute_error_tolerance = 1.0e-6
    ats.relative_error_tolerance = 1.0e-6
    tstart = perf_counter()
    simdata = test_vehicle.simulate_flight(initial_height, initial_velocity, ModelType.ThreeDOF, OdeMethod.RK45, ats, initial_angle, iout)
    tend = perf_counter()
    t3 = tend-tstart

    print("First Run:  both tolerances are set at E-2")
    print(f"Time Elapsed = {t1:.3e} s\n\n")
    print("Second Run: both tolerances are set at E-4")
    print(f"Time Elapsed = {t2:.3e} s\n\n")
    print("Third Run:  both tolerances are set at E-6")
    print(f"Time Elapsed = {t3:.3e} s\n\n")
main()
