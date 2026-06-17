import math
from time import perf_counter
from hprm import Rocket, AdaptiveTimeStep, OdeMethod


def main():
    print("Testing out the High Powered Rocket Modeling Program")

    # Define the Test Vehicle
    test_vehicle = Rocket(
        10.0,  # mass kg
        0.3,  # drag coefficient
        0.005,  # cross-sectional refference area
        0.05,  # lifting-surface refference area
        5.0,  # Moment of Inertia (for a 3DoF rocket)
        0.5,  # Dimensional stability margin (distance between cp and cg)
        0.2,  # Derivative of lift coefficient with alpha(angle of attack)
    )

    # Common initial conditions
    initial_height = 0.0
    initial_velocity = 100.0
    initial_angle = math.pi - 0.1

    # Timestep method object
    ats = AdaptiveTimeStep.default()

    # Methodology:
    # First Run without logging for profiling the mthod
    # Then Run with logging to show the accuracy of the apogee obtained

    # Run the simulation
    ats.absolute_error_tolerance = 1.0e-0
    ats.relative_error_tolerance = 1.0e-0
    tstart = perf_counter()
    _ = test_vehicle.simulate_flight_3dof(
        initial_height,
        initial_velocity,
        initial_angle,
        OdeMethod.RK45,
        ats,
    )
    tend = perf_counter()
    t1 = tend - tstart
    _ = test_vehicle.simulate_flight_3dof(
        initial_height,
        initial_velocity,
        initial_angle,
        OdeMethod.RK45,
        ats,
        False,  # print_output
        True,  # log_output
    )

    # Run the simulation
    ats.absolute_error_tolerance = 1.0e-2
    ats.relative_error_tolerance = 1.0e-2
    tstart = perf_counter()
    _ = test_vehicle.simulate_flight_3dof(
        initial_height,
        initial_velocity,
        initial_angle,
        OdeMethod.RK45,
        ats,
    )
    tend = perf_counter()
    t2 = tend - tstart
    _ = test_vehicle.simulate_flight_3dof(
        initial_height,
        initial_velocity,
        initial_angle,
        OdeMethod.RK45,
        ats,
        False,
        True,
    )

    # Run the simulation
    ats.absolute_error_tolerance = 1.0e-4
    ats.relative_error_tolerance = 1.0e-4
    tstart = perf_counter()
    _ = test_vehicle.simulate_flight_3dof(
        initial_height,
        initial_velocity,
        initial_angle,
        OdeMethod.RK45,
        ats,
    )
    tend = perf_counter()
    t3 = tend - tstart
    _ = test_vehicle.simulate_flight_3dof(
        initial_height,
        initial_velocity,
        initial_angle,
        OdeMethod.RK45,
        ats,
        False,
        True,
    )

    print("First Run:  both tolerances are set at E-2")
    print(f"Time Elapsed = {t1:.3e} s\n\n")
    print("Second Run: both tolerances are set at E-4")
    print(f"Time Elapsed = {t2:.3e} s\n\n")
    print("Third Run:  both tolerances are set at E-6")
    print(f"Time Elapsed = {t3:.3e} s\n\n")


main()
