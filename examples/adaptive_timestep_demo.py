"""This script shows how to customize adaptive timestep error tolerances to compare
simulation speeds and the total number of generated data points."""

import math
from time import perf_counter
from hprm import Rocket, AdaptiveTimeStep, OdeMethod


rocket = Rocket(
    mass=10.0,
    cd=0.3,
    area_drag=0.005,
    area_lift=0.05,
    moment_of_inertia=5.0,
    stab_margin_dimensional=0.5,
    cl_a=0.2,
)

# Adaptive time stepping is exclusive to OdeMethod.RK45. If you do not specify# a timestep
# configuration, it defaults to AdaptiveTimeStep.default(). For fixed solvers like RK3 and Euler,
# the simulator ignores these tolerances and applies a standard fixed timestep of 0.01 seconds.
ats = AdaptiveTimeStep.default()

initial_height = 0.0
initial_velocity = 100.0
initial_angle = math.radians(88)

# First we try some looser tolerances
ats.absolute_error_tolerance = 1.0
ats.relative_error_tolerance = 1.0

# Profile execution speed by running without data logging active
start_time = perf_counter()
rocket.simulate_flight_3dof(
    initial_height=initial_height,
    initial_velocity=initial_velocity,
    initial_angle=initial_angle,
    integration_method=OdeMethod.RK45,
    timestep_config=ats,
)
duration_1 = perf_counter() - start_time

# Run a second time with data logging to check the final array size
time_steps_1, _ = rocket.simulate_flight_3dof(
    initial_height=initial_height,
    initial_velocity=initial_velocity,
    initial_angle=initial_angle,
    integration_method=OdeMethod.RK45,
    timestep_config=ats,
)

# Next we try the default tolerances of 1.0e-2 to see how much more accurate it is and how that
# affects the number of points generated
ats.absolute_error_tolerance = 1.0e-2
ats.relative_error_tolerance = 1.0e-2

start_time = perf_counter()
rocket.simulate_flight_3dof(
    initial_height=initial_height,
    initial_velocity=initial_velocity,
    initial_angle=initial_angle,
    integration_method=OdeMethod.RK45,
    timestep_config=ats,
)
duration_2 = perf_counter() - start_time

time_steps_2, _ = rocket.simulate_flight_3dof(
    initial_height=initial_height,
    initial_velocity=initial_velocity,
    initial_angle=initial_angle,
    integration_method=OdeMethod.RK45,
    timestep_config=ats,
)

# Finally we try some very tight tolerances to see how much more accurate it is and how that
# affects the number of points generated.
ats.absolute_error_tolerance = 1.0e-4
ats.relative_error_tolerance = 1.0e-4

start_time = perf_counter()
rocket.simulate_flight_3dof(
    initial_height=initial_height,
    initial_velocity=initial_velocity,
    initial_angle=initial_angle,
    integration_method=OdeMethod.RK45,
    timestep_config=ats,
)
duration_3 = perf_counter() - start_time

time_steps_3, _ = rocket.simulate_flight_3dof(
    initial_height=initial_height,
    initial_velocity=initial_velocity,
    initial_angle=initial_angle,
    integration_method=OdeMethod.RK45,
    timestep_config=ats,
)

# Print out the profiling metrics to analyze the adaptive step behavior
print(f"Run 1 (Tolerances: 1.0e-0) | Time: {duration_1:.3e} s | Points: {len(time_steps_1)}")
print(f"Run 2 (Tolerances: 1.0e-2) | Time: {duration_2:.3e} s | Points: {len(time_steps_2)}")
print(f"Run 3 (Tolerances: 1.0e-4) | Time: {duration_3:.3e} s | Points: {len(time_steps_3)}")
