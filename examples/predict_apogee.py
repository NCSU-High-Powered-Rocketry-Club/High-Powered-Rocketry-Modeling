"""This script shows how to use HPRM to predict the apogee of a rocket."""

from hprm import InitialState1DOF, InitialState3DOF, Rocket, OdeMethod
import math

# Create your rocket, this is roughly what a heavier 6-inch diameter rocket might look like, but you
# can change these values to see how they affect the apogee prediction
rocket = Rocket(
    mass=17.0,
    cd=0.39,
    area_drag=0.0182,
    # for 1dof, you don't need any of the following values and can just set them to 0
    # but they are required for 3dof sims
    area_lift=0.0182,
    moment_of_inertia=11.5,
    stab_margin_dimensional=0.5,
    cl_a=0.2,
)

initial_state = InitialState1DOF(initial_height=0.0, initial_velocity=150.0)

# This predicts apogee using a 1 degree of freedom model, which assumes the rocket is always
# pointing straight up and only simulates vertical motion. This is a pretty good for predicting
# apogee, but if you want to simulate more complex trajectories you can use the 3dof model instead.

# It uses RK45 as the integration method. It's recommended you use this unless you have a specific
# reason to use a different method
apogee = rocket.predict_apogee_1dof(initial_state, integration_method=OdeMethod.RK45)

print(f"1DOF Predicted Apogee: {apogee:.2f} m")

initial_state = InitialState3DOF(
    x=0.0,
    y=0.0,
    angle=math.radians(90),  # The horizon has an angle of 0, so straight up is 90 deg, or pi/2 rads
    vx=0.0,
    vy=150.0,
    angular_rate=0.0,
)

# This predicts apogee using a 3 degree of freedom model, which also simulates the rocket's
# orientation. This is helpful for if your rocket tilts a lot during flight.
apogee = rocket.predict_apogee_3dof(
    initial_state=initial_state,
    integration_method=OdeMethod.RK45,
)

print(f"3DOF Predicted Apogee: {apogee:.2f} m")
