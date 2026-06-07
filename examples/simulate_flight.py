"""This script runs both 1DOF and 3DOF simulations, extracts all state variables,
and displays every single plot on a single page using a unified subplot layout."""

import math
import plotly.graph_objects as go
from plotly.subplots import make_subplots
from hprm import Rocket, OdeMethod


# Create your rocket, this is roughly what a heavier 6-inch diameter rocket might look like, but you
# can change these values to see how they affect the apogee prediction
rocket = Rocket(
    mass=17.0,
    cd=0.39,
    area_drag=0.0182,
    area_lift=0.0182,
    moment_of_inertia=11.5,
    stab_margin_dimensional=0.5,
    cl_a=0.2,
)

# Runs a 1dof simulation and returns the time steps and state matrix as NumPy arrays.
# The state matrix has 2 columns: [altitude, velocity]
time_1dof, state_1dof = rocket.simulate_flight_1dof(
    initial_height=0.0,
    initial_velocity=150.0,
    integration_method=OdeMethod.RK45,
)

# You can access the altitude and velocity data from the 1DOF state matrix by slicing
alt_1dof = state_1dof[:, 0]
vel_1dof = state_1dof[:, 1]

# Runs a 3DOF simulation. We launch at 88 degrees (tilted 2 degrees) so we # can actually see the
# horizontal and rotational elements react.
# The state matrix has 6 columns: [x, y, theta, vx, vy, omega]
time_3dof, state_3dof = rocket.simulate_flight_3dof(
    initial_height=0.0,
    initial_velocity=150.0,
    initial_angle=math.radians(88),
    integration_method=OdeMethod.RK45,
)

# You can access the full 3DOF state space by slicing each column of the matrix
x_3dof = state_3dof[:, 0]
y_3dof = state_3dof[:, 1]
theta_3dof = state_3dof[:, 2]
vx_3dof = state_3dof[:, 3]
vy_3dof = state_3dof[:, 4]
omega_3dof = state_3dof[:, 5]

# Plots the flights
fig = make_subplots(
    rows=4,
    cols=2,
    subplot_titles=(
        "1DOF: Altitude (m)",
        "1DOF: Vertical Velocity (m/s)",
        "3DOF: Horizontal Position x (m)",
        "3DOF: Vertical Position y (m)",
        "3DOF: Horizontal Velocity vx (m/s)",
        "3DOF: Vertical Velocity vy (m/s)",
        "3DOF: Pitch Angle theta (rad)",
        "3DOF: Angular Velocity omega (rad/s)",
    ),
)
fig.add_trace(go.Scatter(x=time_1dof, y=alt_1dof, mode="lines", name="1DOF Alt"), row=1, col=1)
fig.add_trace(go.Scatter(x=time_1dof, y=vel_1dof, mode="lines", name="1DOF Vel"), row=1, col=2)
fig.add_trace(go.Scatter(x=time_3dof, y=x_3dof, mode="lines", name="3DOF x"), row=2, col=1)
fig.add_trace(go.Scatter(x=time_3dof, y=y_3dof, mode="lines", name="3DOF y"), row=2, col=2)
fig.add_trace(go.Scatter(x=time_3dof, y=vx_3dof, mode="lines", name="3DOF vx"), row=3, col=1)
fig.add_trace(go.Scatter(x=time_3dof, y=vy_3dof, mode="lines", name="3DOF vy"), row=3, col=2)
fig.add_trace(go.Scatter(x=time_3dof, y=theta_3dof, mode="lines", name="3DOF theta"), row=4, col=1)
fig.add_trace(go.Scatter(x=time_3dof, y=omega_3dof, mode="lines", name="3DOF omega"), row=4, col=2)
fig.update_layout(
    title_text="Comprehensive 1DOF vs 3DOF Flight Profile Analysis", height=1100, showlegend=False
)
fig.show()
