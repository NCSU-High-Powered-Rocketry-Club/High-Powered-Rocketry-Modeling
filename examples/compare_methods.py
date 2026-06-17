"""This script runs a 3DOF flight simulation using every available integration method
to compare their results, giving each subplot its own isolated legend box."""

import math
import plotly.graph_objects as go
from plotly.subplots import make_subplots
from hprm import InitialState3DOF, Rocket, OdeMethod


rocket = Rocket(
    mass=17.0,
    cd=0.39,
    area_drag=0.0182,
    area_lift=0.0182,
    moment_of_inertia=11.5,
    stab_margin_dimensional=0.5,
    cl_a=0.2,
)

initial_state = InitialState3DOF(
    x=0.0,
    y=0.0,
    angle=math.radians(88),
    vx=0,
    vy=150.0,
    angular_rate=0.0,
)

# These are the integration methods we currently support.
methods_to_compare = [OdeMethod.RK45, OdeMethod.RK3, OdeMethod.Euler]

fig = make_subplots(
    rows=2,
    cols=1,
    subplot_titles=("Vertical Position y (m) Comparison", "Vertical Velocity vy (m/s) Comparison"),
)

# Plots the position and velocity for each integration method
for method in methods_to_compare:
    time_steps, state_matrix = rocket.simulate_flight_3dof(
        initial_state=initial_state,
        integration_method=method,
    )

    # Extract the vertical position (y) and vertical velocity (vy) for this method
    y_3dof = state_matrix[:, 1]
    vy_3dof = state_matrix[:, 4]

    method_label = str(method)

    fig.add_trace(
        go.Scatter(x=time_steps, y=y_3dof, mode="lines", name=method_label, legend="legend1"),
        row=1,
        col=1,
    )
    fig.add_trace(
        go.Scatter(x=time_steps, y=vy_3dof, mode="lines", name=method_label, legend="legend2"),
        row=2,
        col=1,
    )

fig.update_layout(
    title_text="3DOF Flight Profile Integration Method Comparison",
    height=800,
    legend1=dict(yanchor="top", y=1, xanchor="left", x=1.02),
    legend2=dict(yanchor="top", y=0.4, xanchor="left", x=1.02),
)

fig.show()
