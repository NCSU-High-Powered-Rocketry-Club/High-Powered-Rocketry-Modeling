"""This script shows how varying different rocket design parameters affects the overall
3DOF flight trajectory by sweeping through values and co-plotting the results."""

import math
import plotly.graph_objects as go
from plotly.subplots import make_subplots
from hprm import Rocket, OdeMethod, InitialState3DOF

# We will sweep across five different parameters to see how they affect flight performance.
mass_values = [12.0, 17.0, 22.0]
cd_values = [0.25, 0.39, 0.55]
margin_values = [0.2, 0.5, 0.9]
moi_values = [5.0, 11.5, 20.0]
cla_values = [2.0, 6.0, 11.0]

fig = make_subplots(
    rows=5,
    cols=1,
    subplot_titles=(
        "Effect of Mass on Altitude (m)",
        "Effect of Drag Coefficient (Cd) on Altitude (m)",
        "Effect of Stability Margin on Pitch Angle theta (rad)",
        "Effect of Moment of Inertia (MOI) on Angular Velocity omega (rad/s)",
        "Effect of Cl_a (Lift Curve Slope) on Pitch Angle theta (rad)",
    ),
)

initial_state = InitialState3DOF(
    x=0.0,
    y=0.0,
    angle=math.radians(85),
    vx=0.0,
    vy=150.0,
    angular_rate=0.0,
)

# Sweep 1: Mass. We hold all other parameters at their baseline values.
for mass in mass_values:
    rocket = Rocket(
        mass=mass,
        cd=0.39,
        area_drag=0.0182,
        area_lift=0.0182,
        moment_of_inertia=11.5,
        stab_margin_dimensional=0.5,
        cl_a=11.0,
    )
    time_steps, state_matrix = rocket.simulate_flight_3dof(
        initial_state=initial_state,
        integration_method=OdeMethod.RK45,
    )

    # Plot the flights for the mass sweep onto the first row
    fig.add_trace(
        go.Scatter(
            x=time_steps, y=state_matrix[:, 1], mode="lines", name=f"{mass} kg", legend="legend1"
        ),
        row=1,
        col=1,
    )

# Sweep 2: Drag Coefficient (Cd). We vary drag while holding mass and stability baseline.
for cd in cd_values:
    rocket = Rocket(
        mass=17.0,
        cd=cd,
        area_drag=0.0182,
        area_lift=0.0182,
        moment_of_inertia=11.5,
        stab_margin_dimensional=0.5,
        cl_a=11.0,
    )
    time_steps, state_matrix = rocket.simulate_flight_3dof(
        initial_state=initial_state,
        integration_method=OdeMethod.RK45,
    )

    # Plot the flights for the drag sweep onto the second row
    fig.add_trace(
        go.Scatter(
            x=time_steps, y=state_matrix[:, 1], mode="lines", name=f"Cd = {cd}", legend="legend2"
        ),
        row=2,
        col=1,
    )

# Sweep 3: Stability Margin. We track the pitch angle (theta) to see how stiffness changes.
for margin in margin_values:
    rocket = Rocket(
        mass=17.0,
        cd=0.39,
        area_drag=0.0182,
        area_lift=0.0182,
        moment_of_inertia=11.5,
        stab_margin_dimensional=margin,
        cl_a=11.0,
    )
    time_steps, state_matrix = rocket.simulate_flight_3dof(
        initial_state=initial_state,
        integration_method=OdeMethod.RK45,
    )

    # Plot the flights for the stability sweep onto the third row
    fig.add_trace(
        go.Scatter(
            x=time_steps,
            y=state_matrix[:, 2],
            mode="lines",
            name=f"Margin = {margin}m",
            legend="legend3",
        ),
        row=3,
        col=1,
    )

# Sweep 4: Moment of Inertia (MOI). We track angular velocity to see resistance to rotation changes.
for moi in moi_values:
    rocket = Rocket(
        mass=17.0,
        cd=0.39,
        area_drag=0.0182,
        area_lift=0.0182,
        moment_of_inertia=moi,
        stab_margin_dimensional=0.5,
        cl_a=11.0,
    )
    time_steps, state_matrix = rocket.simulate_flight_3dof(
        initial_state=initial_state,
        integration_method=OdeMethod.RK45,
    )

    # Plot the flights for the MOI sweep onto the fourth row
    fig.add_trace(
        go.Scatter(
            x=time_steps, y=state_matrix[:, 5], mode="lines", name=f"MOI = {moi}", legend="legend4"
        ),
        row=4,
        col=1,
    )

# Sweep 5: Lift Curve Slope (cl_a). We track pitch angle to see correction response speed.
for cla in cla_values:
    rocket = Rocket(
        mass=17.0,
        cd=0.39,
        area_drag=0.0182,
        area_lift=0.0182,
        moment_of_inertia=11.5,
        stab_margin_dimensional=0.5,
        cl_a=cla,
    )
    time_steps, state_matrix = rocket.simulate_flight_3dof(
        initial_state=initial_state,
        integration_method=OdeMethod.RK45,
    )

    # Plot the flights for the cl_a sweep onto the fifth row
    fig.add_trace(
        go.Scatter(
            x=time_steps, y=state_matrix[:, 2], mode="lines", name=f"cl_a = {cla}", legend="legend5"
        ),
        row=5,
        col=1,
    )

fig.update_layout(
    title_text="Rocket Parameter Sensitivity Analysis (3DOF)",
    height=1500,
    legend1=dict(title="Mass Variations", yanchor="top", y=0.98, xanchor="left", x=1.02),
    legend2=dict(title="Drag Variations", yanchor="top", y=0.78, xanchor="left", x=1.02),
    legend3=dict(title="Stability Variations", yanchor="top", y=0.58, xanchor="left", x=1.02),
    legend4=dict(title="MOI Variations", yanchor="top", y=0.38, xanchor="left", x=1.02),
    legend5=dict(title="cl_a Variations", yanchor="top", y=0.18, xanchor="left", x=1.02),
)

fig.show()
