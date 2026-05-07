import math

import numpy as np
import plotly.graph_objects as go
from plotly.subplots import make_subplots

from hprm import AdaptiveTimeStep, ModelType, OdeMethod, Rocket


def simdata_to_numpy(simdata, ncols: int) -> np.ndarray:
    nrows = simdata.get_len()
    data = np.zeros((nrows, ncols), dtype=float)
    for row in range(nrows):
        for col in range(ncols):
            data[row, col] = simdata.get_val(row, col)
    return data


def main() -> None:
    rocket = Rocket(
        10.0,   # mass [kg]
        0.3,    # drag coefficient [-]
        0.005,  # drag reference area [m^2]
        0.05,   # lift reference area [m^2]
        5.0,    # moment of inertia [kg m^2]
        0.5,    # stability margin [m]
        0.2,    # dCL/d(alpha) [1/rad]
    )

    initial_height = 0.0
    initial_velocity = 120.0
    initial_angle = math.pi - 0.1

    ats = AdaptiveTimeStep.default()
    ats.absolute_error_tolerance = 1.0e-3
    ats.relative_error_tolerance = 1.0e-3

    # For 3DOF logs:
    # col 0=t, 1=x, 2=y, 3=theta, 4=vx, 5=vy, 6=omega, 7=ax, 8=ay, 9=alpha_dot
    simdata = rocket.simulate_flight(
        initial_height,
        initial_velocity,
        ModelType.ThreeDOF,
        OdeMethod.RK45,
        ats,
        initial_angle,
    )
    data = simdata_to_numpy(simdata, ncols=10)

    t = data[:, 0]
    x = data[:, 1]
    y = data[:, 2]
    theta = data[:, 3]

    fig = make_subplots(
        rows=2,
        cols=1,
        subplot_titles=("Flight trajectory (x-y)", "Altitude and attitude vs time"),
        vertical_spacing=0.12,
    )
    fig.add_trace(
        go.Scatter(x=x, y=y, mode="lines", name="Trajectory"),
        row=1,
        col=1,
    )
    fig.add_trace(
        go.Scatter(x=t, y=y, mode="lines", name="Altitude y(t)"),
        row=2,
        col=1,
    )
    fig.add_trace(
        go.Scatter(x=t, y=theta, mode="lines", name="Pitch angle θ(t)", yaxis="y3"),
        row=2,
        col=1,
    )

    fig.update_xaxes(title_text="Horizontal position x [m]", row=1, col=1)
    fig.update_yaxes(title_text="Altitude y [m]", row=1, col=1)

    fig.update_xaxes(title_text="Time [s]", row=2, col=1)
    fig.update_yaxes(title_text="Altitude y [m]", row=2, col=1)
    fig.update_layout(
        title="3DOF Flight Simulation with RK45",
        template="plotly_white",
        hovermode="x unified",
        yaxis3=dict(
            title="Pitch angle θ [rad]",
            overlaying="y2",
            side="right",
            showgrid=False,
        ),
    )

    fig.show()


if __name__ == "__main__":
    main()
