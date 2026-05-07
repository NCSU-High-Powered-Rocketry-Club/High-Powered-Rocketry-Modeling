import plotly.graph_objects as go
from plotly.subplots import make_subplots

from hprm import AdaptiveTimeStep, ModelType, OdeMethod, Rocket


def main() -> None:
    # Define a simple test rocket
    rocket = Rocket(
        10.0,  # mass (kg)
        0.30,  # drag coefficient
        0.005,  # reference area for drag (m^2)
        0.05,  # reference area for lift (m^2)
        5.0,  # moment of inertia (kg*m^2)
        0.5,  # dimensional stability margin (m)
        0.2,  # lift slope (1/rad)
    )

    # Initial conditions
    initial_height = 0.0
    initial_velocity = 100.0

    # In the current 3DOF model, theta=0 rad corresponds to pointing +Y (up).
    # Give it a small tilt so the 2D trajectory is more interesting.
    initial_angle = 0.15

    # RK45 uses an adaptive timestep configuration
    ats = AdaptiveTimeStep.default()
    ats.absolute_error_tolerance = 1e-3
    ats.relative_error_tolerance = 1e-3

    # IMPORTANT: log_output=True gives you the full timeseries (otherwise you only get the final row).
    simdata = rocket.simulate_flight(
        initial_height,
        initial_velocity,
        ModelType.ThreeDOF,
        OdeMethod.RK45,
        ats,
        initial_angle,
        print_output=False,
        log_output=True,
    )

    n = simdata.get_len()

    # ThreeDOF log columns (see src/state/model_3dof.rs::get_logrow)
    t = [float(simdata.get_val(i, 0)) for i in range(n)]
    x = [float(simdata.get_val(i, 1)) for i in range(n)]
    y = [float(simdata.get_val(i, 2)) for i in range(n)]
    vy = [float(simdata.get_val(i, 5)) for i in range(n)]

    apogee_i = max(range(len(y)), key=y.__getitem__)
    apogee_t = t[apogee_i]
    apogee_x = x[apogee_i]
    apogee_y = y[apogee_i]

    print(f"Logged {n} points")
    print(f"Apogee: {apogee_y:.2f} m at t={apogee_t:.3f} s")
    print(f"Downrange at apogee: {apogee_x:.2f} m")

    # Simple + useful plots:
    #   1) Altitude vs time (explicitly zoomed)
    #   2) Vertical velocity vs time (apogee is where vy crosses 0)
    fig = make_subplots(
        rows=2,
        cols=1,
        shared_xaxes=True,
        subplot_titles=("Altitude vs time", "Vertical velocity vs time"),
        vertical_spacing=0.10,
    )

    line = dict(width=3)

    fig.add_trace(
        go.Scatter(x=t, y=y, name="altitude y (m)", mode="lines", line=line),
        row=1,
        col=1,
    )
    fig.add_trace(
        go.Scatter(
            x=[apogee_t],
            y=[apogee_y],
            name="apogee",
            mode="markers",
            marker=dict(size=10, symbol="diamond"),
        ),
        row=1,
        col=1,
    )

    fig.add_trace(
        go.Scatter(x=t, y=vy, name="vy (m/s)", mode="lines", line=line),
        row=2,
        col=1,
    )

    # Reference lines
    fig.add_vline(
        x=apogee_t,
        line_width=1,
        line_dash="dash",
        line_color="rgba(0,0,0,0.35)",
        row=1,
        col=1,
    )
    fig.add_vline(
        x=apogee_t,
        line_width=1,
        line_dash="dash",
        line_color="rgba(0,0,0,0.35)",
        row=2,
        col=1,
    )
    fig.add_hline(
        y=0.0,
        line_width=1,
        line_dash="dot",
        line_color="rgba(0,0,0,0.35)",
        row=2,
        col=1,
    )

    # Explicit zoom so altitude plot isn't squished by autoscaling weirdness
    t0 = 0.0
    t1 = max(t[-1], 1e-9)
    y0 = min(0.0, min(y))
    y1 = apogee_y * 1.05 if apogee_y > 0 else max(y) * 1.05

    fig.update_xaxes(title_text="t (s)", range=[t0, t1], row=1, col=1)
    fig.update_xaxes(title_text="t (s)", range=[t0, t1], row=2, col=1)
    fig.update_yaxes(title_text="y (m)", range=[y0, y1], row=1, col=1)
    fig.update_yaxes(title_text="vy (m/s)", row=2, col=1)

    fig.update_layout(
        title=dict(
            text=f"HPRM 3DOF flight (RK45) — apogee {apogee_y:.1f} m @ {apogee_t:.2f} s",
            x=0.5,
        ),
        template="plotly_white",
        hovermode="x unified",
        legend=dict(orientation="h", yanchor="bottom", y=1.02, xanchor="left", x=0),
        margin=dict(l=70, r=30, t=90, b=60),
        height=700,
    )

    fig.show()


if __name__ == "__main__":
    main()
