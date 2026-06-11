import pytest
import numpy as np
from hprm import (
    Rocket,
    OdeMethod,
    AdaptiveTimeStep,
    FixedTimeStep,
    InitialState1DOF,
    InitialState3DOF,
)


def make_rocket() -> Rocket:
    """
    Utility factory function to instantiate a Rocket with standard test parameters.
    """
    return Rocket(
        mass=15.0,
        cd=0.5,
        area_drag=0.0182,
        area_lift=0.05,
        moment_of_inertia=5.0,
        stab_margin_dimensional=0.5,
        cl_a=0.2,
    )


@pytest.mark.parametrize(
    "initial_height, initial_velocity, ode_method, expected_apogee",
    [
        (0.0, 150.0, OdeMethod.Euler, 834.2602394788471),
        (0.0, 150.0, OdeMethod.RK45, 829.640509126735),
        (100.0, 50.0, OdeMethod.Euler, 224.16277121951634),
        (100.0, 50.0, OdeMethod.RK45, 221.78341013425936),
    ],
    ids=[
        "ground_start_1dof_euler",
        "ground_start_1dof_rk45",
        "air_start_1dof_euler",
        "air_start_1dof_rk45",
    ],
)
def test_simulation_integration_1dof(
    initial_height,
    initial_velocity,
    ode_method,
    expected_apogee,
):
    timestep = AdaptiveTimeStep.default() if ode_method == OdeMethod.RK45 else FixedTimeStep(0.1)
    rocket = make_rocket()
    state = InitialState1DOF(initial_height, initial_velocity)

    assert rocket.predict_apogee_1dof(
        state,
        ode_method,
        timestep_config=timestep,
    ) == pytest.approx(expected_apogee)


@pytest.mark.parametrize(
    "initial_height, initial_velocity, initial_angle, ode_method, expected_apogee",
    [
        (0.0, 150.0, 5.0, OdeMethod.Euler, 717.4301398012462),
        (0.0, 150.0, 5.0, OdeMethod.RK45, 754.8978591535089),
        (100.0, 50.0, 5.0, OdeMethod.Euler, 222.01259116956078),
        (100.0, 50.0, 5.0, OdeMethod.RK45, 219.86912837488353),
    ],
    ids=[
        "ground_start_3dof_euler",
        "ground_start_3dof_rk45",
        "air_start_3dof_euler",
        "air_start_3dof_rk45",
    ],
)
def test_simulation_integration_3dof(
    initial_height,
    initial_velocity,
    initial_angle,
    ode_method,
    expected_apogee,
):
    timestep = AdaptiveTimeStep.default() if ode_method == OdeMethod.RK45 else FixedTimeStep(0.1)
    rocket = make_rocket()

    # Matching previous internal construction logic:
    # x=0, y=height, angle=angle, vx=0, vy=velocity, omega=0
    state = InitialState3DOF(0.0, initial_height, initial_angle, 0.0, initial_velocity, 0.0)

    assert rocket.predict_apogee_3dof(
        state,
        ode_method,
        timestep_config=timestep,
    ) == pytest.approx(expected_apogee)


def test_simulate_flight_1dof_format():
    """
    Verifies that the NumPy array structures returned by simulate_flight_1dof
    conform to expected dimensional bounds, shapes, and structural baselines.
    """
    rocket = make_rocket()
    state = InitialState1DOF(initial_height=10.0, initial_velocity=150.0)

    time_arr, state_mat = rocket.simulate_flight_1dof(
        initial_state=state,
        integration_method=OdeMethod.Euler,
        timestep_config=FixedTimeStep(0.1),
    )

    assert time_arr.ndim == 1
    assert state_mat.ndim == 2
    assert time_arr.shape[0] == state_mat.shape[0]
    assert state_mat.shape[1] >= 2

    assert time_arr[0] == 0.0
    assert state_mat[0, 0] == pytest.approx(10.0)
    assert state_mat[0, 1] == pytest.approx(150.0)
