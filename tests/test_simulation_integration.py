import pytest
from hprm import Rocket, ModelType, OdeMethod, AdaptiveTimeStep, FixedTimeStep


@pytest.mark.parametrize(
    "initial_height, initial_velocity, initial_angle, model_type, ode_method, expected_apogee",
    [
        (0.0, 150.0, 5.0, ModelType.OneDOF, OdeMethod.Euler, 834.2602394788471),
        (0.0, 150.0, 0.0, ModelType.OneDOF, OdeMethod.RK45, 829.640509126735),
        (0.0, 150.0, 5.0, ModelType.ThreeDOF, OdeMethod.Euler, 717.4301398012462),
        (0.0, 150.0, 5.0, ModelType.ThreeDOF, OdeMethod.RK45, 754.8978591535089),
        (100.0, 50.0, 0.0, ModelType.OneDOF, OdeMethod.Euler, 224.16277121951634),
        (100.0, 50.0, 0.0, ModelType.OneDOF, OdeMethod.RK45, 221.78341013425936),
        (100.0, 50.0, 5.0, ModelType.ThreeDOF, OdeMethod.Euler, 222.01259116956078),
        (100.0, 50.0, 5.0, ModelType.ThreeDOF, OdeMethod.RK45, 219.86912837488353),
    ],
    ids=[
        "ground_start_1dof_euler",
        "ground_start_1dof_rk45",
        "ground_start_3dof_euler",
        "ground_start_3dof_rk45",
        "air_start_1dof_euler",
        "air_start_1dof_rk45",
        "air_start_3dof_euler",
        "air_start_3dof_rk45",
    ],
)
def test_simulation_integration(
    initial_height,
    initial_velocity,
    initial_angle,
    model_type,
    ode_method,
    expected_apogee,
):
    timestep = (
        AdaptiveTimeStep.default()
        if ode_method == OdeMethod.RK45
        else FixedTimeStep(0.1)
    )

    rocket = Rocket(
        15.0,  # mass kg
        0.5,  # drag coefficient
        0.0182,  # cross-sectional reference area
        0.05,  # lifting-surface reference area
        5.0,  # Moment of Inertia (for a 3DoF rocket)
        0.5,  # Dimensional stability margin (distance between cp and cg)
        0.2,  # Derivative of lift coefficient with alpha(angle of attack)
    )

    assert rocket.predict_apogee(
        initial_height,
        initial_velocity,
        model_type,
        ode_method,
        timestep,
        initial_angle,
    ) == pytest.approx(expected_apogee)
