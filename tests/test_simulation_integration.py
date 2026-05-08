import pytest
from hprm import Rocket, ModelType, OdeMethod, AdaptiveTimeStep, FixedTimeStep


@pytest.mark.parametrize(
    "initial_height, initial_velocity, initial_angle, model_type, ode_method, expected_apogee",
    [
        (0.0, 150.0, 5.0, ModelType.OneDOF, OdeMethod.Euler, 834.8588998048522),
        (0.0, 150.0, 0.0, ModelType.OneDOF, OdeMethod.RK45, 828.8664901620618),
        (0.0, 150.0, 5.0, ModelType.ThreeDOF, OdeMethod.Euler, 717.8879668101595),
        (0.0, 150.0, 5.0, ModelType.ThreeDOF, OdeMethod.RK45, 757.7452749266654),
        (100.0, 50.0, 0.0, ModelType.OneDOF, OdeMethod.Euler, 224.2492375402945),
        (100.0, 50.0, 0.0, ModelType.OneDOF, OdeMethod.RK45, 193.50475329606496),
        (100.0, 50.0, 5.0, ModelType.ThreeDOF, OdeMethod.Euler, 222.17711767083458),
        (100.0, 50.0, 5.0, ModelType.ThreeDOF, OdeMethod.RK45, 219.59526362032514),
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
    timestep = AdaptiveTimeStep.default() if ode_method == OdeMethod.RK45 else FixedTimeStep(0.1)

    rocket = Rocket(
        15.0,  # mass kg
        0.5,  # drag coefficient
        0.0182,  # cross-sectional reference area
        0.05,  # lifting-surface reference area
        5.0,  # Moment of Inertia (for a 3DoF rocket)
        0.5,  # Dimensional stability margin (distance between cp and cg)
        0.2,  # Derivative of lift coefficient with alpha(angle of attack)
    )

    assert (
        rocket.predict_apogee(
            initial_height,
            initial_velocity,
            model_type,
            ode_method,
            timestep,
            initial_angle,
        )
        == pytest.approx(expected_apogee)
    )
