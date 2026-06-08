import pytest
from hprm import Rocket, OdeMethod, AdaptiveTimeStep, FixedTimeStep


pytestmark = pytest.mark.benchmark


def make_bench_rocket() -> Rocket:
    # This is just the typical 6 inch rocket used in the examples
    return Rocket(
        mass=17.0,
        cd=0.39,
        area_drag=0.0182,
        area_lift=0.0182,
        moment_of_inertia=11.5,
        stab_margin_dimensional=0.5,
        cl_a=0.2,
    )


def test_bench_1dof_euler(benchmark):
    """Benchmarks the 1-DOF explicit Euler solver (Fixed Step)."""
    rocket = make_bench_rocket()
    timestep = FixedTimeStep(0.01)
    benchmark(
        rocket.predict_apogee_1dof,
        initial_height=0.0,
        initial_velocity=150.0,
        integration_method=OdeMethod.Euler,
        timestep_config=timestep,
    )


def test_bench_1dof_rk3(benchmark):
    """Benchmarks the 1-DOF RK3 solver (Fixed Step)."""
    rocket = make_bench_rocket()
    timestep = FixedTimeStep(0.01)
    benchmark(
        rocket.predict_apogee_1dof,
        initial_height=0.0,
        initial_velocity=150.0,
        integration_method=OdeMethod.RK3,
        timestep_config=timestep,
    )


def test_bench_1dof_rk45(benchmark):
    """Benchmarks the 1-DOF adaptive RK45 solver (Adaptive Step)."""
    rocket = make_bench_rocket()
    timestep = AdaptiveTimeStep.default()
    benchmark(
        rocket.predict_apogee_1dof,
        initial_height=0.0,
        initial_velocity=150.0,
        integration_method=OdeMethod.RK45,
        timestep_config=timestep,
    )


def test_bench_3dof_rk45(benchmark):
    """Benchmarks the 3-DOF adaptive RK45 solver (Heaviest Combined Loop)."""
    rocket = make_bench_rocket()
    timestep = AdaptiveTimeStep.default()
    benchmark(
        rocket.predict_apogee_3dof,
        initial_height=0.0,
        initial_velocity=150.0,
        initial_angle=1.57079,
        integration_method=OdeMethod.RK45,
        timestep_config=timestep,
    )
