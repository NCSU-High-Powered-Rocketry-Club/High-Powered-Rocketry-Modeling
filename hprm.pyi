from __future__ import annotations

from enum import Enum
from typing import Optional


class ModelType(Enum):
    """
    Represents the type of dynamic model used for the simulation.
    """

    OneDOF = 0
    """
    One degree of freedom model; rocket moves only in the vertical direction (y).
    """

    ThreeDOF = 1
    """
    Three degrees of freedom model; rocket moves in 2D with rotation (x, y, theta).
    """


class OdeMethod(Enum):
    """
    Numerical integration methods for the ODE solver.
    """

    Euler = 0
    """
    First-order explicit Euler method.
    """

    RK3 = 1
    """
    Third-order Runge–Kutta method.
    """

    RK45 = 2
    """
    Fourth-order Runge–Kutta method with adaptive time stepping.
    """


class FixedTimeStep:
    """
    Configuration for fixed time stepping.

    :param dt: Time step size in seconds.
    """

    dt: float
    """
    Time step size in seconds.
    """

    def __init__(self, dt: float) -> None:
        ...


class AdaptiveTimeStep:
    """
    Configuration for adaptive time stepping.
    """

    dt: float
    """
    Current timestep in seconds.
    """
    dt_min: float
    """
    Minimum allowed timestep in seconds.
    """
    dt_max: float
    """
    Maximum allowed timestep in seconds.
    """
    absolute_error_tolerance: float
    """
    Target absolute error tolerance.
    """
    relative_error_tolerance: float
    """
    Target relative error tolerance.
    """

    def __init__(
        self,
        dt: float,
        dt_min: float,
        dt_max: float,
        absolute_error_tolerance: float,
        relative_error_tolerance: float,
    ) -> None:
        """
        Create an AdaptiveTimeStep with specified parameters.
        """
        ...

    @staticmethod
    def default() -> AdaptiveTimeStep:
        """
        Create an AdaptiveTimeStep using default parameters.
            - dt = 0.01
            - dt_min = 1e-6
            - dt_max = 10.0
            - absolute_error_tolerance = 1e-2
            - relative_error_tolerance = 1e-2
        """
        ...

    def next_dt(self, error_norm: float) -> float:
        """
        Compute the next timestep based on the current error norm.

        :param error_norm: Norm of the estimated local error.
        :return: Suggested new timestep in seconds, clamped to [dt_min, dt_max].
        """
        ...


class SimulationData:
    """
    Stores the results of a simulation as a time history.
    """

    len: int
    """
    Number of rows in the simulation log.
    """

    def __init__(self) -> None:
        ...

    def get_val(self, index: int, col: int) -> float:
        """
        Get a value from the simulation data.

        :param index: Row index (time step).
        :param col: Column index (0 for time, 1+ for state variables).
        :return: Value at the given row and column.
        """
        ...

    def get_len(self) -> int:
        """
        Get the number of data points.

        :return: Number of rows in the simulation log.
        """
        ...


class Rocket:
    """
    Physical properties of the rocket used in the simulation.

    :param mass: Mass of the rocket in kilograms.
    :param cd: Drag coefficient.
    :param area_drag: Reference area for drag in square meters.
    :param area_lift: Reference area for lift in square meters.
    :param moment_of_inertia: Moment of inertia about the z-axis in kg·m².
    :param stab_margin_dimensional: Static stability margin in meters.
    :param cl_a: Lift coefficient slope per radian.
    """

    mass: float
    """
    Mass of the rocket in kilograms.
    """
    cd: float
    """
    Drag coefficient.
    """
    area_drag: float
    """
    Reference area for drag in square meters.
    """
    area_lift: float
    """
    Reference area for lift in square meters.
    """
    moment_of_inertia: float
    """
    Moment of inertia about the z-axis in kg·m².
    """
    stab_margin_dimensional: float
    """
    Static stability margin in meters.
    """
    cl_a: float
    """
    Lift coefficient slope per radian.
    """

    def __init__(
        self,
        mass: float,
        cd: float,
        area_drag: float,
        area_lift: float,
        moment_of_inertia: float,
        stab_margin_dimensional: float,
        cl_a: float,
    ) -> None:
        ...

    def simulate_flight(
        self,
        initial_height: float,
        initial_velocity: float,
        model_type: ModelType,
        integration_method: OdeMethod,
        timestep_config: Optional[FixedTimeStep | AdaptiveTimeStep] = None,
        initial_angle: Optional[float] = None,
        print_output: bool = False,
    ) -> SimulationData:
        """
        Simulate the rocket's flight and return the full time history.

        :param initial_height: Initial altitude of the rocket in meters.
        :param initial_velocity: Initial vertical velocity in meters per second.
        :param model_type: Dynamic model to use (OneDOF or ThreeDOF).
        :param integration_method: Numerical integration method to use.
        :param timestep_config: Time step configuration (fixed or adaptive), or None for defaults.
        :param initial_angle: Initial orientation in radians for ThreeDOF, or None for default.
        :param print_output: Whether to print simulation progress to stdout.
        :return: SimulationData containing the simulated trajectory.
        """
        ...

    def predict_apogee(
        self,
        initial_height: float,
        initial_velocity: float,
        model_type: ModelType,
        integration_method: OdeMethod,
        timestep_config: Optional[FixedTimeStep | AdaptiveTimeStep] = None,
        initial_angle: Optional[float] = None,
        print_output: bool = False,
    ) -> float:
        """
        Predict the apogee (maximum altitude) of the flight.

        :param initial_height: Initial altitude of the rocket in meters.
        :param initial_velocity: Initial vertical velocity in meters per second.
        :param model_type: Dynamic model to use (OneDOF or ThreeDOF).
        :param integration_method: Numerical integration method to use.
        :param timestep_config: Time step configuration (fixed or adaptive), or None for defaults.
        :param initial_angle: Initial orientation in radians for ThreeDOF, or None for default.
        :param print_output: Whether to print simulation progress to stdout.
        :return: Maximum altitude reached during the simulated flight in meters.
        """
        ...
