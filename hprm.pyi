from __future__ import annotations

from enum import Enum
from typing import Optional
import numpy as np

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

    def __init__(self, dt: float) -> None: ...

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

class InitialState1DOF:
    """
    Defines the initial conditions of a 1-DOF simulation.
    """

    initial_height: float
    """
    Initial altitude of the rocket in meters (m).
    """

    initial_velocity: float
    """
    Initial vertical velocity of the rocket in meters per second (m/s).
    """

    def __init__(self, initial_height: float, initial_velocity: float) -> None:
        """
        Create a new 1-DOF initial state.

        :param initial_height: Initial altitude in meters (m).
        :param initial_velocity: Initial vertical velocity in meters per second (m/s).
        """
        ...

class InitialState3DOF:
    """
    Defines the initial conditions of a 3-DOF simulation.
    """

    x: float
    """
    Initial horizontal position in meters (m).
    """

    y: float
    """
    Initial vertical position (altitude) in meters (m).
    """

    angle: float
    """
    Initial orientation angle in radians (rad). Pi / 2 means pointing straight up.
    """

    vx: float
    """
    Initial horizontal velocity in meters per second (m/s).
    """

    vy: float
    """
    Initial vertical velocity in meters per second (m/s).
    """

    angular_rate: float
    """
    Initial angular velocity in radians per second (rad/s).
    """

    def __init__(
        self,
        x: float,
        y: float,
        angle: float,
        vx: float,
        vy: float,
        angular_rate: float,
    ) -> None:
        """
        Create a new 3-DOF initial state.

        :param x: Initial horizontal position in meters (m).
        :param y: Initial vertical position (altitude) in meters (m).
        :param angle: Initial orientation angle in radians (rad).
        :param vx: Initial horizontal velocity in meters per second (m/s).
        :param vy: Initial vertical velocity in meters per second (m/s).
        :param angular_rate: Initial angular velocity in radians per second (rad/s).
        """
        ...

class RocketProperties:
    """
    Internal physical property group for the rocket.
    Accessible from Python for reading or updating fields dynamically.
    """

    mass: float
    """
    Mass of the rocket in kilograms (kg).
    """

    cd: float
    """
    Drag coefficient (dimensionless).
    """

    area_drag: float
    """
    Reference area for drag in square meters (m²).
    """

    area_lift: float
    """
    Reference area for lift in square meters (m²).
    """

    moment_of_inertia: float
    """
    Moment of inertia about the z-axis in kilogram square meters (kg·m²).
    """

    stab_margin_dimensional: float
    """
    Static stability margin in meters (m).
    """

    cl_a: float
    """
    Lift coefficient slope per radian (1/rad).
    """

class Rocket:
    """
    The main class for simulating rocket flight. Contains methods for 1-DOF and 3-DOF simulations,
    as well as apogee predictions.
    """

    rocket_properties: RocketProperties
    """
    The physical properties of the rocket.
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
        """
        Creates a new Rocket instance and initializes its underlying RocketProperties group.

        :param mass: Mass of the rocket in kilograms.
        :param cd: Drag coefficient.
        :param area_drag: Reference area for drag in square meters.
        :param area_lift: Reference area for lift in square meters.
        :param moment_of_inertia: Moment of inertia about the z-axis in kg·m².
        :param stab_margin_dimensional: Static stability margin in meters.
        :param cl_a: Lift coefficient slope per radian.
        """
        ...

    def simulate_flight_1dof(
        self,
        initial_state: InitialState1DOF,
        integration_method: OdeMethod,
        timestep_config: Optional[FixedTimeStep | AdaptiveTimeStep] = None,
        max_iterations: int = 100000,
        print_output: bool = False,
    ) -> tuple[np.ndarray, np.ndarray]:
        """
        Simulate the rocket's flight using a 1-DOF model (vertical motion only).

        :param initial_state: The initial height and velocity of the rocket.
        :param integration_method: Numerical integration method to use.
        :param timestep_config: Time step configuration (fixed or adaptive), or None for defaults.
        :param max_iterations: Maximum integration iterations allowed.
        :param print_output: Whether to print simulation progress to stdout.
        :return: A tuple containing (time_array, state_matrix) as NumPy arrays.
        """
        ...

    def simulate_flight_3dof(
        self,
        initial_state: InitialState3DOF,
        integration_method: OdeMethod,
        timestep_config: Optional[FixedTimeStep | AdaptiveTimeStep] = None,
        max_iterations: int = 100000,
        print_output: bool = False,
    ) -> tuple[np.ndarray, np.ndarray]:
        """
        Simulate the rocket's flight using a 3-DOF model (2D translation and rotation).

        :param initial_state: The initial 6-DOF condition of the rocket.
        :param integration_method: Numerical integration method to use.
        :param timestep_config: Time step configuration (fixed or adaptive), or None for defaults.
        :param max_iterations: Maximum integration iterations allowed.
        :param print_output: Whether to print simulation progress to stdout.
        :return: A tuple containing (time_array, state_matrix) as NumPy arrays.
        """
        ...

    def predict_apogee_1dof(
        self,
        initial_state: InitialState1DOF,
        integration_method: OdeMethod,
        timestep_config: Optional[FixedTimeStep | AdaptiveTimeStep] = None,
        max_iterations: int = 100000,
        print_output: bool = False,
    ) -> float:
        """
        Predict the apogee (maximum altitude) using a 1-DOF model.

        :param initial_state: The initial height and velocity of the rocket.
        :param integration_method: Numerical integration method to use.
        :param timestep_config: Time step configuration (fixed or adaptive), or None for defaults.
        :param max_iterations: Maximum integration iterations allowed.
        :param print_output: Whether to print simulation progress to stdout.
        :return: Maximum altitude reached in meters.
        """
        ...

    def predict_apogee_3dof(
        self,
        initial_state: InitialState3DOF,
        integration_method: OdeMethod,
        timestep_config: Optional[FixedTimeStep | AdaptiveTimeStep] = None,
        max_iterations: int = 100000,
        print_output: bool = False,
    ) -> float:
        """
        Predict the apogee (maximum altitude) using a 3-DOF model.

        :param initial_state: The initial 6-DOF condition of the rocket.
        :param integration_method: Numerical integration method to use.
        :param timestep_config: Time step configuration (fixed or adaptive), or None for defaults.
        :param max_iterations: Maximum integration iterations allowed.
        :param print_output: Whether to print simulation progress to stdout.
        :return: Maximum altitude reached in meters.
        """
        ...
