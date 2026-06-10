use crate::constants::ode_constants::{
    DEFAULT_MAX_TIMESTEP, DEFAULT_MIN_TIMESTEP, DEFAULT_TIMESTEP, DEFAULT_TOLERANCE, SAFETY_FACTOR,
};

use pyo3::exceptions::PyTypeError;
use pyo3::prelude::*;

use crate::state::State;

#[pyclass(eq, eq_int)]
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
/// Numerical integration methods for the ODE solver.
pub enum OdeMethod {
    /// First-order explicit Euler method.
    Euler,
    /// Third-order Runge-Kutta method.
    RK3,
    /// Fourth-order Runge-Kutta method with adaptive time stepping.
    RK45,
}

#[derive(FromPyObject, Clone, Debug)]
pub enum TimeStepOptions {
    Fixed(FixedTimeStep),
    Adaptive(AdaptiveTimeStep),
}

#[pyclass(get_all, set_all)]
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct FixedTimeStep {
    pub dt: f64,
}

#[pymethods]
impl FixedTimeStep {
    #[new]
    pub fn new(dt: f64) -> Self {
        Self { dt }
    }
}

#[pyclass(get_all, set_all)]
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct AdaptiveTimeStep {
    /// Initial timestep guess
    pub dt: f64,
    /// Minimum timestep
    pub dt_min: f64,
    /// Maximum timestep
    pub dt_max: f64,
    /// Absolute error tolerance
    pub absolute_error_tolerance: f64,
    /// Relative error tolerance
    pub relative_error_tolerance: f64,
}

#[pymethods]
impl AdaptiveTimeStep {
    #[new]
    pub fn new(
        dt: f64,
        dt_min: f64,
        dt_max: f64,
        absolute_error_tolerance: f64,
        relative_error_tolerance: f64,
    ) -> Self {
        Self {
            dt,
            dt_min,
            dt_max,
            absolute_error_tolerance,
            relative_error_tolerance,
        }
    }

    #[staticmethod]
    pub fn default() -> Self {
        Self {
            dt: DEFAULT_TIMESTEP,
            dt_min: DEFAULT_MIN_TIMESTEP,
            dt_max: DEFAULT_MAX_TIMESTEP,
            absolute_error_tolerance: DEFAULT_TOLERANCE,
            relative_error_tolerance: DEFAULT_TOLERANCE,
        }
    }

    /// Computes the next timestep size based on the current error norm using a common
    /// adaptive timestep control formula.
    pub fn next_dt(&self, error_norm: f64) -> f64 {
        let dt = self.dt;

        // Account for edge case where error norm is extremely small or 0, if so double it
        if error_norm <= 1e-30 {
            return (dt * 2.0).clamp(self.dt_min, self.dt_max);
        }

        (dt * (((self.absolute_error_tolerance + self.relative_error_tolerance * dt)
            * SAFETY_FACTOR
            / error_norm)
            .powf(0.25)) // it's a 4th order method, so we have to do 1/4 power here
        .clamp(0.5, 2.0)) // limits the change in timestep to be between 0.5x and 2x
        .clamp(self.dt_min, self.dt_max)
    }
}

#[derive(Clone, Debug, PartialEq)]
pub(crate) enum OdeSolver {
    Euler(FixedTimeStep),
    RK3(FixedTimeStep),
    RK45(AdaptiveTimeStep),
}

impl OdeSolver {
    /// Factory method to create an OdeSolver from a given method and optional timestep configuration.
    /// If the timestep configuration is not provided, it will use default values based on the method,
    /// for RK45 it will use the default adaptive timestep configuration, and for Euler and RK3 it will
    /// use a default fixed timestep of 0.01 seconds.
    pub(crate) fn from_method(
        method: OdeMethod,
        timestep_config: Option<TimeStepOptions>,
    ) -> PyResult<Self> {
        match (method, timestep_config) {
            (OdeMethod::Euler, Some(TimeStepOptions::Fixed(f))) => Ok(OdeSolver::Euler(f)),
            (OdeMethod::Euler, None) => Ok(OdeSolver::Euler(FixedTimeStep::new(0.01))),
            (OdeMethod::Euler, Some(TimeStepOptions::Adaptive(_))) => {
                Err(PyTypeError::new_err("Euler requires FixedTimeStep"))
            }

            (OdeMethod::RK3, Some(TimeStepOptions::Fixed(f))) => Ok(OdeSolver::RK3(f)),
            (OdeMethod::RK3, None) => Ok(OdeSolver::RK3(FixedTimeStep::new(0.01))),
            (OdeMethod::RK3, Some(TimeStepOptions::Adaptive(_))) => {
                Err(PyTypeError::new_err("RK3 requires FixedTimeStep"))
            }

            (OdeMethod::RK45, Some(TimeStepOptions::Adaptive(a))) => Ok(OdeSolver::RK45(a)),
            (OdeMethod::RK45, None) => Ok(OdeSolver::RK45(AdaptiveTimeStep::default())),
            (OdeMethod::RK45, Some(TimeStepOptions::Fixed(_))) => {
                Err(PyTypeError::new_err("RK45 requires AdaptiveTimeStep"))
            }
        }
    }

    /// Once we have taken a timestep and find that we have passed apogee, we can use this method to backtrack to the
    /// actual apogee by using the previous state and the current state to estimate the time fraction at which we
    /// reached apogee, then we can adjust the timestep accordingly and rerun the timestep to get a more accurate
    /// estimate of the apogee state.
    pub(crate) fn backtrack_apogee(&mut self, state: &mut State, prev_state: &State) {
        let vertical_rate_of_distance_change_with_time_in_meters_per_second =
            state.get_vertical_velocity();
        let previous_vertical_rate_of_distance_change_with_time_in_meters_per_second =
            prev_state.get_vertical_velocity();
        // Time fraction which is approx apogee assuming const acceleration (v(t) = v0 + at)
        let tau: f64 = previous_vertical_rate_of_distance_change_with_time_in_meters_per_second
            / (previous_vertical_rate_of_distance_change_with_time_in_meters_per_second
                - vertical_rate_of_distance_change_with_time_in_meters_per_second);

        // Update the tinestep to be the desired size
        match self {
            OdeSolver::Euler(fixed) => fixed.dt *= tau,
            OdeSolver::RK3(fixed) => fixed.dt *= tau,
            OdeSolver::RK45(ats) => ats.dt *= tau,
        };

        //Rerun the timestep
        let mut tmp_state = *prev_state;
        self.timestep(&mut tmp_state);
        *state = tmp_state;
    }

    pub(crate) fn timestep(&mut self, state: &mut State) {
        match self {
            OdeSolver::Euler(fixed) => Self::explicit_euler(state, fixed.dt),
            OdeSolver::RK3(fixed) => Self::runge_kutta_3(state, fixed.dt),
            OdeSolver::RK45(a) => Self::runge_kutta_45(state, a),
        }
    }

    /// The Explicit euler method is the most basic, just multiplying the derivative by the timestep
    fn explicit_euler(state: &mut State, dt: f64) {
        let dudt = state.get_derivatives();
        let du = dudt.scale(dt);
        state.update(du, dt)
    }

    /// Runge-Kutta 3rd order method, a 3-stage method that provides better accuracy
    /// than Euler's method by taking multiple intermediate steps within each timestep.
    fn runge_kutta_3(state: &mut State, dt: f64) {
        // Runge-Kutta methods are a family of higher-order integration schemes.
        // The account for varying degrees of non-linearity /
        // curvature in the function you are trying to calculate.
        // This method is a 3-stage method based off Strong Stability Preserving (SSP) aka.
        // Total variation Diminishing (TVD) form of RK3. (commonly used in PDE applications)

        let mut state_rk: State = *state;

        //Stage 1       dt = 1 * DT
        let dudt = state_rk.get_derivatives();
        let mut du = dudt.clone().scale(dt);
        state_rk.update(du, 0.0);

        // Stage 2       dt = 0.5 * DT
        let dudt2 = state_rk.get_derivatives();
        let coeff: f64 = 0.25 * dt;
        du = dudt.clone().scale(coeff) + dudt2.clone().scale(coeff);

        state_rk = *state;
        state_rk.update(du, 0.0);

        // Stage 3
        let dudt3 = state_rk.get_derivatives();
        let coeff = dt * 1.0 / 6.0;
        du = dudt.scale(coeff);
        du += dudt2.scale(coeff);
        du += dudt3.scale(4.0 * coeff);
        state.update(du, dt);
    }

    /// Runge-Kutta-Fehlberg method, a 4th-order method with an embedded 5th-order method for error
    /// estimation and adaptive timestep control. Basically what this means is that we get the best of
    /// both worlds: we get a 4th-order accurate solution, but it can also be a lot faster than a fixed
    /// timestep method because it can take larger steps when the solution is smooth and smaller steps
    /// when the solution is changing rapidly.
    ///
    /// Don't worry about all of the scary numbers, they are just the coefficients of the method which
    /// were derived by Fehlberg in the 1960s.
    fn runge_kutta_45(state: &mut State, adaptive_time_step: &mut AdaptiveTimeStep) {
        let dt = adaptive_time_step.dt;

        // TODO: when we replace vecops, we don't have to have all of these update calls

        // ========== Stage 1 ==========
        let dudt1 = state.get_derivatives();
        let k1 = dudt1.clone().scale(dt);

        // ========== Stage 2 ==========
        let mut stage = *state;
        // ut = u + 0.2 * k1
        stage.update(k1.clone().scale(0.2), 0.0);
        let dudt2 = stage.get_derivatives();
        let k2 = dudt2.clone().scale(dt);

        // ========== Stage 3 ==========
        let mut stage = *state;
        // ut = u + 0.075*k1 + 0.225*k2
        stage.update(k1.clone().scale(0.075), 0.0);
        stage.update(k2.clone().scale(0.225), 0.0);
        let dudt3 = stage.get_derivatives();
        let k3 = dudt3.clone().scale(dt);

        // ========== Stage 4 ==========
        let mut stage = *state;
        // ut = u + (44/45)*k1 - (56/15)*k2 + (32/9)*k3
        stage.update(k1.clone().scale(44.0 / 45.0), 0.0);
        stage.update(k2.clone().scale(-56.0 / 15.0), 0.0);
        stage.update(k3.clone().scale(32.0 / 9.0), 0.0);
        let dudt4 = stage.get_derivatives();
        let k4 = dudt4.clone().scale(dt);

        // ========== Stage 5 ==========
        let mut stage = *state;
        // ut = u + (19372/6561)*k1 - (25360/2187)*k2
        //          + (64448/6561)*k3 - (212/729)*k4
        stage.update(k1.clone().scale(19372.0 / 6561.0), 0.0);
        stage.update(k2.clone().scale(-25360.0 / 2187.0), 0.0);
        stage.update(k3.clone().scale(64448.0 / 6561.0), 0.0);
        stage.update(k4.clone().scale(-212.0 / 729.0), 0.0);
        let dudt5 = stage.get_derivatives();
        let k5 = dudt5.clone().scale(dt);

        // ========== Stage 6 ==========
        let mut stage = *state;
        // ut = u + (9017/3168)*k1 - (355/33)*k2
        //          + (46732/5247)*k3 + (49/176)*k4
        //          - (5103/18656)*k5
        stage.update(k1.clone().scale(9017.0 / 3168.0), 0.0);
        stage.update(k2.clone().scale(-355.0 / 33.0), 0.0);
        stage.update(k3.clone().scale(46732.0 / 5247.0), 0.0);
        stage.update(k4.clone().scale(49.0 / 176.0), 0.0);
        stage.update(k5.clone().scale(-5103.0 / 18656.0), 0.0);
        let dudt6 = stage.get_derivatives();
        let k6 = dudt6.clone().scale(dt);

        // ========== Stage 7 (5th-order combination) ==========
        let mut stage = *state;
        // ut = u + (35/384)*k1 + (500/1113)*k3
        //          + (125/192)*k4 - (2187/6784)*k5
        //          + (11/84)*k6
        stage.update(k1.clone().scale(35.0 / 384.0), 0.0);
        stage.update(k3.clone().scale(500.0 / 1113.0), 0.0);
        stage.update(k4.clone().scale(125.0 / 192.0), 0.0);
        stage.update(k5.clone().scale(-2187.0 / 6784.0), 0.0);
        stage.update(k6.clone().scale(11.0 / 84.0), 0.0);
        let dudt7 = stage.get_derivatives();
        let k7 = dudt7.clone().scale(dt);

        // ---------- Build 5th-order increment (du5) ----------
        let mut du5 = k1.clone().scale(35.0 / 384.0);
        du5 += k3.clone().scale(500.0 / 1113.0);
        du5 += k4.clone().scale(125.0 / 192.0);
        du5 += k5.clone().scale(-2187.0 / 6784.0);
        du5 += k6.clone().scale(11.0 / 84.0);
        // (no k7 in the 5th-order solution)

        // ---------- Build 4th-order increment (du4) ----------
        let mut du4 = k1.clone().scale(5179.0 / 57600.0);
        du4 += k3.clone().scale(7571.0 / 16695.0);
        du4 += k4.clone().scale(393.0 / 640.0);
        du4 += k5.clone().scale(-92097.0 / 339200.0);
        du4 += k6.clone().scale(187.0 / 2100.0);
        du4 += k7.clone().scale(1.0 / 40.0);

        // ---------- Error estimate: || du4 - du5 || ----------
        let error_vec = du4 - du5;

        // Find the size of the error vector
        let error_norm: f64 = error_vec.dot(&error_vec).sqrt();

        // ---------- Update timestep adaptively ----------
        let new_dt = adaptive_time_step.next_dt(error_norm);
        adaptive_time_step.dt = new_dt;
        //println!("RK45 Error Norm: {:},     dt: {:}", error_norm, new_dt);

        // ---------- Finally, advance the actual state with 5th-order increment ----------
        state.update(du5, dt);
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::rocket::RocketProperties;
    use crate::state::State;
    use approx::{assert_abs_diff_eq, assert_relative_eq};

    fn make_rocket_properties() -> RocketProperties {
        RocketProperties::new(15.0, 0.5, 0.02, 0.01, 0.25, 0.10, 4.5)
    }

    #[test]
    fn test_euler_1dof() {
        let rocket_properties = make_rocket_properties();
        // Start at 100m altitude, climbing straight up at 50 m/s
        let mut state = State::new_1dof(rocket_properties, 100.0, 50.0);

        // Setup Euler solver with a fixed timestep of 0.1 seconds
        let mut solver = OdeSolver::Euler(FixedTimeStep::new(0.1));

        // Execute a single step
        solver.timestep(&mut state);

        // For Euler: new_height = old_height + old_velocity * dt
        // 100.0 + (50.0 * 0.1) = 105.0m
        assert_abs_diff_eq!(state.get_altitude(), 105.0, epsilon = 1e-12);

        // Velocity must decrease due to gravity and atmospheric drag acting downwards, but like not too much
        assert!(state.get_vertical_velocity() < 50.0);
        assert!(state.get_vertical_velocity() > 45.0);
        assert_abs_diff_eq!(state.get_time(), 0.1, epsilon = 1e-12);
    }

    #[test]
    fn test_euler_3dof() {
        let rocket_properties = make_rocket_properties();

        // 85 degrees in radians
        let angle_deg = 85.0_f64;
        let angle_rad = angle_deg.to_radians();
        let v_initial = 10.0;
        let h_initial = 10.0;

        // Calculate initial x and y velocity components
        let v_y0 = v_initial * angle_rad.sin(); // ~9.96 m/s

        // 3DOF initialization: height = 0.0, velocity = 10.0, angle = 85 degrees
        let mut state = State::new_3dof(rocket_properties, h_initial, v_initial, angle_rad);

        let dt = 0.2;
        let mut solver = OdeSolver::Euler(FixedTimeStep::new(dt));

        solver.timestep(&mut state);

        // Explicit Euler Kinematics for altitude (y-axis):
        // new_altitude = old_altitude + old_y_velocity * dt
        let expected_altitude = h_initial + (v_y0 * dt);

        assert_abs_diff_eq!(state.get_time(), dt, epsilon = 1e-12);
        assert_abs_diff_eq!(state.get_altitude(), expected_altitude, epsilon = 1e-12);

        // The vertical velocity must decrease due to gravity and drag, but not too much
        assert!(state.get_vertical_velocity() < v_y0);
        assert!(state.get_vertical_velocity() > 7.0);
    }

    /// This test was written assuming the math is correct, using values gotten by actually running the code
    #[test]
    fn test_rk3_1dof() {
        let rocket_properties = make_rocket_properties();
        let mut state = State::new_1dof(rocket_properties, 0.0, 100.0);

        let dt = 0.05;
        let mut solver = OdeSolver::RK3(FixedTimeStep::new(dt));

        solver.timestep(&mut state);

        assert_abs_diff_eq!(state.get_time(), dt, epsilon = 1e-12);

        let expected_altitude = 4.982661071033707;

        assert_abs_diff_eq!(state.get_altitude(), expected_altitude, epsilon = 1e-6);

        // Velocity decreases due to gravity (~9.81 * 0.05 = ~0.49 m/s) and drag.
        assert!(state.get_vertical_velocity() < 100.0);
        assert!(state.get_vertical_velocity() > 95.0);
    }

    /// This test was written assuming the math is correct, using values gotten by actually running the code
    #[test]
    fn test_rk45_1dof() {
        let rocket_properties = make_rocket_properties();
        let mut state = State::new_1dof(rocket_properties, 500.0, 300.0);

        // Configure adaptive steps with an initial step size of 0.1
        let initial_dt = 0.1;
        let adaptive_config = AdaptiveTimeStep::new(initial_dt, 0.01, 1.0, 1e-4, 1e-4);
        let mut solver = OdeSolver::RK45(adaptive_config);

        // Executes exactly 1 step using the initial_dt (0.1s) before adapting
        solver.timestep(&mut state);

        // Time must advance precisely by the initial fixed step size
        assert_abs_diff_eq!(state.get_time(), initial_dt, epsilon = 1e-12);

        let expected_altitude = 529.7690989929642;
        let expected_velocity = 295.40061606079655;
        let expected_adapted_dt = 0.2;

        // Assert exact matches for the state updates
        assert_abs_diff_eq!(state.get_altitude(), expected_altitude, epsilon = 1e-6);
        assert_abs_diff_eq!(
            state.get_vertical_velocity(),
            expected_velocity,
            epsilon = 1e-6
        );

        // Gets internal state of RK45 to verify what it changed the next dt to
        if let OdeSolver::RK45(ref updated_config) = solver {
            assert_abs_diff_eq!(updated_config.dt, expected_adapted_dt, epsilon = 1e-6);
        }
    }

    #[test]
    fn test_backtrack_apogee_interpolation() {
        let rocket_properties = make_rocket_properties();

        // 1. Establish a valid previous state just before apogee.
        // At 0.45 m/s, it will reach apogee in roughly (0.45 / 9.81) ≈ 0.046 seconds.
        let prev_state = State::new_1dof(rocket_properties, 1500.0, 0.45);

        // 2. Compute a physically realistic next state using the solver itself.
        // A step of 0.1 seconds will naturally push the velocity past zero to approx -0.53 m/s.
        let mut current_state = prev_state.clone();
        let mut solver = OdeSolver::RK3(FixedTimeStep::new(0.1));

        solver.timestep(&mut current_state);

        // 3. Execute backtrack recalculation
        solver.backtrack_apogee(&mut current_state, &prev_state);

        // Because the states are now mathematically consistent with the underlying gravity/drag,
        // the backtracked state should land cleanly at the apex point (velocity near 0.0).
        assert_relative_eq!(current_state.get_vertical_velocity(), 0.0, epsilon = 1e-2);
        assert!(current_state.get_altitude() >= prev_state.get_altitude());
    }

    #[test]
    fn test_fixed_timestep_initialization() {
        let expected_dt = 0.05;
        let config = FixedTimeStep::new(expected_dt);

        assert_abs_diff_eq!(config.dt, expected_dt, epsilon = 1e-12);
    }

    #[test]
    fn test_adaptive_timestep_initialization() {
        let config = AdaptiveTimeStep::new(0.1, 0.01, 1.0, 1e-5, 1e-6);
        assert_abs_diff_eq!(config.dt, 0.1, epsilon = 1e-12);
        assert_abs_diff_eq!(config.dt_min, 0.01, epsilon = 1e-12);
        assert_abs_diff_eq!(config.dt_max, 1.0, epsilon = 1e-12);
        assert_abs_diff_eq!(config.absolute_error_tolerance, 1e-5, epsilon = 1e-12);
        assert_abs_diff_eq!(config.relative_error_tolerance, 1e-6, epsilon = 1e-12);
    }

    #[test]
    fn test_adaptive_timestep_default() {
        let config = AdaptiveTimeStep::default();
        assert_abs_diff_eq!(config.dt, super::DEFAULT_TIMESTEP, epsilon = 1e-12);
        assert_abs_diff_eq!(config.dt_min, super::DEFAULT_MIN_TIMESTEP, epsilon = 1e-12);
        assert_abs_diff_eq!(config.dt_max, super::DEFAULT_MAX_TIMESTEP, epsilon = 1e-12);
        assert_abs_diff_eq!(
            config.absolute_error_tolerance,
            super::DEFAULT_TOLERANCE,
            epsilon = 1e-12
        );
        assert_abs_diff_eq!(
            config.relative_error_tolerance,
            super::DEFAULT_TOLERANCE,
            epsilon = 1e-12
        );
    }

    #[test]
    fn test_adaptive_timestep_next_dt() {
        let config = AdaptiveTimeStep::new(0.1, 0.01, 0.5, 1e-4, 1e-4);
        let error_norm = 5e-4;

        let dt = config.dt;
        let expected_dt = (dt
            * (((config.absolute_error_tolerance + config.relative_error_tolerance * dt)
                * super::SAFETY_FACTOR
                / error_norm)
                .powf(0.25))
            .clamp(0.5, 2.0))
        .clamp(config.dt_min, config.dt_max);

        let actual_dt = config.next_dt(error_norm);
        assert_abs_diff_eq!(actual_dt, expected_dt, epsilon = 1e-12);
    }

    #[test]
    fn test_adaptive_timestep_next_dt_zero() {
        // When error_norm <= 1e-30, next_dt should double the current dt (0.1 -> 0.2)
        let config = AdaptiveTimeStep::new(0.1, 0.01, 0.5, 1e-4, 1e-4);
        let next = config.next_dt(0.0);
        assert_abs_diff_eq!(next, 0.2, epsilon = 1e-12);

        // Verify that the doubling behavior safely respects the dt_max ceiling
        // (0.4 * 2.0 = 0.8, which should clamp hard down to 0.5)
        let config_max_clamp = AdaptiveTimeStep::new(0.4, 0.01, 0.5, 1e-4, 1e-4);
        let next_clamped = config_max_clamp.next_dt(1e-35);
        assert_abs_diff_eq!(next_clamped, 0.5, epsilon = 1e-12);
    }
}
