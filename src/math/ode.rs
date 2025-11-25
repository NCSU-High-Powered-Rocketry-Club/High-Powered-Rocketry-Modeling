use pyo3::exceptions::PyTypeError;
use pyo3::prelude::*;

use crate::OdeMethod;
use crate::math::vec_ops::VectorOperations;
use crate::state::State;

#[derive(FromPyObject)]
pub enum TimeStepOptions {
    Fixed(FixedTimeStep),
    Adaptive(AdaptiveTimeStep),
}

#[derive(Clone)]
pub(crate) enum OdeSolver {
    Euler(FixedTimeStep),
    RK3(FixedTimeStep),
    RK45(AdaptiveTimeStep),
}

#[pyclass(dict, get_all, set_all)]
#[derive(Clone)]
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

#[pyclass(dict, get_all, set_all)]
#[derive(Clone)]
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
    pub(crate) fn new() -> Self {
        Self{
            dt: 0.01,
            dt_min: 1e-6,
            dt_max: 10.0,
            absolute_error_tolerance: 1.0e-2,
            relative_error_tolerance: 1.0e-2,
        }
    }
    pub fn next_dt(&self, error_norm: f64) -> f64 {
        let dt = self.dt;
 
        // Account for edge case where error norm is extremely small or 0
        if error_norm <= 1e-30 {
            return (dt * 2.0).clamp(self.dt_min, self.dt_max);
        }
 
        (dt * (((self.absolute_error_tolerance + self.relative_error_tolerance * dt) * 0.5 / error_norm).powf(0.25))
            .clamp(0.5, 2.0))
            .clamp(self.dt_min, self.dt_max)
    }
}

impl OdeSolver {
    pub fn from_method(
        method: OdeMethod,
        timestep_config: Option<TimeStepOptions>,
    ) -> PyResult<Self> {
        match (method, timestep_config) {
            (OdeMethod::Euler, Some(TimeStepOptions::Fixed(f))) =>
                Ok(OdeSolver::Euler(f)),
            (OdeMethod::Euler, None) =>
                Ok(OdeSolver::Euler(FixedTimeStep::new(0.01))),
            (OdeMethod::Euler, Some(TimeStepOptions::Adaptive(_))) =>
                Err(PyTypeError::new_err("Euler requires FixedTimeStep")),

            (OdeMethod::RK3, Some(TimeStepOptions::Fixed(f))) =>
                Ok(OdeSolver::RK3(f)),
            (OdeMethod::RK3, None) =>
                Ok(OdeSolver::RK3(FixedTimeStep::new(0.01))),
            (OdeMethod::RK3, Some(TimeStepOptions::Adaptive(_))) =>
                Err(PyTypeError::new_err("RK3 requires FixedTimeStep")),

            (OdeMethod::RK45, Some(TimeStepOptions::Adaptive(a))) =>
                Ok(OdeSolver::RK45(a)),
            (OdeMethod::RK45, None) =>
                Ok(OdeSolver::RK45(AdaptiveTimeStep::new())),
            (OdeMethod::RK45, Some(TimeStepOptions::Fixed(_))) =>
                Err(PyTypeError::new_err("RK45 requires AdaptiveTimeStep")),
        }
    }

    pub(crate) fn timestep(&mut self, state: &mut State) {
        match self {
            OdeSolver::Euler(fixed) => Self::explicit_euler(state, fixed.dt),
            OdeSolver::RK3(fixed)   => Self::runge_kutta_3(state, fixed.dt),
            OdeSolver::RK45(a)      => Self::runge_kutta_45(state, a),
        }
    }

    fn explicit_euler(state: &mut State, dt: f64) {
        //The Explicit euler method is the most basic,
        // just multiplying th derivative by the timestep
        let dudt = state.get_derivs();
        let du = dudt.scale(dt);
        state.update(du, dt)
    }

    fn runge_kutta_3(state: &mut State, dt: f64) {
        // Runge-Kutta methods are a family of higher-order integration schemes.
        // The account for varying degrees of non-linearity /
        // curvature in the function you are trying to calculate.
        // This method is a 3-stage method based off Strong Stability Preserving (SSP) aka.
        // Total variation Diminishing (TVD) form of RK3. (commonly used in PDE applications)

        let mut state_rk: State = state.clone();

        //Stage 1       dt = 1 * DT
        let dudt = state_rk.get_derivs();
        let mut du = dudt.clone().scale(dt);
        state_rk.update(du, 0.0);

        // Stage 2       dt = 0.5 * DT
        let dudt2 = state_rk.get_derivs();
        let coeff: f64 = 0.25 * dt;
        du = dudt.clone().scale(coeff) + dudt2.clone().scale(coeff);

        state_rk = state.clone();
        state_rk.update(du, 0.0);

        // Stage 3
        let dudt3 = state_rk.get_derivs();
        let coeff = dt * 1.0 / 6.0;
        du = dudt.scale(coeff);
        du = du + dudt2.scale(coeff);
        du = du + dudt3.scale(4.0 * coeff);
        state.update(du, dt);
    }

    fn runge_kutta_45(state: &mut State, adaptive_time_step: &mut AdaptiveTimeStep) {
        let dt = adaptive_time_step.dt;

        // TODO: when we replace vecops, we don't have to have all of these update calls

        // ========== Stage 1 ==========
        let dudt1 = state.get_derivs();
        let k1 = dudt1.clone().scale(dt);

        // ========== Stage 2 ==========
        let mut stage = state.clone();
        // ut = u + 0.2 * k1
        stage.update(k1.clone().scale(0.2), 0.0);
        let dudt2 = stage.get_derivs();
        let k2 = dudt2.clone().scale(dt);

        // ========== Stage 3 ==========
        let mut stage = state.clone();
        // ut = u + 0.075*k1 + 0.225*k2
        stage.update(k1.clone().scale(0.075), 0.0);
        stage.update(k2.clone().scale(0.225), 0.0);
        let dudt3 = stage.get_derivs();
        let k3 = dudt3.clone().scale(dt);

        // ========== Stage 4 ==========
        let mut stage = state.clone();
        // ut = u + (44/45)*k1 - (56/15)*k2 + (32/9)*k3
        stage.update(k1.clone().scale(44.0 / 45.0), 0.0);
        stage.update(k2.clone().scale(-56.0 / 15.0), 0.0);
        stage.update(k3.clone().scale(32.0 / 9.0), 0.0);
        let dudt4 = stage.get_derivs();
        let k4 = dudt4.clone().scale(dt);

        // ========== Stage 5 ==========
        let mut stage = state.clone();
        // ut = u + (19372/6561)*k1 - (25360/2187)*k2
        //          + (64448/6561)*k3 - (212/729)*k4
        stage.update(k1.clone().scale(19372.0 / 6561.0), 0.0);
        stage.update(k2.clone().scale(-25360.0 / 2187.0), 0.0);
        stage.update(k3.clone().scale(64448.0 / 6561.0), 0.0);
        stage.update(k4.clone().scale(-212.0 / 729.0), 0.0);
        let dudt5 = stage.get_derivs();
        let k5 = dudt5.clone().scale(dt);

        // ========== Stage 6 ==========
        let mut stage = state.clone();
        // ut = u + (9017/3168)*k1 - (355/33)*k2
        //          + (46732/5247)*k3 + (49/176)*k4
        //          - (5103/18656)*k5
        stage.update(k1.clone().scale(9017.0 / 3168.0), 0.0);
        stage.update(k2.clone().scale(-355.0 / 33.0), 0.0);
        stage.update(k3.clone().scale(46732.0 / 5247.0), 0.0);
        stage.update(k4.clone().scale(49.0 / 176.0), 0.0);
        stage.update(k5.clone().scale(-5103.0 / 18656.0), 0.0);
        let dudt6 = stage.get_derivs();
        let k6 = dudt6.clone().scale(dt);

        // ========== Stage 7 (5th-order combination) ==========
        let mut stage = state.clone();
        // ut = u + (35/384)*k1 + (500/1113)*k3
        //          + (125/192)*k4 - (2187/6784)*k5
        //          + (11/84)*k6
        stage.update(k1.clone().scale(35.0 / 384.0), 0.0);
        stage.update(k3.clone().scale(500.0 / 1113.0), 0.0);
        stage.update(k4.clone().scale(125.0 / 192.0), 0.0);
        stage.update(k5.clone().scale(-2187.0 / 6784.0), 0.0);
        stage.update(k6.clone().scale(11.0 / 84.0), 0.0);
        let dudt7 = stage.get_derivs();
        let k7 = dudt7.clone().scale(dt);

        // ---------- Build 5th-order increment (du5) ----------
        let mut du5 = k1.clone().scale(35.0 / 384.0);
        du5 = du5 + k3.clone().scale(500.0 / 1113.0);
        du5 = du5 + k4.clone().scale(125.0 / 192.0);
        du5 = du5 + k5.clone().scale(-2187.0 / 6784.0);
        du5 = du5 + k6.clone().scale(11.0 / 84.0);
        // (no k7 in the 5th-order solution)

        // ---------- Build 4th-order increment (du4) ----------
        let mut du4 = k1.clone().scale(5179.0 / 57600.0);
        du4 = du4 + k3.clone().scale(7571.0 / 16695.0);
        du4 = du4 + k4.clone().scale(393.0 / 640.0);
        du4 = du4 + k5.clone().scale(-92097.0 / 339200.0);
        du4 = du4 + k6.clone().scale(187.0 / 2100.0);
        du4 = du4 + k7.clone().scale(1.0 / 40.0);

        // ---------- Error estimate: || du4 - du5 || ----------
        let error_vec = (du4.clone() - du5.clone());

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
