use pyo3::pyclass;

use crate::math::vec_ops::VectorOperations;
use crate::state::State;

#[pyclass(dict, get_all, set_all)]
#[derive(Clone)]
pub(crate) enum OdeMethod {
    //1st argument = timestep size
    Euler(f64),
    RK3(f64),
    RK45(AdaptiveTimeStep),
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

impl OdeMethod {
    pub(crate) fn timestep(&self, state: &mut State) {
        //Wrapper function. Used to execute an iteration, or timestep,
        // given a state/ODE, and a timestepping method
        match self {
            OdeMethod::Euler(delta_time) => Self::explicit_euler(state, *delta_time),
            OdeMethod::RK3(delta_time) => Self::runge_kutta_3(state, *delta_time),
            OdeMethod::RK45(adaptive_time_step) => Self::runge_kutta_45(state, adaptive_time_step),
            _ => {
                println!("Invalid ODE Integration Method");
                std::process::exit(1);
            }
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

        //Stage 2       dt = 0.5 * DT
        let dudt2 = state_rk.get_derivs();
        let coeff: f64 = 0.25 * dt;
        du = dudt.clone().scale(coeff) + dudt2.clone().scale(coeff);

        state_rk = state.clone();
        state_rk.update(du, 0.0);

        //Stage 3
        let dudt3 = state_rk.get_derivs();
        let coeff = dt * 1.0 / 6.0;
        du = dudt.scale(coeff);
        du = du + dudt2.scale(coeff);
        du = du + dudt3.scale(4.0 * coeff);
        state.update(du, dt);
    }

    fn runge_kutta_45(state: &mut State, adaptive_time_step: &AdaptiveTimeStep) {
        // Placeholder for RK45 implementation
        // Currently not implemented
        println!("RK45 method is not yet implemented.");
        std::process::exit(1);
    }
}
