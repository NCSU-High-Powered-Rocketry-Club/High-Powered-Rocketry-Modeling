use crate::state_mod::{State, StateVector};

pub(crate) enum OdeMethod {
    //1st argument = timestep size
    Euler(f64),
    RK3(f64),
}

impl OdeMethod {
    pub(crate) fn timestep(&self, state: &mut State) {
        //Wrapper function. Used to execute an iteration, or timestep,
        // given a state/ODE, and a timestepping method
        match self {
            OdeMethod::Euler(delta_time) => Self::explicit_euler(state, *delta_time),
            OdeMethod::RK3(delta_time) => Self::runge_kutta_3(state, *delta_time),
            _ => {
                println!("Invalid ODE Integration Method");
                std::process::exit(1);
            }
        }
    }

    fn explicit_euler(state: &mut State, dt: f64) {
        //The Explicit euler method is the most basic,
        // just multiplying th derivative by the timestep
        let dudt: StateVector = state.get_derivs();
        let du = state.multiply(dudt, dt);
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
        let dudt: StateVector = state_rk.get_derivs();
        let mut du = state_rk.multiply(dudt.clone(), dt);
        state_rk.update(du, 0.0);

        //Stage 2       dt = 0.5 * DT
        let dudt2 = state_rk.get_derivs();
        let coeff: f64 = 0.25 * dt;
        du = state_rk.add(
            state_rk.multiply(dudt.clone(), coeff),
            state_rk.multiply(dudt2.clone(), coeff),
        );
        state_rk = state.clone();
        state_rk.update(du, 0.0);

        //Stage 3
        let dudt3 = state_rk.get_derivs();
        let coeff = dt * 1.0 / 6.0;
        du = state_rk.multiply(dudt, coeff);
        du = state_rk.add(du, state_rk.multiply(dudt2, coeff));
        du = state_rk.add(du, state_rk.multiply(dudt3, 4.0 * coeff));
        state.update(du, dt);
    }
}
