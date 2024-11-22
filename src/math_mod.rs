use crate::state_mod::{State, StateVector};

pub(crate) enum OdeIterators {
    //1st argument = timestep size
    Euler(f64),
    RK3(f64),
}

impl OdeIterators {
    pub(crate) fn timestep(&self, state: &mut State) {
        match self {
            OdeIterators::Euler(delta_time) => Self::explicit_euler(state, *delta_time),
            OdeIterators::RK3(delta_time) => Self::runge_kutta_3(state, *delta_time),
            _ => {
                println!("Invalid ODE Integration Method");
                std::process::exit(1);
            }
        }
    }

    fn explicit_euler(state: &mut State, dt: f64) {
        let dudt: StateVector = state.get_derivs();
        let du = state.multiply(dudt, dt);
        state.update(du, dt)
    }

    fn runge_kutta_3(state: &mut State, dt: f64) {
        //Based off Strong Stability Preserving (SSP) aka. Total variation Diminishing (TVD) RK3
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
