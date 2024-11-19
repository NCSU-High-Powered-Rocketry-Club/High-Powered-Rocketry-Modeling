use crate::state_module::State;
use crate::throw_error;

pub(crate) struct ODE {
    deltat: f64,
    integration_method: i32,
}

impl ODE {
    pub(crate) fn new(deltat: f64, integration_method: i32) -> ODE {
        ODE {
            deltat,
            integration_method,
        }
    }

    pub(crate) fn timestep(&self, state: &mut State) {
        match self.integration_method {
            1 => self.explicit_euler(state),
            2 => self.runge_kutta_3(state),
            _ => {
                crate::throw_error!("Invalid ODE Integration Method", self.integration_method);
                std::process::exit(1);
            }
        }
    }

    fn explicit_euler(&self, state: &mut State) {
        let (dhdt, dvdt) = state.get_derivs();
        let dh = dhdt * self.deltat;
        let dv = dvdt * self.deltat;
        state.update(dh, dv, self.deltat)
    }

    fn runge_kutta_3(&self, state: &mut State) {
        //Based off Strong Stability Preserving (SSP) aka. Total variation Diminishing (TVD) RK3
        let mut state_rk: State = state.copy();

        //Stage 1       dt = 1 * DT
        let (dhdt, dvdt) = state.get_derivs();
        let mut dh = dhdt * self.deltat;
        let mut dv = dvdt * self.deltat;
        state_rk.update(dh,dv, 0.0);

        //Stage 2       dt = 0.5 * DT
        let (dhdt2, dvdt2) = state_rk.get_derivs();
        dh = (dhdt*0.25 + dhdt2*0.25) * self.deltat;
        dv = (dvdt * 0.25 + dvdt2*0.25) * self.deltat;
        state_rk = state.copy();
        state_rk.update(dh,dv, 0.0);

        //Stage 3
        let (dhdt3, dvdt3) = state_rk.get_derivs();
        let coeff = 1.0 /6.0;
        dh = (dhdt*coeff + dhdt2*coeff + dhdt3*coeff*4.0) * self.deltat;
        dv = (dvdt*coeff + dvdt2*coeff + dvdt3*coeff*4.0) * self.deltat;
        state.update(dh, dv, self.deltat);
    }
}
