use crate::state_module::State;

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
}
