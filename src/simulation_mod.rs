use crate::math_mod::OdeIterators;
use crate::state_mod::{State, StateVector};
use crate::simdata_mod::SimulationData;

pub(crate) struct Simulation {
    state: State,
    nvar: usize,
    ode: OdeIterators,
    exit_condition: i32,
    iter: u64,
    maxiter: u64,
    data : SimulationData
}
impl Simulation {
    pub(crate) fn new(
        state: State,
        ode: OdeIterators,
        exit_condition: i32,
        maxiter: u64,
    ) -> Simulation {
        Simulation {
            state,
            nvar: state.get_ndim(),
            ode,
            exit_condition,
            iter: 0,
            maxiter,
            data : SimulationData::new(&state)
        }
    }

    pub(crate) fn run(&mut self) {
        //Executes the simulation
        for i in 0..self.maxiter {
            let dudt = self.state.get_derivs();
            let u = self.state.get_state_vec();
            
            //log data
            self.data.add_row((u, dudt), self.state.get_time());
                
            //Check for Exit Condition
            if self.is_done() {
                self.iter = i;
                println!("\n==================== Calculation complete! ================================================================================");
                self.state.print_state(i);
                println!("===========================================================================================================================\n");
                break;
            }
            //Output Simulation Info
            if i % 100 == 0 {
                self.state.print_state(i);
            }

            //Advance the calculation
            self.ode.timestep(&mut self.state);
        }
    }
    pub(crate) fn apogee(&mut self) -> f64 {
        if !self.is_done() {
            println!("Apogee requested before simulation has been run!!!\n");
            f64::NAN
        } else {
            self.state.get_altitude()
        }
    }
    //
    //
    //
    fn is_done(&self) -> bool {
        match self.exit_condition {
            1 => self.condition_one(),
            _ => {
                crate::throw_error!("Invalid Simulation End Criterion", self.exit_condition);
                true
            }
        }
    }
    fn condition_one(&self) -> bool {
        // Stop calculation when apogee is reached
        if self.state.get_vertical_velocity() < 0.0 {
            true
        } else {
            false
        }
    }
}
