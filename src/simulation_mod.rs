use crate::math::ode::OdeMethod;
use crate::math::vec_ops::MathVector;
use crate::simdata_mod::{SimulationData};
use crate::state_mod::{State};

pub(crate) struct Simulation {
    // Struct used to coordinate the execution of a simulation. It is supplied with a
    // State space/model, and a timestepping method, and will carry out iterations until a stopping
    // criterea is reached, or the maximum number of iterations have been carried out.
    state: State,
    nvar: usize,
    ode: OdeMethod,
    exit_condition: i32,
    pub(crate) iter: u64,
    maxiter: u64,
}
impl Simulation {
    pub(crate) fn new(
        state: State,
        ode: OdeMethod,
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
        }
    }

    pub(crate) fn run<const L: usize>(&mut self, mut log: &SimulationData<[f64;L]>) {
        //Executes the simulation
        //const NLOG: usize = L;

        for i in 0..self.maxiter {
            let dudt = self.state.get_derivs();
            let u = self.state.get_state_vec();

            //Add data to sata struct
            //let mut row = [0.0; NLOG];
            //row[0..u.len].copy_from_slice(&u);
            //row[u.len..NLOG].copy_from_slice(&dudt.data[..(NLOG-u.len)]);
            //let row : MathVector<L> = u;
            //*log.add_row(u, self.state.get_time());

            //Check for Exit Condition
            if self.is_done() {
                self.iter = i;
                println!("\n==================== Calculation complete! ================================================================================");
                self.state.print_state(i);
                println!("===========================================================================================================================\n");
                break;
            }
            //Output simulation info to terminal
            if i % 1 == 0 {
                self.state.print_state(i);
            }

            //Advance the calculation
            self.ode.timestep(&mut self.state);
        }
    }
    pub(crate) fn apogee(&mut self) -> f64 {
        // Getter to obtain the apogee of aa flight after the simulation is complete
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
