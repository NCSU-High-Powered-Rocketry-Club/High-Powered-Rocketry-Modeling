use crate::ode::OdeSolver;
use crate::simdata_mod::SimulationData;
use crate::state::State;

pub(crate) struct Simulation {
    // Struct used to coordinate the execution of a simulation. It is supplied with a
    // State space/model, and a timestepping method, and will carry out iterations until a stopping
    // criterea is reached, or the maximum number of iterations have been carried out.
    state: State,
    ode: OdeSolver,
    exit_condition: i32,
    pub(crate) iter: u64,
    maxiter: u64,
}
impl Simulation {
    pub(crate) fn new(
        state: State,
        ode: OdeSolver,
        exit_condition: i32,
        maxiter: u64,
    ) -> Simulation {
        Simulation {
            state,
            ode,
            exit_condition,
            iter: 0,
            maxiter,
        }
    }

    pub(crate) fn run(&mut self, log: &mut SimulationData, print_output: bool) {
        // Executes the simulation
        for i in 0..self.maxiter {
            log.add_row(self.state.get_logrow(), self.state.get_time());

            // Check for exit condition
            if self.is_done() {
                self.iter = i;

                if print_output {
                    println!("\n==================== Calculation complete! ================================================================================");
                    self.state.print_state(i);
                    println!("===========================================================================================================================\n");
                }

                break;
            }

            // Output simulation info to terminal
            if print_output && i % 10 == 0 {
                self.state.print_state(i);
            }

            // Advance the calculation
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
                panic!("Invalid Simulation End Criterion: {}", self.exit_condition);
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
