use crate::math_module::ODE;
use crate::state_module::State;

pub(crate) struct Simulation {
    state: State,
    ode: ODE,
    exit_condition: i32,
    iter: u64,
    maxiter: u64,
}
impl Simulation {
    pub(crate) fn new(state: State, ode: ODE, exit_condition: i32, maxiter: u64) -> Simulation {
        Simulation {
            state,
            ode,
            exit_condition,
            iter: 0,
            maxiter,
        }
    }

    pub(crate) fn run(&mut self) {
        //Executes the simulation
        for i in 0..self.maxiter {
            self.iter = i;

            if self.is_done() {
                println!("\n==================== Calculation complete! ====================");
                println!(
                    "Iter:{:6},    Time:{:5.2}(s),    Altitude:{:8.2}(m),    Velocity:{:8.2}(m/s)    Acceleration:{:8.2}(m/ss)\n",
                    i,
                    self.state.get_time(),
                    self.state.get_height(),
                    self.state.get_velocity(),
                    self.state.get_derivs().1
                );
                break;
            }

            self.ode.timestep(&mut self.state);

            if i % 100 == 0 {
                println!(
                    "Iter:{:6},    Time:{:5.2},    Altitude:{:8.2},    Velocity:{:8.2}    Acceleration:{:8.2}",
                    i,
                    self.state.get_time(),
                    self.state.get_height(),
                    self.state.get_velocity(),
                    self.state.get_derivs().1
                );
            }
        }
    }
    pub(crate) fn apogee(&mut self) -> f64 {
        if !self.is_done() {
            println!("Apogee requested before simulation has been run!!!\n");
            f64::NAN
        } else {
            self.state.get_height()
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
                std::process::exit(1);
            }
        }
    }
    fn condition_one(&self) -> bool {
        // Stop calculation when apogee is reached
        if self.state.get_velocity() < 0.0 {
            true
        } else {
            false
        }
    }
}
