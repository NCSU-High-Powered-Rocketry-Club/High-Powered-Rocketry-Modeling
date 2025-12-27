use crate::ode::OdeSolver;
use crate::simdata_mod::SimulationData;
use crate::state::State;

/// Enum defining the various exit conditions for the simulation. Eventually, more exit
/// conditions, such as ground impact, can be added here.
pub enum SimulationExitCondition {
    ApogeeReached,
    // TODO: Add more exit conditions as needed
}

/// Struct used to coordinate the execution of a simulation. It is supplied with a
/// State space/model, and a timestepping method, and will carry out iterations until a stopping
/// criterea is reached, or the maximum number of iterations have been carried out.
pub(crate) struct Simulation {
    state: State,
    ode: OdeSolver,
    exit_condition: SimulationExitCondition,
    pub(crate) current_iteration: u64,
    max_iterations: u64,
}

impl Simulation {
    pub(crate) fn new(
        state: State,
        ode: OdeSolver,
        exit_condition: SimulationExitCondition,
        max_iterations: u64,
    ) -> Simulation {
        Simulation {
            state,
            ode,
            exit_condition,
            current_iteration: 0,
            max_iterations,
        }
    }

    /// Run the simulation until the exit condition is met or the maximum number of iterations is reached.
    ///
    /// # Arguments
    ///
    /// - `&mut self` (`undefined`) - Describe this parameter.
    /// - `log` (`&mut SimulationData`) - Describe this parameter.
    /// - `print_output` (`bool`) - Describe this parameter.
    pub(crate) fn run(&mut self, log: &mut SimulationData, print_output: bool) {
        // Executes the simulation
        for i in 0..self.max_iterations {
            let old_state = self.state.clone();

            self.current_iteration = i;
            log.add_row(self.state.get_logrow(), self.state.get_time());

            // Output simulation info to terminal
            if print_output && i % 1 == 0 {
                self.state.print_state(i);
            }

            // Advance the calculation
            self.ode.timestep(&mut self.state);
            //
            // Check Exit Condition
            if self.is_done() {
                // Mitigate overshoot errors
                match self.exit_condition {
                    SimulationExitCondition::ApogeeReached => self.ode.backtrack_apogee(&mut self.state, &old_state)
                }
                //
                if print_output {
                    println!("\n==================== Calculation complete! ================================================================================");
                    self.state.print_state(i+1);
                    println!("===========================================================================================================================\n");
                }

                break;
            }
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

    fn is_done(&self) -> bool {
        match self.exit_condition {
            SimulationExitCondition::ApogeeReached => self.condition_apogee(),
        }
    }

    fn condition_apogee(&self) -> bool {
        // Stop calculation when apogee is reached
        let tolerance : f64 = 1.0; // m/s
        self.state.get_vertical_velocity() < tolerance
    }
}

#[cfg(test)]
mod tests {
    use crate::{ode::FixedTimeStep, rocket::Rocket, state::model_1dof::DOF1};

    use super::*;
    use approx::assert_abs_diff_eq;
    use nalgebra::Vector2;

    fn make_simulation() -> Simulation {
        let rocket = Rocket {
            mass: 50.0,
            cd: 0.75,
            area_drag: 0.03,
            area_lift: 0.0,
            moment_of_inertia: 0.0,
            stab_margin_dimensional: 0.0,
            cl_a: 0.0,
        };

        let state = State::__1DOF(DOF1::new(Vector2::new(0.0, 100.0), rocket));

        let ode_solver = OdeSolver::Euler(FixedTimeStep { dt: 0.1 });

        Simulation::new(
            state,
            ode_solver,
            SimulationExitCondition::ApogeeReached,
            100,
        )
    }

    #[test]
    fn test_run() {
        // Tests that running a simple simulation completes without error
        let mut simulation = make_simulation();
        let max_iterations: u64 = 100;
        simulation.max_iterations = max_iterations;

        assert!(!simulation.is_done());

        simulation.run(&mut SimulationData::new(), false);

        assert!(simulation.is_done());
        assert!(simulation.current_iteration <= max_iterations);

        // Tests that running a simulation with too low max_iterations stops correctly
        let mut simulation = make_simulation();
        let max_iterations: u64 = 5;
        simulation.max_iterations = max_iterations;

        assert!(!simulation.is_done());

        simulation.run(&mut SimulationData::new(), false);

        assert!(!simulation.is_done());
        // We have to do - 1 here because current_iteration is zero-indexed
        assert_eq!(simulation.current_iteration, max_iterations - 1);
    }

    #[test]
    fn test_exit_condition_apogee_reached() {
        // ApogeeReached should trigger strictly when vertical velocity < 0
        let rocket = Rocket {
            mass: 50.0,
            cd: 0.75,
            area_drag: 0.03,
            area_lift: 0.0,
            moment_of_inertia: 0.0,
            stab_margin_dimensional: 0.0,
            cl_a: 0.0,
        };

        let state_positive_vel = State::__1DOF(DOF1::new(Vector2::new(0.0, 100.0), rocket));
        let sim_positive_vel = Simulation::new(
            state_positive_vel,
            OdeSolver::Euler(FixedTimeStep { dt: 0.1 }),
            SimulationExitCondition::ApogeeReached,
            100,
        );
        assert!(!sim_positive_vel.is_done());

        // v < 0.0 => done
        let rocket2 = Rocket {
            mass: 50.0,
            cd: 0.75,
            area_drag: 0.03,
            area_lift: 0.0,
            moment_of_inertia: 0.0,
            stab_margin_dimensional: 0.0,
            cl_a: 0.0,
        };
        let state_negative_vel = State::__1DOF(DOF1::new(Vector2::new(0.00, -0.01), rocket2));
        let sim_negative_vel = Simulation::new(
            state_negative_vel,
            OdeSolver::Euler(FixedTimeStep { dt: 0.1 }),
            SimulationExitCondition::ApogeeReached,
            100,
        );
        assert!(sim_negative_vel.is_done());
    }

    #[test]
    fn test_apogee() {
        // Before running, the sim should not be done, so apogee() returns NaN
        let mut simulation = make_simulation();
        assert!(!simulation.is_done());

        let a0 = simulation.apogee();
        assert!(a0.is_nan());

        // After running, it should be done, and apogee should be a finite altitude
        simulation.run(&mut SimulationData::new(), false);

        assert!(simulation.is_done());

        let a1 = simulation.apogee();
        assert!(a1.is_finite());
        assert!(a1 >= 0.0);
    }
}
