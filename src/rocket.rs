use crate::ode::{OdeSolver, TimeStepOptions};
use crate::simdata_mod::SimulationData;
use crate::simulation::{Simulation, SimulationExitCondition};
use crate::state::{InitialCondition,State};
use crate::state::state_vector::{StateVector};
use pyo3::prelude::*;

#[pyclass(eq, eq_int)]
#[derive(Clone, Copy, PartialEq, Debug)]
/// Represents the type of dynamic model used for the simulation.
pub enum ModelType {
    /// One degree of freedom model, modeling the rocket as only going up and down (y).
    OneDOF,
    /// Three degrees of freedom model, modeling the rocket in 2D space with rotation (x, y, theta).
    ThreeDOF,
}

#[pyclass(eq, eq_int)]
#[derive(Clone, Copy, PartialEq, Debug)]
/// Numerical integration methods for the ODE solver.
pub enum OdeMethod {
    /// First-order explicit Euler method.
    Euler,
    /// Third-order Runge-Kutta method.
    RK3,
    /// Fourth-order Runge-Kutta method with adaptive time stepping.
    RK45,
}

#[pyclass(dict, get_all, set_all)]
#[derive(Debug, Clone, Copy)]
/// Represents the physical properties of the rocket used in the simulation.
pub(crate) struct Rocket {
    /// Mass of the rocket (kg)
    pub(crate) mass: f64,
    /// Drag coefficient
    pub(crate) cd: f64,
    /// Reference area for drag (m^2)
    pub(crate) area_drag: f64,
    /// Reference area for lift (m^2)
    pub(crate) area_lift: f64,
    /// Moment of inertia about the z-axis (kg*m^2)
    pub(crate) moment_of_inertia: f64,
    /// Static stability margin (m)
    pub(crate) stab_margin_dimensional: f64,
    /// Lift coefficient slope (per radian)
    pub(crate) cl_a: f64,
}

#[pymethods]
impl Rocket {
    #[new]
    pub(crate) fn new(
        mass: f64,
        cd: f64,
        area_drag: f64,
        area_lift: f64,
        moment_of_inertia: f64,
        stab_margin_dimensional: f64,
        cl_a: f64,
    ) -> Self {
        Self {
            mass,
            cd,
            area_drag,
            area_lift,
            moment_of_inertia,
            stab_margin_dimensional,
            cl_a,
        }
    }

    #[pyo3(signature = (initial_condition, integration_method, timestep_config=None, print_output=false, log_output=false))]
    fn simulate_flight(
        &self,
        initial_condition: &impl InitialCondition,
        integration_method: OdeMethod,
        timestep_config: Option<TimeStepOptions>,
        print_output: bool,
        log_output: bool,
    ) -> PyResult<SimulationData> {
        let ode_solver = OdeSolver::from_method(integration_method, timestep_config)?;

        let state = State::from_initial_condition(
            *self,
            initial_condition,
        );

        const MAXITER: u64 = 1e5 as u64;
        let mut simulation = Simulation::new(
            state,
            ode_solver,
            SimulationExitCondition::ApogeeReached,
            MAXITER,
        );
        let mut log = SimulationData::new();
        simulation.run(&mut log, print_output, log_output);
        Ok(log)
    }

    #[pyo3(signature = (initial_condition, integration_method, timestep_config=None, initial_angle=None, print_output=false))]
    fn predict_apogee(
        &self,
        initial_condition: &impl InitialCondition,
        integration_method: OdeMethod,
        timestep_config: Option<TimeStepOptions>,
        initial_angle: Option<f64>,
        print_output: bool,
    ) -> PyResult<f64> {
        let log = self.simulate_flight(
            initial_condition,
            integration_method,
            timestep_config,
            print_output,
            false,
        )?;

        // Gets the height column based on model type
        let height_col = match initial_condition.as_statevector() {
            StateVector::__1DOF(_v) => 1,
            StateVector::__3DOF(_v) => 2,
            _ => 0,
        };
        /// Man that is janky, someone has to make that better  -Sailor

        let max_height = log.get_val((log.len as usize)-1, height_col);

        Ok(max_height)
    }
}
