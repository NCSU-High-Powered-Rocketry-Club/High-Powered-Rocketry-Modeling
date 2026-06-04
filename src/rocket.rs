use crate::constants::simulation_constants::{DATA_LENGTH, MAX_ITERATIONS};
use crate::ode::{OdeSolver, TimeStepOptions};
use crate::simdata_mod::SimulationData;
use crate::simulation::{Simulation, SimulationExitCondition};
use crate::state::State;
use numpy::{ndarray::Array2, PyArray1, PyArray2, ToPyArray};
use pyo3::prelude::*;
use pyo3::Bound;

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

    #[pyo3(signature = (initial_height, initial_velocity, integration_method, timestep_config=None, max_iterations=MAX_ITERATIONS, print_output=false, log_output=false))]
    #[allow(clippy::too_many_arguments)]
    #[allow(clippy::type_complexity)]
    fn simulate_flight_1dof<'py>(
        &self,
        py: Python<'py>,
        initial_height: f64,
        initial_velocity: f64,
        integration_method: OdeMethod,
        timestep_config: Option<TimeStepOptions>,
        max_iterations: u64,
        print_output: bool,
        log_output: bool,
    ) -> PyResult<(Bound<'py, PyArray1<f64>>, Bound<'py, PyArray2<f64>>)> {
        // Create the ODE solver based on the specified integration method and time step configuration
        let ode_solver = OdeSolver::from_method(integration_method, timestep_config)?;
        // Initialize the state of the rocket for a 1DOF simulation
        let state = State::new_1dof(*self, initial_height, initial_velocity);

        // Create a new simulation instance with the initialized state, ODE solver, and exit condition
        let mut simulation = Simulation::new(
            state,
            ode_solver,
            SimulationExitCondition::ApogeeReached,
            max_iterations,
        );

        // Run the simulation and log the results into our custom SimulationData struct
        let mut log = SimulationData::new();
        simulation.run(&mut log, print_output, log_output);

        // Then converts the logged time and state data into NumPy arrays to return to Python
        let time_array = log.time_log.to_pyarray(py);
        let rows = log.time_log.len();
        // First flattens the 2D state log into a 1D vector, then reshapes it back into a 2D array
        let flat_data: Vec<f64> = log.state_log.iter().flatten().copied().collect();
        let matrix = Array2::from_shape_vec((rows, DATA_LENGTH), flat_data).unwrap();
        let state_matrix = matrix.to_pyarray(py);

        Ok((time_array, state_matrix))
    }

    #[pyo3(signature = (initial_height, initial_velocity, initial_angle, integration_method, timestep_config=None, max_iterations=MAX_ITERATIONS, print_output=false, log_output=false))]
    #[allow(clippy::too_many_arguments)]
    #[allow(clippy::type_complexity)]
    fn simulate_flight_3dof<'py>(
        &self,
        py: Python<'py>,
        initial_height: f64,
        initial_velocity: f64,
        initial_angle: f64,
        integration_method: OdeMethod,
        timestep_config: Option<TimeStepOptions>,
        max_iterations: u64,
        print_output: bool,
        log_output: bool,
    ) -> PyResult<(Bound<'py, PyArray1<f64>>, Bound<'py, PyArray2<f64>>)> {
        let ode_solver = OdeSolver::from_method(integration_method, timestep_config)?;

        let state = State::new_3dof(*self, initial_height, initial_velocity, initial_angle);

        let mut simulation = Simulation::new(
            state,
            ode_solver,
            SimulationExitCondition::ApogeeReached,
            max_iterations,
        );
        let mut log = SimulationData::new();
        simulation.run(&mut log, print_output, log_output);

        let time_array = log.time_log.to_pyarray(py);
        let rows = log.time_log.len();
        let flat_data: Vec<f64> = log.state_log.iter().flatten().copied().collect();
        let matrix = Array2::from_shape_vec((rows, DATA_LENGTH), flat_data).unwrap();
        let state_matrix = matrix.to_pyarray(py);

        Ok((time_array, state_matrix))
    }

    #[pyo3(signature = (initial_height, initial_velocity, integration_method, timestep_config=None, max_iterations=MAX_ITERATIONS, print_output=false))]
    #[allow(clippy::too_many_arguments)]
    fn predict_apogee_1dof(
        &self,
        initial_height: f64,
        initial_velocity: f64,
        integration_method: OdeMethod,
        timestep_config: Option<TimeStepOptions>,
        max_iterations: u64,
        print_output: bool,
    ) -> PyResult<f64> {
        let ode_solver = OdeSolver::from_method(integration_method, timestep_config)?;

        let state = State::new_1dof(*self, initial_height, initial_velocity);

        let mut simulation = Simulation::new(
            state,
            ode_solver,
            SimulationExitCondition::ApogeeReached,
            max_iterations,
        );

        let mut log = SimulationData::new();
        simulation.run(&mut log, print_output, false);

        const HEIGHT_COL: usize = 1;

        let max_height = log.get_val((log.len as usize) - 1, HEIGHT_COL);

        Ok(max_height)
    }

    #[pyo3(signature = (initial_height, initial_velocity, initial_angle, integration_method, timestep_config=None, max_iterations=MAX_ITERATIONS, print_output=false))]
    #[allow(clippy::too_many_arguments)]
    fn predict_apogee_3dof(
        &self,
        initial_height: f64,
        initial_velocity: f64,
        initial_angle: f64,
        integration_method: OdeMethod,
        timestep_config: Option<TimeStepOptions>,
        max_iterations: u64,
        print_output: bool,
    ) -> PyResult<f64> {
        let ode_solver = OdeSolver::from_method(integration_method, timestep_config)?;

        let state = State::new_3dof(*self, initial_height, initial_velocity, initial_angle);

        let mut simulation = Simulation::new(
            state,
            ode_solver,
            SimulationExitCondition::ApogeeReached,
            max_iterations,
        );

        let mut log = SimulationData::new();
        simulation.run(&mut log, print_output, false);

        const HEIGHT_COL: usize = 2;

        let max_height = log.get_val((log.len as usize) - 1, HEIGHT_COL);

        Ok(max_height)
    }
}
