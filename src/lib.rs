mod math;
mod physics_mod;
mod simdata_mod;
mod simulation;
mod state;

use pyo3::exceptions::PyTypeError;
use pyo3::prelude::*;
use std::f64::consts::PI;

use crate::math::ode::{AdaptiveTimeStep, FixedTimeStep, OdeSolver, TimeStepOptions};
use crate::simdata_mod::{SimulationData};
use crate::simulation::Simulation;
use crate::state::{model_1dof::Dof1, model_3dof::Dof3, State};

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

#[pyclass(dict,get_all,set_all)]
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
    pub(crate) inertia_z: f64,
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
        inertia_z: f64,
        stab_margin_dimensional: f64,
        cl_a: f64,
    ) -> Self {
        Self {
            mass,
            cd,
            area_drag,
            area_lift,
            inertia_z,
            stab_margin_dimensional,
            cl_a,
        }
    }
    
    #[pyo3(signature = (initial_height, initial_velocity, model_type, integration_method, timestep_config=None, initial_angle=None))]
    fn simulate_flight(
        &self,
        initial_height: f64,
        initial_velocity: f64,
        model_type: ModelType,
        integration_method: OdeMethod,
        timestep_config: Option<TimeStepOptions>,
        initial_angle: Option<f64>,
    ) -> PyResult<SimulationData> {
        let ode_solver = OdeSolver::from_method(integration_method, timestep_config)?;

        let state = State::from_model_type(
            model_type,
            *self,
            initial_height,
            initial_velocity,
            initial_angle,
        );

        const MAXITER: u64 = 1e5 as u64;
        let mut simulation = Simulation::new(state, ode_solver, 1, MAXITER);
        let mut log = SimulationData::new();
        simulation.run(&mut log);
        Ok(log)
    }
}

#[pymodule]
fn hprm(m: &Bound<'_, PyModule>) -> PyResult<()> {
    m.add_class::<ModelType>()?;
    m.add_class::<OdeMethod>()?;
    m.add_class::<Rocket>()?;
    m.add_class::<SimulationData>()?;
    m.add_class::<crate::math::ode::FixedTimeStep>()?;
    m.add_class::<crate::math::ode::AdaptiveTimeStep>()?;
    Ok(())
}