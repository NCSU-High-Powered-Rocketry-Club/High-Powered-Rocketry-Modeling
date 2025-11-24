mod math;
mod physics_mod;
mod simdata_mod;
mod simulation;
mod state;

use pyo3::exceptions::PyTypeError;
use pyo3::prelude::*;
use std::f64::consts::PI;
use std::io::BufRead;

use crate::math::ode::{AdaptiveTimeStep, FixedTimeStep, OdeMethod, TimeStepOptions};
use crate::simdata_mod::{SimulationData};
use crate::simulation::Simulation;
use crate::state::{model_1dof::Dof1, model_3dof::Dof3, State};

#[pyclass(eq, eq_int)]
#[derive(Clone, Copy, PartialEq, Debug)]
pub enum ModelType {
    OneDOF,
    ThreeDOF,
}

#[pyclass(eq, eq_int)]
#[derive(Clone, Copy, PartialEq, Debug)]
pub enum IntegrationMethod {
    Euler,
    RK3,
    RK45,
}

#[pyclass(dict,get_all,set_all)]
#[derive(Debug, Clone, Copy)]
pub(crate) struct Rocket {
    pub(crate) mass: f64,
    pub(crate) cd: f64,
    pub(crate) area_drag: f64,
    pub(crate) area_lift: f64,
    pub(crate) inertia_z: f64,
    pub(crate) stab_margin_dimensional: f64,
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
    
    #[pyo3(signature = (initial_height, initial_velocity, model_type, integration_method, timestep_config=None))]
    fn simulate_flight(
        &self,
        initial_height: f64,
        initial_velocity: f64,
        model_type: ModelType,
        integration_method: IntegrationMethod,
        timestep_config: Option<TimeStepOptions>,
    ) -> PyResult<SimulationData> {
        let ode_method = match (integration_method, timestep_config) {
            (IntegrationMethod::Euler, Some(TimeStepOptions::Fixed(f))) => OdeMethod::Euler(f),
            (IntegrationMethod::Euler, None) => OdeMethod::Euler(FixedTimeStep::new(0.01)),
            (IntegrationMethod::Euler, Some(TimeStepOptions::Adaptive(_))) => return Err(PyTypeError::new_err("Euler requires FixedTimeStep")),

            (IntegrationMethod::RK3, Some(TimeStepOptions::Fixed(f))) => OdeMethod::RK3(f),
            (IntegrationMethod::RK3, None) => OdeMethod::RK3(FixedTimeStep::new(0.01)),
            (IntegrationMethod::RK3, Some(TimeStepOptions::Adaptive(_))) => return Err(PyTypeError::new_err("RK3 requires FixedTimeStep")),

            (IntegrationMethod::RK45, Some(TimeStepOptions::Adaptive(a))) => OdeMethod::RK45(a),
            (IntegrationMethod::RK45, None) => OdeMethod::RK45(AdaptiveTimeStep::new()),
            (IntegrationMethod::RK45, Some(TimeStepOptions::Fixed(_))) => return Err(PyTypeError::new_err("RK45 requires AdaptiveTimeStep")),
        };

        let state = match model_type {
            ModelType::OneDOF => {
                let u1 = [initial_height, initial_velocity];
                State::__1DOF(Dof1::new(u1, self.clone()))
            },
            ModelType::ThreeDOF => {
                // u3 = [x, y, theta, vx, vy, omega]
                // y = initial_height, vy = initial_velocity, theta = PI/2
                let u3 = [0.0, initial_height, PI / 2.0, 0.0, initial_velocity, 0.0];
                State::__3DOF(Dof3::new(u3, self.clone()))
            }
        };

        const MAXITER: u64 = 1e5 as u64;
        let mut simulation = Simulation::new(state, ode_method, 1, MAXITER);
        let mut log = SimulationData::new();
        simulation.run(&mut log);
        Ok(log)
    }
}

#[pymodule]
fn hprm(m: &Bound<'_, PyModule>) -> PyResult<()> {
    m.add_class::<ModelType>()?;
    m.add_class::<IntegrationMethod>()?;
    m.add_class::<Rocket>()?;
    m.add_class::<SimulationData>()?;
    m.add_class::<crate::math::ode::FixedTimeStep>()?;
    m.add_class::<crate::math::ode::AdaptiveTimeStep>()?;
    Ok(())
}