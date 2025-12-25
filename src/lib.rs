mod ode;
mod physics_mod;
mod rocket;
mod simdata_mod;
mod simulation;
mod state;

use pyo3::exceptions::PyTypeError;
use pyo3::prelude::*;
use std::f64::consts::PI;

use crate::ode::{OdeSolver, TimeStepOptions};
use crate::rocket::{ModelType, OdeMethod, Rocket};
use crate::simdata_mod::SimulationData;
use crate::simulation::Simulation;
use crate::state::{model_1dof::DOF1, model_3dof::DOF3, State};

#[pymodule(gil_used = false)]
fn hprm(m: &Bound<'_, PyModule>) -> PyResult<()> {
    m.add_class::<ModelType>()?;
    m.add_class::<OdeMethod>()?;
    m.add_class::<Rocket>()?;
    m.add_class::<SimulationData>()?;
    m.add_class::<crate::ode::FixedTimeStep>()?;
    m.add_class::<crate::ode::AdaptiveTimeStep>()?;
    Ok(())
}
