mod constants;
mod ode;
mod physics_mod;
mod rocket;
mod simdata_mod;
mod simulation;
mod state;

use pyo3::exceptions::PyTypeError;
use pyo3::prelude::*;
use std::f64::consts::PI;

use crate::ode::{AdaptiveTimeStep, FixedTimeStep, OdeMethod, OdeSolver, TimeStepOptions};
use crate::rocket::Rocket;
use crate::simulation::Simulation;
use crate::state::{model_1dof::OneDOFModel, model_3dof::ThreeDOFModel, State};

#[pymodule(gil_used = false)]
fn hprm(m: &Bound<'_, PyModule>) -> PyResult<()> {
    m.add_class::<OdeMethod>()?;
    m.add_class::<Rocket>()?;
    m.add_class::<FixedTimeStep>()?;
    m.add_class::<AdaptiveTimeStep>()?;
    Ok(())
}
