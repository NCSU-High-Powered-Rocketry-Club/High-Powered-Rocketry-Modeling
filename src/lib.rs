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

use crate::ode::OdeSolver;
use crate::simulation::Simulation;
use crate::state::{InitialState1DOF, InitialState3DOF};
use crate::state::{model_1dof::OneDOFModel, model_3dof::ThreeDOFModel, State};

pub use crate::ode::{AdaptiveTimeStep, FixedTimeStep, OdeMethod, TimeStepOptions};
pub use crate::rocket::{Rocket, RocketProperties};

#[pymodule(gil_used = false)]
fn hprm(m: &Bound<'_, PyModule>) -> PyResult<()> {
    m.add_class::<OdeMethod>()?;
    m.add_class::<Rocket>()?;
    m.add_class::<FixedTimeStep>()?;
    m.add_class::<AdaptiveTimeStep>()?;
    m.add_class::<InitialState1DOF>()?;
    m.add_class::<InitialState3DOF>()?;
    Ok(())
}
