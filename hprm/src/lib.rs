mod math;
mod physics_mod;
mod rocket_mod;
mod simdata_mod;
mod simulation;
mod state;

use pyo3::exceptions::PyTypeError;
use pyo3::prelude::*;
use std::f64::consts::PI;
use std::io::BufRead;

use crate::math::ode::OdeMethod;
use crate::rocket_mod::Rocket;
use crate::simdata_mod::{SimulationData};
use crate::simulation::Simulation;
use crate::state::{model_1dof::Dof1, model_3dof::Dof3, PyState, State};

#[macro_export]
macro_rules! throw_error {
    ($x:literal,$y:expr) => {
        println!(
            "Program Execution Stopped Due to Error\n{}\nValue : {}\n",
            $x, $y
        )
    };
}

#[pyfunction]
fn sim_apogee(test_rocket: Rocket, py_state: &mut PyState, ode_method: &OdeMethod) -> PyResult<SimulationData> {

    // Initial Conditions
    let state = match py_state.ndof {
        1 => State::__1DOF(Dof1::new(py_state.u1, test_rocket.clone())),
        3 => State::__3DOF(Dof3::new(py_state.u3, test_rocket.clone())),
        //        6 => State::__6DOF(Dof6::new(pystate.u6, test_rocket.clone())),
        _ => {
            return Err(PyErr::new::<PyTypeError, _>(
                "Invalid State Space Chosen. Must be either 1, or 3 DoF.",
            ))
        }
    };
    // iteration/calculation Parameters
    const MAXITER: u64 = 1e5 as u64; //Maximum number of iterations before stopping calculation
    //Assemble Simulation Struct
    let mut case: Simulation = Simulation::new(state.clone(), ode_method.clone(), 1, MAXITER);
    let mut data : SimulationData = SimulationData::new();
    case.run(&mut data);
    println!(
        "Apogee {:6.2}\n",
        case.apogee(),
    );
    Ok(data)
}

#[pymodule]
fn hprm(m: &Bound<'_, PyModule>) -> PyResult<()> {
    m.add_function(wrap_pyfunction!(sim_apogee, m)?)?;
    m.add_class::<Rocket>()?;
    m.add_class::<PyState>()?;
    m.add_class::<OdeMethod>()?;
    m.add_class::<SimulationData>()?;
    Ok(())
}
