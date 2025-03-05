mod math;
mod physics_mod;
mod plotting;
mod rocket_mod;
mod simdata_mod;
mod simulation_mod;
mod state;

use plotters::prelude::*;
use pyo3::exceptions::PyTypeError;
use pyo3::prelude::*;
use std::f64::consts::PI;
use std::io::BufRead;

use crate::math::ode::OdeMethod;
use crate::plotting::{add_line_to_plot, make_line_plot, plot_plot};
use crate::rocket_mod::Rocket;
use crate::simdata_mod::{SimulationData};
use crate::simulation_mod::Simulation;
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
fn main(test_rocket: Rocket, py_state: &mut PyState, ode_method: &OdeMethod) -> PyResult<()> {

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
    let mut data : SimulationData<18> = SimulationData::new(); //18 is hardcoded as max value
    case.run(&mut data);
    println!(
        "Apogee {:6.2}\n",
        case.apogee(),
    );
    // ========== Plotting Results ==========
    /*
    //Altitude
    let (mut chart1, root1) = make_line_plot(
        "test.png",
        "Altitude Comparison",
        "Time (s)",
        0,
        "Altitude (m)",
        1,
        "",
        &RED,
        [0.0, 10.0],
        [0.0, 550.0],
        &data,
    )
    .expect("TODO: panic message");
    //let _ = add_line_to_plot(0, 2, "RK3 3Dof", &BLUE, &data_rk3, &mut chart1);
    plot_plot(&mut chart1, &root1);

    // Need to generalize, might just move to python w/ matplotlib

    //Flight Path
    let (mut ch2, rt2) = make_line_plot(
        "test2.png",
        "Test Rocket Flight Path 2D",
        "X (m)",
        1,
        "Y (m)",
        2,
        "RK3 3Dof",
        &RED,
        [-50.0, 50.0],
        [0.0, 550.0],
        &data_rk3,
    )
    .expect("TODO: panic message");
    plot_plot(&mut ch2, &rt2);

    //Rocket Orientation
    let (mut ch3, rt3) = make_line_plot(
        "test3.png",
        "Test Rocket Flight Angle 2D",
        "Tims (s)",
        0,
        "Orintation (radians CCW)",
        3,
        "RK3 3Dof",
        &RED,
        [0.0, 11.0],
        [-5., 5.0],
        &data_rk3,
    )
    .expect("TODO: panic message");
    plot_plot(&mut ch3, &rt3);
    */
    Ok(())
}

#[pymodule]
fn hprm(m: &Bound<'_, PyModule>) -> PyResult<()> {
    m.add_function(wrap_pyfunction!(main, m)?)?;
    m.add_class::<Rocket>()?;
    m.add_class::<PyState>()?;
    m.add_class::<OdeMethod>()?;
    Ok(())
}
