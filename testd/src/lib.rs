mod math;
mod physics_mod;
mod plotting;
mod rocket_mod;
mod simdata_mod;
mod simulation_mod;
mod state;

use std::f64::consts::PI;
use plotters::prelude::*;
use std::io::BufRead;
use pyo3::prelude::*;

use crate::math::ode::OdeMethod;
use crate::plotting::{add_line_to_plot, make_line_plot, plot_plot};
use crate::rocket_mod::Rocket;
use crate::simdata_mod::SimulationData;
use crate::simulation_mod::Simulation;
use crate::state::{model_1dof::Dof1, model_3dof::Dof3, State};

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
fn main() -> PyResult<()> {
    // Rocket Parameters
    let mass: f64 = 10.0; //kg
    let cd: f64 = 0.3;
    let cl_alpha: f64 = 0.2;
    let area_drag: f64 = 0.005; // m^2
    let area_lift: f64 = 0.05;
    let inertia: f64 = 5.0;
    let stab_mgn: f64 = 0.5;
    let test_rocket: Rocket =
        Rocket::new(mass, cd, area_drag, area_lift, inertia, stab_mgn, cl_alpha);

    // Initial Conditions
    let u0: [f64; 2] = [0.0, 100.0]; // m, m/s
    let state_euler = State::__1DOF(Dof1::new(u0, test_rocket.clone()));

    let u0: [f64; 6] = [0.0, 0.0, PI/2.0, 0.0, 100.0, 0.0]; // m, m, rad, m/s, m/s, rad/s
    let state_rk3 = State::__3DOF(Dof3::new(u0, test_rocket.clone()));

    // iteration/calculation Parameters
    const MAXITER: u64 = 1e5 as u64; //Maximum number of iterations before stopping calculation
    const DT: f64 = 1e-2 as f64; //Timestep size to use when integrating ODE
    let euler_method = OdeMethod::Euler(DT);
    let rk3 = OdeMethod::RK3(DT);

    //Assemble Simulation Struct
    let mut case_euler: Simulation = Simulation::new(state_euler.clone(), euler_method, 1, MAXITER);
    let mut case_rk3: Simulation = Simulation::new(state_rk3.clone(), rk3, 1, MAXITER);

    //Create Data Structures
    let mut data_euler: SimulationData<{ Dof1::NLOG }> = SimulationData::new();
    let mut data_rk3: SimulationData<{ Dof3::NLOG }> = SimulationData::new();

    case_euler.run(&mut data_euler);
    case_rk3.run(&mut data_rk3);

    println!(
        "Euler, 1Dof: Apogee {:6.2}\nRK3, 3Dof  : Apogee {:6.2}\n",
        case_euler.apogee(),
        case_rk3.apogee()
    );

    // ========== Plotting Results ==========

    //Altitude
    let (mut chart1, root1) = make_line_plot(
        "test.png",
        "Altitude Comparison",
        "Time (s)",
        0,
        "Altitude (m)",
        1,
        "Euler's Method, 1Dof",
        &RED,
        [0.0, 10.0],
        [0.0, 550.0],
        &data_euler,
    )
    .expect("TODO: panic message");
    let _ =add_line_to_plot(0, 2, "RK3 3Dof", &BLUE, &data_rk3, &mut chart1);
    plot_plot(&mut chart1, &root1);

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
    ).expect("TODO: panic message");
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
    ).expect("TODO: panic message");
    plot_plot(&mut ch3, &rt3);

    Ok(())
}


#[pymodule]
fn testd(m: &Bound<'_, PyModule>) -> PyResult<()> {
    m.add_function(wrap_pyfunction!(main, m)?)?;
    Ok(())
}
