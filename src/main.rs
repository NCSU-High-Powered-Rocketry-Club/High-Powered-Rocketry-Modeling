mod math_mod;
mod physics_mod;
mod rocket_mod;
mod simulation_mod;
mod state_mod;
mod simdata_mod;

use crate::math_mod::OdeIterators;
use crate::rocket_mod::Rocket;
use crate::simulation_mod::Simulation;
use crate::state_mod::{Dof1, Dof3, State};

#[macro_export]
macro_rules! throw_error {
    ($x:literal,$y:expr) => {
        println!(
            "Program Execution Stopped Due to Error\n{}\nValue : {}\n",
            $x, $y
        )
    };
}

fn main() {
    // Rocket Parameters
    let mass: f64 = 10.0; //kg
    let cd: f64 = 0.3;
    let area: f64 = 0.005; // m^2
    let test_rocket: Rocket = Rocket::new(mass, cd, area);

    // Initial Conditions
    //    let u0: [f64; 2] = [0.0, 100.0]; // m, m/s
    //    let state = State::__1DOF(Dof1::new(u0, test_rocket));

    let u0: [f64; 6] = [0.0, 0.0, 0.0, 0.0, 100.0, 0.0]; // m, m/s
    let state = State::__3DOF(Dof3::new(u0, test_rocket));

    // iteration/calculation Parameters
    const MAXITER: u64 = 1e5 as u64;
    const DT: f64 = 1e-2 as f64;
    let euler_method = OdeIterators::Euler(DT);
    let rk3 = OdeIterators::RK3(DT);

    //Assemble Simulation Struct
    let mut case_euler: Simulation = Simulation::new(state.clone(), euler_method, 1, MAXITER);
    let mut case_rk3: Simulation = Simulation::new(state.clone(), rk3, 1, MAXITER);

    case_euler.run();
    case_rk3.run();

    println!(
        "Euler: Apogee {:6.2}\nRK3  : Apogee {:6.2}\n",
        case_euler.apogee(),
        case_rk3.apogee()
    );
    println!("Try different timestep sizes and see how the different methods' accuracy behaves!")
}
