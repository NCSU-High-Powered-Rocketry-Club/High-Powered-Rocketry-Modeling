mod math_module;
mod physics_module;
mod rocket_mod;
mod simulation_mod;
mod state_module;

use crate::math_module::ODE;
use crate::rocket_mod::Rocket;
use crate::simulation_mod::Simulation;
use crate::state_module::State;

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
    let v0: f64 = 100.0; // m/s
    let h0: f64 = 0.0; // m
    let state = State::new(h0, v0, test_rocket);

    // iteration/calculation Parameters
    const MAXITER: u64 = 1e5 as u64;
    const DT: f64 = 1e-3 as f64;
    const ITERMETHOD: i32 = 1;
    let euler_method = ODE::new(DT, ITERMETHOD);

    //Assemble Simulation Struct
    let mut case: Simulation = Simulation::new(state, euler_method, 1, MAXITER);

    case.run();
}


