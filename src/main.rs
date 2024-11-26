mod math_mod;
mod physics_mod;
mod rocket_mod;
mod simdata_mod;
mod simulation_mod;
mod state_mod;

use std::io::BufRead;
use plotters::prelude::*;

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

fn main() -> Result<(), Box<dyn std::error::Error>> {
    // Rocket Parameters
    let mass: f64 = 10.0; //kg
    let cd: f64 = 0.3;
    let area: f64 = 0.005; // m^2
    let inertia: f64 = 10.0;
    let stab_mgn: f64 = 0.3;
    let test_rocket: Rocket = Rocket::new(mass, cd, area, inertia, stab_mgn);

    // Initial Conditions
    let u0: [f64; 2] = [0.0, 100.0]; // m, m/s
    let state_euler = State::__1DOF(Dof1::new(u0, test_rocket));

    let u0: [f64; 6] = [0.0, 0.0, 0.0, 0.0, 100.0, 0.0]; // m, m, rad, m/s, m/s, rad/s
    let state_rk3 = State::__3DOF(Dof3::new(u0, test_rocket));

    // iteration/calculation Parameters
    const MAXITER: u64 = 1e5 as u64;        //Maximum number of iterations before stopping calculation
    const DT: f64 = 1e-1 as f64;            //Timestep size to use when integrating ODE
    let euler_method = OdeIterators::Euler(DT);
    let rk3 = OdeIterators::RK3(DT);

    //Assemble Simulation Struct
    let mut case_euler: Simulation = Simulation::new(state_euler.clone(), euler_method, 1, MAXITER);
    let mut case_rk3: Simulation = Simulation::new(state_rk3.clone(), rk3, 1, MAXITER);

    case_euler.run();
    case_rk3.run();

    println!(
        "Euler: Apogee {:6.2}\nRK3  : Apogee {:6.2}\n",
        case_euler.apogee(),
        case_rk3.apogee()
    );
    println!("Try different timestep sizes and see how the different methods' accuracy behaves!");


    // ========== Plotting Results (will be cleaned up in the future)
    let file_name = "test.png";

    let xmin = 8f32;
    let xmax = 10f32;
    let ymin = 470f32;
    let ymax = 495f32;

    let plot_title = "Test Rocket Flight";
    let y_label = "Altitude (m)";
    let x_label = "Time (s)";

    let series_1_name = "Euler's Method";
    let series_2_name = "Runge-Kutta 3-stage Method";



    let root = BitMapBackend::new(file_name, (640, 480)).into_drawing_area();
    root.fill(&WHITE)?;
    let mut chart = ChartBuilder::on(&root)
        .caption(plot_title, ("sans-serif", 50).into_font())
        .margin(10)
        .x_label_area_size(40)
        .y_label_area_size(70)
        .build_cartesian_2d(xmin..xmax, ymin..ymax)?;

    chart.configure_mesh()
        .x_desc(x_label)
        .y_desc(y_label)
        .axis_desc_style(("sans-serif", 20))
        .draw()?;


    // ########## The number in ~~~.col( x ).~~~~ is what determines which variable we are
    // ########## looking at. Since this is state-dependant, I think it would be nice to get a
    // ########## string or otherwise more general way of specifying that. But this works for now.
    chart
        .draw_series(LineSeries::new(
            (0..case_euler.iter).map(
                |ind| (case_euler.data.col(0).get_val(ind as usize)as f32,
                            case_euler.data.col(1).get_val(ind as usize) as f32)
            ),
            RED.stroke_width(2),
        ))?
        .label(series_1_name)
        .legend(|(x, y)| PathElement::new(vec![(x, y), (x + 20, y)], &RED));
    chart
        .draw_series(LineSeries::new(
            (0..case_rk3.iter).map(
                |ind| (case_rk3.data.col(0).get_val(ind as usize)as f32,
                            case_rk3.data.col(2).get_val(ind as usize) as f32)
            ),
            BLUE.stroke_width(2),
        ))?
        .label(series_2_name)
        .legend(|(x, y)| PathElement::new(vec![(x, y), (x + 20, y)], &BLUE));

    chart
        .configure_series_labels()
        .background_style(&WHITE.mix(0.8))
        .border_style(&BLACK)
        .draw()?;

    root.present()?;

    Ok(())
}