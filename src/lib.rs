mod math;
mod physics_mod;
mod simdata_mod;
mod simulation;
mod state;

use pyo3::exceptions::PyTypeError;
use pyo3::prelude::*;
use std::f64::consts::PI;
use std::io::BufRead;

use crate::math::ode::{AdaptiveTimeStep, OdeMethod};
use crate::simdata_mod::{SimulationData};
use crate::simulation::Simulation;
use crate::state::{model_1dof::Dof1, model_3dof::Dof3, PyState, State};

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
    fn __repr__(&self) -> String {
        format!(
            "Rocket(mass={}kg, Cd={}, DragArea={}m^2)",
            self.mass, self.cd, self.area_drag
        )
    }

    // 3. Full Simulation Method (Returns Data Object)
    fn simulate_flight(
        &self,
        height: f64,
        velocity: f64,
        model_type: ModelType,
        method: IntegrationMethod,
    ) -> PyResult<SimulationData> {
        // A. Create the simulation using helper
        let mut sim = self.create_simulation(height, velocity, model_type, method)?;

        // B. Run it and capture data
        let mut data = SimulationData::new();
        sim.run(&mut data);

        Ok(data)
    }

    // 4. Apogee Prediction Method (Returns Float)
    fn predict_apogee(
        &self,
        height: f64,
        velocity: f64,
        model_type: ModelType,
        method: IntegrationMethod,
    ) -> PyResult<f64> {
        // A. Create the simulation using helper
        let mut sim = self.create_simulation(height, velocity, model_type, method)?;

        // B. Run it (we create a dummy data container because the trait requires it)
        // Optimization: If performance is critical, we could make a 'run_silent' method later
        let mut data = SimulationData::new();
        sim.run(&mut data);

        // C. Return just the scalar value
        Ok(sim.apogee())
    }
    
}

//
impl Rocket {
    /// Internal helper to build the complex Simulation struct from simple Python inputs
    fn create_simulation(
        &self,
        h: f64,
        v: f64,
        model_type: ModelType,
        method: IntegrationMethod,
    ) -> PyResult<Simulation> {
        
        // Step 1: Configure Solver based on Enum
        let solver = match method {
            IntegrationMethod::Euler => OdeMethod::Euler(1e-2),
            IntegrationMethod::RK45 => {
                let mut ats = AdaptiveTimeStep::new();
                // Hardcoded sensible defaults for Python users
                ats.absolute_error_tolerance = 1e-5;
                ats.relative_error_tolerance = 1e-5;
                OdeMethod::RK45(ats)
            }
        };

        // Step 2: Configure State based on Enum
        let state = match model_type {
            ModelType::OneDOF => {
                // 1DOF Vector: [Height, Velocity]
                let u = vec![h, v];
                State::__1DOF(Dof1::new(u, self.clone()))
            },
            ModelType::ThreeDOF => {
                // 3DOF Vector: [x, y, theta, vx, vy, omega]
                // We assume vertical launch (theta = PI/2)
                let u = vec![0.0, h, PI / 2.0, 0.0, v, 0.0];
                State::__3DOF(Dof3::new(u, self.clone()))
            }
        };

        // Step 3: Build Simulation
        // We hardcode max iterations to prevent infinite loops
        const MAXITER: u64 = 200_000;
        
        Ok(Simulation::new(state, solver, 1, MAXITER))
    }
}

#[pymodule]
fn hprm(m: &Bound<'_, PyModule>) -> PyResult<()> {
    m.add_class::<Rocket>()?;
    m.add_class::<SimulationData>()?; 
    m.add_class::<ModelType>()?;
    m.add_class::<IntegrationMethod>()?;
    Ok(())
}