pub(crate) mod model_1dof;
pub(crate) mod model_3dof;
pub(crate) mod state_vector;

use crate::math::Norm;
use crate::state::model_1dof::Dof1;
use crate::state::model_3dof::Dof3;
use crate::simdata_mod::{SimulationData};
use state_vector::StateVector;

use std::process::exit;
use pyo3::prelude::*;
use nalgebra::{Vector2, Vector3, Vector6, SVector};

#[pyclass(dict, get_all, set_all)]
pub(crate) struct PyState {
    pub ndof: i32,
    pub nlog: usize,
    pub u1: [f64; 2],
    pub u3: [f64; 6],
    pub u6: [f64; 12],
}
#[pymethods]
impl PyState {
    fn __repr__(&self) -> String {
        "Python Interface for State enum".to_string()
    }
    fn __str__(&self) -> String {
        "Python Interface for State enum".to_string()
    }
    #[new]
    pub(crate) fn new(ndof: i32) -> Self {
        let nlog = match ndof{
            1 => Dof1::NLOG,
            3 => Dof3::NLOG,
            _ => {0_usize}
        };
        Self {
            ndof,
            nlog,
            u1: [0.0;2],
            u3: [0.0;6],
            u6: [0.0;12],
        }
    }
    pub(crate) fn set_new_model(&mut self, new_ndof: i32) -> () {
        self.nlog = match new_ndof{
            1 => Dof1::NLOG,
            3 => Dof3::NLOG,
            _ => {0_usize}
        };
        self.ndof = new_ndof;
    }
}

impl PyState{
    pub(crate) fn u1_as_vector(&self) -> Vector2<f64> {
            Vector2::<f64>::from_vec(self.u1.to_vec())
    }
    pub(crate) fn u3_as_vector(&self) -> Vector6<f64> {
            Vector6::<f64>::from_vec(self.u3.to_vec())
    }
}

#[derive(Debug, Clone, Copy)]
pub(crate) enum State {
    // Enumeration to unify the different available state spaces / ODE models
    // Each of these specifies what the simulation is really all about: what equations you are
    //         actually solving, what data types, and the number of variables that are needed,...
    __1DOF(Dof1),
    __3DOF(Dof3),
    //__6DOF(Dof6),
}

impl State {
    pub(crate) fn get_logrow(&self) -> StateVector {
        match self {
            // Custom printout to let the user know the status of the state during iterations
            State::__1DOF(dof1) => StateVector::__1DLOG(dof1.get_logrow()),
            State::__3DOF(dof3) => StateVector::__3DLOG(dof3.get_logrow()),
            _ => panic!("Not a valid State, logrow"),
        }
    }
    pub(crate) fn print_state(&self, i: u64) {
        match self {
            // Custom printout to let the user know the status of the state during iterations
            State::__1DOF(dof1) => dof1.print_state_1dof(i),
            State::__3DOF(dof3) => dof3.print_state_3dof(i),
            _ => println!("Ignoring, State:print_state"),
        }
    }
    pub(crate) fn _get_state_vec(&self) -> StateVector {
        // Return the current values of the state variables using that StateVector enum
        match self {
            State::__1DOF(dof1) => StateVector::__1DOF(dof1.u),
            State::__3DOF(dof3) => StateVector::__3DOF(dof3.u),
            _ => panic!("Not a valid State, get_state_vec"),
        }
    }
    pub(crate) fn get_altitude(&self) -> f64 {
        // get the current elevation/height
        match self {
            State::__1DOF(dof1) => dof1.get_height(),
            State::__3DOF(dof3) => dof3.get_height(),
            _ => {
                println!("Ignoring, State:get_altitude");
                f64::NAN
            }
        }
    }
    pub(crate) fn get_vertical_velocity(&self) -> f64 {
        // get the velocity in the vertical direction
        match self {
            State::__1DOF(dof1) => dof1.get_velocity(),
            State::__3DOF(dof3) => dof3.get_y_velocity(),
            _ => {
                println!("Ignoring Invalid State, State:get_vertical_velocity");
                f64::NAN
            }
        }
    }
    pub(crate) fn get_time(&self) -> f64 {
        // return the value of the time variable
        match self {
            State::__1DOF(dof1) => dof1.get_time_1dof(),
            State::__3DOF(dof3) => dof3.get_time_3dof(),
            _ => {
                println!("Ignoring Invalid State, State:get_altitude");
                f64::NAN
            }
        }
    }
    pub(crate) fn get_derivs(&mut self) -> StateVector {
        // Return a vector which contains the derivatives of the model/state variables
        match self {
            State::__1DOF(dof1) => StateVector::__1DOF(dof1.get_derivs_1dof()),
            State::__3DOF(dof3) => StateVector::__3DOF(dof3.get_derivs_3dof()),
            _ => {
                println!("Unrecoverable Invalid State, State:get_derivs"); //This could be improved w/ option
                exit(0)
            }
        }
    }
    pub(crate) fn update(&mut self, du_vec: StateVector, dt: f64) -> () {
        // Used by the math module to modify the value of the current state once the timestep has
        //  been calculated by the OdeIterator
        match (self, du_vec) {
            (State::__1DOF(dof1), StateVector::__1DOF(du)) => dof1.update_state(du, dt),
            (State::__3DOF(dof3), StateVector::__3DOF(du)) => dof3.update_state(du, dt),
            _ => {
                println!("Invalid State, State:update");
            }
        }
    }
}
