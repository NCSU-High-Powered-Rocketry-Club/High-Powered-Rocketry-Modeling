pub(crate) mod state_vector;
pub(crate) mod model_1dof;
pub(crate) mod model_3dof;

use crate::math::vec_ops::VectorOperations;
use crate::math::Norm;
use state_vector::StateVector;
use std::ops::{Add, AddAssign, Mul, MulAssign};
use std::process::exit;
use crate::state::model_1dof::Dof1;
use crate::state::model_3dof::Dof3;

#[derive(Debug, Clone, Copy)]
pub(crate) enum State {
    // Enumeration to unify the different available state spaces / ODE models
    // Each of these specifies what the simulation is really all about: what equations you are
    //         actually solving, what data types, and the number of variables that are needed,...
    __1DOF(Dof1),
    __3DOF(Dof3),
}

impl State {
    pub(crate) fn get_logrow(&self) -> StateVector {
        match self {
            // Custom printout to let the user know the status of the state during iterations
            State::__1DOF(dof1) => StateVector::__1DLOG(dof1.get_logrow()),
            State::__3DOF(dof3) => StateVector::__3DLOG(dof3.get_logrow()),
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
    pub(crate) fn get_state_vec(&self) -> StateVector {
        // Return the current values of the state variables using that StateVector enum
        match self {
            State::__1DOF(dof1) => StateVector::__1DOF(dof1.u),
            State::__3DOF(dof3) => StateVector::__3DOF(dof3.u),
        }
    }
    pub(crate) fn get_ndim(&self) -> usize {
        // Get number of dimensions
        match self {
            State::__1DOF(_dof1) => 2usize,
            State::__3DOF(_dof3) => 6usize,
            _ => {
                println!("Ignoring, State:get_ndim");
                0usize
            }
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
