pub(crate) mod model_1dof;
pub(crate) mod model_3dof;
pub(crate) mod state_vector;

use nalgebra::{Vector2, Vector6};

use crate::rocket::Rocket;
use crate::state::model_1dof::OneDOFModel;
use crate::state::model_3dof::ThreeDOFModel;
use crate::state::state_vector::StateVector;

use std::f64::consts::PI;
use std::process::exit;

/// The internal simulation state, wrapping either a 1-DOF or 3-DOF model.
#[derive(Debug, Clone, Copy)]
pub(crate) enum State {
    OneDOF(OneDOFModel),
    ThreeDOF(ThreeDOFModel),
}

impl State {
    /// Makes a new state for the 1-DOF model, given the rocket parameters and initial conditions.
    pub(crate) fn new_1dof(
        rocket: Rocket,
        initial_height: f64,
        initial_velocity: f64,
    ) -> Self {
        let u1 = Vector2::new(initial_height, initial_velocity);
        State::OneDOF(OneDOFModel::new(u1, rocket))
    }

    /// Makes a new state for the 3-DOF model, given the rocket parameters and initial conditions.
    pub(crate) fn new_3dof(
        rocket: Rocket,
        initial_height: f64,
        initial_velocity: f64,
        initial_angle: f64,
    ) -> Self {
        // u3 = [x, y, theta, vx, vy, omega]
        // PI/2 means pointing up
        let u3 = Vector6::new(
            0.0,
            initial_height,
            initial_angle,
            0.0,
            initial_velocity,
            0.0,
        );
        State::ThreeDOF(ThreeDOFModel::new(u3, rocket))
    }

    pub(crate) fn get_logrow(&self) -> StateVector {
        match self {
            State::OneDOF(dof1) => StateVector::__1DLOG(dof1.get_logrow()),
            State::ThreeDOF(dof3) => StateVector::__3DLOG(dof3.get_logrow()),
        }
    }

    pub(crate) fn print_state(&self, i: u64) {
        match self {
            State::OneDOF(dof1) => dof1.print_state_1dof(i),
            State::ThreeDOF(dof3) => dof3.print_state_3dof(i),
        }
    }

    #[allow(dead_code)]
    pub(crate) fn get_state_vec(&self) -> StateVector {
        match self {
            State::OneDOF(dof1) => StateVector::__1DOF(dof1.u),
            State::ThreeDOF(dof3) => StateVector::__3DOF(dof3.u),
        }
    }

    #[allow(dead_code)]
    pub(crate) fn get_altitude(&self) -> f64 {
        match self {
            State::OneDOF(dof1) => dof1.get_height(),
            State::ThreeDOF(dof3) => dof3.get_height(),
        }
    }

    pub(crate) fn get_vertical_velocity(&self) -> f64 {
        match self {
            State::OneDOF(dof1) => dof1.get_velocity(),
            State::ThreeDOF(dof3) => dof3.get_y_velocity(),
        }
    }

    pub(crate) fn get_time(&self) -> f64 {
        match self {
            State::OneDOF(dof1) => dof1.get_time_1dof(),
            State::ThreeDOF(dof3) => dof3.get_time_3dof(),
        }
    }

    pub(crate) fn get_derivs(&mut self) -> StateVector {
        match self {
            State::OneDOF(dof1) => StateVector::__1DOF(dof1.get_derivs_1dof()),
            State::ThreeDOF(dof3) => StateVector::__3DOF(dof3.get_derivs_3dof()),
        }
    }

    pub(crate) fn update(&mut self, du_vec: StateVector, dt: f64) {
        match (self, du_vec) {
            (State::OneDOF(dof1), StateVector::__1DOF(du)) => dof1.update_state(du, dt),
            (State::ThreeDOF(dof3), StateVector::__3DOF(du)) => dof3.update_state(du, dt),
            // This case should *never* happen because increment types match DOF models.
            _ => {
                println!("Invalid State/update combination");
            }
        }
    }
}
