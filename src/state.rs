pub(crate) mod model_1dof;
pub(crate) mod model_3dof;
pub(crate) mod state_vector;

use nalgebra::{Vector2, Vector6};

use crate::state::model_1dof::Dof1;
use crate::state::model_3dof::Dof3;
use crate::state::state_vector::StateVector;
use crate::{ModelType, Rocket};

use std::f64::consts::PI;
use std::process::exit;

/// The internal simulation state, wrapping either a 1-DOF or 3-DOF model.
/// This matches the `ModelType` enum (Python API) but contains the *actual* model data.
#[derive(Debug, Clone, Copy)]
pub(crate) enum State {
    __1DOF(Dof1),
    __3DOF(Dof3),
}

impl State {
    /// Construct a `State` from ModelType + initial conditions + Rocket.
    /// This moves the logic out of `simulate_flight()`.
    pub(crate) fn from_model_type(
        model_type: ModelType,
        rocket: Rocket,
        initial_height: f64,
        initial_velocity: f64,
        initial_angle: Option<f64>,
    ) -> Self {
        match model_type {
            ModelType::OneDOF => {
                // u1 = [y, vy]
                let u1 = Vector2::new(initial_height, initial_velocity);
                State::__1DOF(Dof1::new(u1, rocket))
            }
            ModelType::ThreeDOF => {
                // u3 = [x, y, theta, vx, vy, omega]
                // Initial orientation = PI/2 (pointing up) if not provided
                let u3 = Vector6::new(
                    0.0,
                    initial_height,
                    initial_angle.unwrap_or(PI / 2.0),
                    0.0,
                    initial_velocity,
                    0.0,
                );
                State::__3DOF(Dof3::new(u3, rocket))
            }
        }
    }

    pub(crate) fn get_logrow(&self) -> StateVector {
        match self {
            State::__1DOF(dof1) => StateVector::__1DLOG(dof1.get_logrow()),
            State::__3DOF(dof3) => StateVector::__3DLOG(dof3.get_logrow()),
        }
    }

    pub(crate) fn print_state(&self, i: u64) {
        match self {
            State::__1DOF(dof1) => dof1.print_state_1dof(i),
            State::__3DOF(dof3) => dof3.print_state_3dof(i),
        }
    }

    pub(crate) fn get_state_vec(&self) -> StateVector {
        match self {
            State::__1DOF(dof1) => StateVector::__1DOF(dof1.u),
            State::__3DOF(dof3) => StateVector::__3DOF(dof3.u),
        }
    }

    pub(crate) fn get_altitude(&self) -> f64 {
        match self {
            State::__1DOF(dof1) => dof1.get_height(),
            State::__3DOF(dof3) => dof3.get_height(),
        }
    }

    pub(crate) fn get_vertical_velocity(&self) -> f64 {
        match self {
            State::__1DOF(dof1) => dof1.get_velocity(),
            State::__3DOF(dof3) => dof3.get_y_velocity(),
        }
    }

    pub(crate) fn get_time(&self) -> f64 {
        match self {
            State::__1DOF(dof1) => dof1.get_time_1dof(),
            State::__3DOF(dof3) => dof3.get_time_3dof(),
        }
    }

    pub(crate) fn get_derivs(&mut self) -> StateVector {
        match self {
            State::__1DOF(dof1) => StateVector::__1DOF(dof1.get_derivs_1dof()),
            State::__3DOF(dof3) => StateVector::__3DOF(dof3.get_derivs_3dof()),
        }
    }

    pub(crate) fn update(&mut self, du_vec: StateVector, dt: f64) {
        match (self, du_vec) {
            (State::__1DOF(dof1), StateVector::__1DOF(du)) => dof1.update_state(du, dt),
            (State::__3DOF(dof3), StateVector::__3DOF(du)) => dof3.update_state(du, dt),
            // This case should *never* happen because increment types match DOF models.
            _ => {
                println!("Invalid State/update combination");
            }
        }
    }
}
