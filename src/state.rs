pub(crate) mod model_1dof;
pub(crate) mod model_3dof;
pub(crate) mod state_vector;

use nalgebra::{Vector2, Vector6};

use crate::rocket::{Rocket, RocketProperties};
use crate::state::model_1dof::OneDOFModel;
use crate::state::model_3dof::ThreeDOFModel;
use crate::state::state_vector::StateVector;

/// The internal simulation state, wrapping either a 1-DOF or 3-DOF model. These models are what
/// contain the actual state information. The State struct provides a common interface for the ODE
/// solver to interact with, while the underlying models handle the specific details of the state.
///
/// Whenever the ODE solver needs to get information about the state or needs to do an operation on the
/// state, it will call the appropriate method on the State struct, such as `get_derivatives()`, which
/// will get the derivatives of the model represented as a `StateVector`. Then with this `StateVector`,
/// the ODE solver can perform its calculations and then call `update()` on the State struct to update
/// the state with the new values.
#[derive(Debug, Clone, Copy)]
pub(crate) enum State {
    OneDOF(OneDOFModel),
    ThreeDOF(ThreeDOFModel),
}

impl State {
    /// Makes a new state for the 1-DOF model, given the rocket parameters and initial conditions.
    pub(crate) fn new_1dof(
        rocket_properties: RocketProperties,
        initial_height: f64,
        initial_velocity: f64,
    ) -> Self {
        let u1 = Vector2::new(initial_height, initial_velocity);
        State::OneDOF(OneDOFModel::new(u1, rocket_properties))
    }

    /// Makes a new state for the 3-DOF model, given the rocket parameters and initial conditions.
    pub(crate) fn new_3dof(
        rocket_properties: RocketProperties,
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
        State::ThreeDOF(ThreeDOFModel::new(u3, rocket_properties))
    }

    /// Gets the current state vector with the additional log information (e.g. acceleration)
    /// included, which is used for logging the simulation data.
    pub(crate) fn get_row_log(&self) -> StateVector {
        match self {
            State::OneDOF(dof1) => StateVector::OneDOFLog(dof1.get_row_log()),
            State::ThreeDOF(dof3) => StateVector::ThreeDOFLog(dof3.get_row_log()),
        }
    }

    /// Prints the current state to the console.
    pub(crate) fn print_state(&self, i: u64) {
        match self {
            State::OneDOF(dof1) => dof1.print_state(i),
            State::ThreeDOF(dof3) => dof3.print_state(i),
        }
    }

    /// Gets the state represented as a `StateVector`.
    #[allow(dead_code)]
    pub(crate) fn get_state_vec(&self) -> StateVector {
        match self {
            State::OneDOF(dof1) => StateVector::OneDOF(dof1.u),
            State::ThreeDOF(dof3) => StateVector::ThreeDOF(dof3.u),
        }
    }

    /// Gets the altitude of the rocket from the state.
    #[allow(dead_code)]
    pub(crate) fn get_altitude(&self) -> f64 {
        match self {
            State::OneDOF(dof1) => dof1.get_height(),
            State::ThreeDOF(dof3) => dof3.get_height(),
        }
    }

    /// Gets the vertical velocity of the rocket from the state.
    pub(crate) fn get_vertical_velocity(&self) -> f64 {
        match self {
            State::OneDOF(dof1) => dof1.get_velocity(),
            State::ThreeDOF(dof3) => dof3.get_y_velocity(),
        }
    }

    /// Gets the time value from the state.
    pub(crate) fn get_time(&self) -> f64 {
        match self {
            State::OneDOF(dof1) => dof1.get_time(),
            State::ThreeDOF(dof3) => dof3.get_time(),
        }
    }

    /// Gets the derivatives of the state, represented as a `StateVector`. This is
    /// used by the ODE solver to perform its calculations.
    pub(crate) fn get_derivatives(&mut self) -> StateVector {
        match self {
            State::OneDOF(dof1) => StateVector::OneDOF(dof1.get_derivatives()),
            State::ThreeDOF(dof3) => StateVector::ThreeDOF(dof3.get_derivatives()),
        }
    }

    /// Updates the state with the given derivatives and timestep. This is used by the ODE solver
    /// to update the state after performing its calculations/iterations.
    pub(crate) fn update(&mut self, du_vec: StateVector, dt: f64) {
        match (self, du_vec) {
            (State::OneDOF(dof1), StateVector::OneDOF(du)) => dof1.update_state(du, dt),
            (State::ThreeDOF(dof3), StateVector::ThreeDOF(du)) => dof3.update_state(du, dt),
            // This case should *never* happen because increment types match DOF models.
            _ => {
                unreachable!("Invalid State/update combination");
            }
        }
    }
}
