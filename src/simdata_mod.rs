use crate::constants::simulation_constants::{DATA_LENGTH, INITIAL_DATA_CAPACITY};
use crate::state::state_vector::StateVector;

#[derive(Clone, Debug)]
pub(crate) struct SimulationData {
    pub(crate) time_log: Vec<f64>,
    pub(crate) state_log: Vec<[f64; DATA_LENGTH]>,
}

impl SimulationData {
    pub(crate) fn new() -> Self {
        Self {
            time_log: Vec::with_capacity(INITIAL_DATA_CAPACITY),
            state_log: Vec::with_capacity(INITIAL_DATA_CAPACITY),
        }
    }

    /// Retrieves the value at the specified row index and column index from the simulation data.
    /// Column index 0 corresponds to time, while column indices 1 to DATA_LENGTH correspond to the
    /// state vector components.
    pub(crate) fn get_val(&self, index: usize, col: usize) -> f64 {
        if index >= self.time_log.len() {
            panic!("Index out of bounds");
        }

        if col == 0 {
            self.time_log[index]
        } else {
            self.state_log[index][col - 1]
        }
    }

    /// Adds a new row of simulation data consisting of the given state vector and time value.
    pub(crate) fn add_row(&mut self, row: StateVector, time: f64) {
        self.time_log.push(time);

        let rowdata = row.as_array();

        // Creates an array filled with zeroes, then copies only the data from rowdata into it
        let mut padded_row = [0.0; DATA_LENGTH];
        let copy_len = rowdata.len();
        padded_row[..copy_len].copy_from_slice(&rowdata[..copy_len]);
        self.state_log.push(padded_row);
    }
}
