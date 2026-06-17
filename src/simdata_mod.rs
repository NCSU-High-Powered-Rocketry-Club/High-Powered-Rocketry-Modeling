use crate::constants::simulation_constants::{DATA_LENGTH, INITIAL_DATA_CAPACITY};
use crate::state::state_vector::StateVector;

#[derive(Clone, Debug)]
pub(crate) struct SimulationData {
    pub(crate) len: u64,
    pub(crate) time_log: Vec<f64>,
    pub(crate) state_log: Vec<[f64; DATA_LENGTH]>,
}

impl SimulationData {
    pub(crate) fn new() -> Self {
        Self {
            len: 0,
            time_log: Vec::with_capacity(INITIAL_DATA_CAPACITY),
            state_log: Vec::with_capacity(INITIAL_DATA_CAPACITY),
        }
    }

    pub(crate) fn get_val(&self, index: usize, col: usize) -> f64 {
        if index >= self.len as usize {
            panic!("Index out of bounds");
        }

        if col == 0 {
            self.time_log[index]
        } else {
            self.state_log[index][col - 1]
        }
    }
}

impl SimulationData {
    pub(crate) fn add_row(&mut self, row: StateVector, time: f64) {
        self.len += 1; // Can maybe speed up by adding this at very end (simulation iter #)
        self.time_log.push(time);
        let rowdata = row.as_array();
        let mut rowvec = rowdata.to_vec();
        if rowdata.len() < DATA_LENGTH {
            while rowvec.len() < DATA_LENGTH {
                rowvec.push(0.0);
            }
        }
        self.state_log
            .push(<[f64; DATA_LENGTH]>::try_from(rowvec).unwrap());
    }
}
