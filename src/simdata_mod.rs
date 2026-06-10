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

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_abs_diff_eq;
    use nalgebra::{Vector2, Vector6};

    #[test]
    fn test_simulation_data_initialization() {
        let data = SimulationData::new();
        assert!(data.time_log.is_empty());
        assert!(data.state_log.is_empty());
        assert!(data.time_log.capacity() >= INITIAL_DATA_CAPACITY);
        assert!(data.state_log.capacity() >= INITIAL_DATA_CAPACITY);
    }

    #[test]
    fn test_for_one_dof() {
        let mut data = SimulationData::new();

        let altitude = 500.0;
        let velocity = 300.0;
        let state = StateVector::OneDOF(Vector2::new(altitude, velocity));
        let time = 12.34;

        data.add_row(state, time);

        // Column 0 must return the recorded timestamp
        assert_abs_diff_eq!(data.get_val(0, 0), time, epsilon = 1e-12);

        // Column 1 and 2 must match altitude and velocity exactly
        assert_abs_diff_eq!(data.get_val(0, 1), altitude, epsilon = 1e-12);
        assert_abs_diff_eq!(data.get_val(0, 2), velocity, epsilon = 1e-12);

        // Columns past index 2 up to DATA_LENGTH must be padded with pure zeroes
        for col in 3..=DATA_LENGTH {
            assert_abs_diff_eq!(data.get_val(0, col), 0.0, epsilon = 1e-12);
        }
    }

    #[test]
    fn test_for_three_dof() {
        let mut data = SimulationData::new();

        // Define a concrete 3DOF state vector: [x, y, angle, vx, vy, angular_rate]
        let elements = [10.0, 20.0, 0.785, 100.0, 200.0, 0.1];
        let state = StateVector::ThreeDOF(Vector6::from_column_slice(&elements));
        let time = 56.78;

        data.add_row(state, time);

        // Column 0 must return the timestamp
        assert_abs_diff_eq!(data.get_val(0, 0), time, epsilon = 1e-12);

        // Columns 1 through 6 map directly to the 3DOF array elements
        for i in 0..6 {
            assert_abs_diff_eq!(data.get_val(0, i + 1), elements[i], epsilon = 1e-12);
        }

        // Check zero padding for any trailing space left over up to DATA_LENGTH
        if DATA_LENGTH > 6 {
            for col in 7..=DATA_LENGTH {
                assert_abs_diff_eq!(data.get_val(0, col), 0.0, epsilon = 1e-12);
            }
        }
    }
}
