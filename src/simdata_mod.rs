use crate::constants::simulation_constants::{DATA_LENGTH, INITIAL_DATA_CAPACITY};
use crate::state::state_vector::StateVector;
use numpy::{PyArray1, PyArray2, ToPyArray};
use pyo3::prelude::*;

#[pyclass(dict, get_all, set_all)]
#[derive(Clone, Debug)]
pub(crate) struct SimulationData {
    pub(crate) len: u64,
    time: Vec<f64>,
    data: Vec<[f64; DATA_LENGTH]>,
    index: usize,
    col: usize,
}

#[pymethods]
impl SimulationData {
    #[new]
    pub(crate) fn new() -> Self {
        Self {
            len: 0,
            time: Vec::with_capacity(INITIAL_DATA_CAPACITY),
            data: Vec::with_capacity(INITIAL_DATA_CAPACITY),
            index: 0,
            col: 0,
        }
    }

    pub(crate) fn get_val(&self, index: usize, col: usize) -> f64 {
        if index >= self.len as usize {
            panic!("Index out of bounds");
        }

        if col == 0 {
            self.time[index]
        } else {
            self.data[index][col - 1]
        }
    }

    pub(crate) fn get_len(&self) -> usize {
        self.time.len()
    }
}

impl SimulationData {
    pub(crate) fn add_row(&mut self, row: StateVector, time: f64) {
        self.len += 1; // Can maybe speed up by adding this at very end (simulation iter #)
        self.time.push(time);
        let rowdata = row.as_array();
        let mut rowvec = rowdata.to_vec();
        if rowdata.len() < DATA_LENGTH {
            while rowvec.len() < DATA_LENGTH {
                rowvec.push(0.0);
            }
        }
        self.data
            .push(<[f64; DATA_LENGTH]>::try_from(rowvec).unwrap());
    }
}
