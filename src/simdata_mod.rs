use crate::state::state_vector::StateVector;
use numpy::{PyArray1, PyArray2, ToPyArray};
use pyo3::prelude::*;
//
const L: usize = 18;
#[pyclass(dict, get_all, set_all)]
#[derive(Clone, Debug)]
pub(crate) struct SimulationData {
    pub(crate) len: u64,
    time: Vec<f64>,
    data: Vec<[f64; L]>,
    index: usize,
    col: usize,
}
#[pymethods]
impl SimulationData {
    const INITCAP: usize = 1000;
    #[new]
    pub(crate) fn new() -> Self {
        Self {
            len: 0,
            time: Vec::with_capacity(Self::INITCAP),
            data: Vec::with_capacity(Self::INITCAP),
            index: 0,
            col: 0,
        }
    }
    //
    //
    pub(crate) fn get_val(&self, index: usize, col: usize) -> f64 {
        index >= self.len as usize;
        if col == 0 {
            self.time[index]
        } else {
            self.data[index][col - 1]
        }
    }
    //
    //
    pub(crate) fn get_len(&self) -> usize {
        self.time.len()
    }
    //
    //
    //pub(crate) fn get_as_numpy_array(&self, py: Python) -> (Py<PyArray1<f64>> , Py<PyArray2<f64>>) {
    //    (self.time.to_pyarray(py).into(), self.data.to_pyarray(py).into())
    //}
}

impl SimulationData {
    pub(crate) fn add_row(&mut self, row: StateVector, time: f64) {
        self.len += 1; // Can maybe speed up by adding this at very end (simulation iter #)
        self.time.push(time);
        let rowdata = row.as_array();
        let mut rowvec = rowdata.to_vec();
        if rowdata.len() < L {
            while rowvec.len() < L {
                rowvec.push(0.0);
            }
        }
        self.data.push(<[f64; L]>::try_from(rowvec).unwrap());
    }
}
