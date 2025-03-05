use crate::state::state_vector::StateVector;
use pyo3::prelude::*;
use crate::state::model_1dof::Dof1;
use crate::state::model_3dof::Dof3;

#[derive(Clone, Debug)]
pub(crate) struct SimulationData<const L: usize> {
    pub(crate) len: u64,
    time: Vec<f64>,
    data: Vec<[f64; L]>,
    index: usize,
    col: usize,
}

/*
#[pyclass(dict, get_all, set_all)]
#[derive(Clone)]
pub(crate) struct PySimData1d {
    pub(crate) data: SimulationData<{ Dof1::NLOG }>,
}
#[pymethods]
impl PySimData1d{
    #[new]
    pub(crate) fn new() -> Self {
        Self {
            data: SimulationData::new()
        }
    }
    fn __repr__(&self) -> String {
        "Python Struct for Storing Sim Data".to_string()
    }
    fn __str__(&self) -> String {
        "Python Struct for Storing Sim Data".to_string()
    }
}


#[pyclass(dict, get_all, set_all)]
#[derive(Clone)]
pub(crate) struct PySimData3d {
    pub(crate) data: SimulationData<{ Dof3::NLOG }>,
}
#[pymethods]
impl PySimData3d{
    #[new]
    pub(crate) fn new() -> Self {
        Self {
            data: SimulationData::new()
        }
    }
    fn __repr__(&self) -> String {
        "Python Struct for Storing Sim Data".to_string()
    }
    fn __str__(&self) -> String {
        "Python Struct for Storing Sim Data".to_string()
    }
}
 */

impl<const L: usize> SimulationData<L> {
    const INITCAP: usize = 1000;
    pub(crate) fn new() -> Self {
        Self {
            len: 0,
            time: Vec::with_capacity(Self::INITCAP),
            data: Vec::with_capacity(Self::INITCAP),
            index: 0,
            col: 0,
        }
    }
    pub(crate) fn add_row(&mut self, row: StateVector, time: f64) -> () {
        self.len += 1; // Can maybe speed up by adding this at very end (simulation iter #)
        self.time.push(time);
        let rowdata = row.as_array();
        let mut rowvec = rowdata.to_vec();
        if (rowdata.len() < L) {
            while rowvec.len() < L {
                rowvec.push(0.0);
            }
        }
        self.data
            .push(<[f64; L]>::try_from(rowvec).unwrap());
    }
    pub(crate) fn get_val(&self, index: usize, col: usize) -> f64 {
        if index >= self.len as usize {
            ()
        }
        if col == 0 {
            self.time[index]
        } else {
            self.data[index][col - 1]
        }
    }
}
