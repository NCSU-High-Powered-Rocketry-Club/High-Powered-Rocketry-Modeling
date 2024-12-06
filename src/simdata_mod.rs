use std::ops::Deref;
use crate::math::vec_ops::{MathVector};
use crate::state_mod::State;


//pub(crate) trait SimulationData : Sized{
//    fn add_row(&mut self, row: Box<(dyn VectorOperations)>, time: f64) -> ();
//    fn set_col(&mut self, col: usize) -> ();
//    fn get_value(&self, index: usize) -> Option<f64>;
//}


#[derive(Clone, Debug)]
pub(crate) struct SimulationData<T>{
    len: u64,
    time: Vec<f64>,
    data: Vec<T>,
    index: usize,
    col: usize,
}

impl<const L: usize> SimulationData<[f64;L]> {
    const INITCAP: usize = 10000;
    pub(crate) fn new() -> Self {
        Self {
            len: 0,
            time: Vec::with_capacity(Self::INITCAP),
            data: Vec::with_capacity(Self::INITCAP),
            index: 0,
            col: 0,
        }
    }
    pub(crate) fn add_row(&mut self, row: MathVector<L>, time: f64) ->() {
        self.len += 1; // Can maybe speed up by adding this at very end (simulation iter #)
        self.time.push(time);
        self.data.push(row.data);
    }
    fn set_col(&mut self, col: usize) {
        self.col = col;
    }
    fn get_value(&self, index: usize) -> Option<f64> {
        if index >= self.len as usize {
            ()
        }
        if self.col == 0 {
            Some(self.time[index])
        } else {
            Some(self.data[index][self.col - 1])
        }
    }
}

/*
impl<const L: usize> Iterator for SimulationData<usize> {
    // A barebones implimintation of the Iterator trait for Simulation Data. Currently unused,
    // but could be useful in the future.
    type Item = f64;

    fn next(&mut self) -> Option<Self::Item> {
        match self {
            SimulationData::__1DOF(dof1, iter) => {
                *iter += 1;
                dof1.get_value(*iter)
            }
            SimulationData::__3DOF(dof3, iter) => {
                *iter += 1;
                dof3.get_value(*iter)
            }
        }
    }
}
 */