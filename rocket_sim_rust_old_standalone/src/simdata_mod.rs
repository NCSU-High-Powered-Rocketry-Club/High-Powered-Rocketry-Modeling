use crate::state::state_vector::StateVector;

#[derive(Clone, Debug)]
pub(crate) struct SimulationData<const L: usize> {
    pub(crate) len: u64,
    time: Vec<f64>,
    data: Vec<[f64; L]>,
    index: usize,
    col: usize,
}

impl<const L: usize> SimulationData<L> {
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
    pub(crate) fn add_row(&mut self, row: StateVector, time: f64) -> () {
        self.len += 1; // Can maybe speed up by adding this at very end (simulation iter #)
        self.time.push(time);
        self.data
            .push(<[f64; L]>::try_from(row.as_array()).unwrap());
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
