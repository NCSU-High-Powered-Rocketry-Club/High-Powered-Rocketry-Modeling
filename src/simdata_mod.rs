use crate::state_mod::{State, StateVector};
#[derive(Debug, Clone)]
pub(crate) enum SimulationData {
    //Enumeration to unify the different types of simulation data, each corresponding to a certain
    // state space / model.
    __1DOF(SimData1Dof, usize),
    __3DOF(SimData3Dof, usize),
}
impl SimulationData {
    pub(crate) fn new(&state: &State) -> Self {
        match state {
            State::__1DOF(_dof1) => SimulationData::__1DOF(SimData1Dof::new(), 0),
            State::__3DOF(_dof3) => SimulationData::__3DOF(SimData3Dof::new(), 0),
        }
    }
    pub(crate) fn add_row(&mut self, new_data_vec: (StateVector, StateVector), time: f64) {
        // This function is used for adding entries to the data each iteration
        //new_data_vec = (u, dudt) = (state variables, state derivatives)
        match (self, new_data_vec.0, new_data_vec.1) {
            (SimulationData::__1DOF(sim_data, _),
                StateVector::__1DOF(new_u),
                StateVector::__1DOF(new_du)) => sim_data.add_row(new_u, new_du, time),
            (SimulationData::__3DOF(sim_data, _),
                StateVector::__3DOF(new_u),
                StateVector::__3DOF(new_du)) => sim_data.add_row(new_u, new_du, time),
            _ => panic!("Incorrect combination of Simulation Data and State vector!!! (SimulationData::add_row)"),
        }
    }
    pub(crate) fn col(&mut self, col: usize) -> &mut Self {
        // A setter which is used to specify which field of data the user wants to retrieve
        match self {
            SimulationData::__1DOF(dof1, _) => dof1.set_col(col),
            SimulationData::__3DOF(dof3, _) => dof3.set_col(col),
        }
        self
    }
    pub(crate) fn get_val(&mut self, ind: usize) -> f64 {
        // A getter which returns the value at the given index
        //      (and in whatever col the data object has been set to)
        match self {
            SimulationData::__1DOF(dof1, _) => dof1.get_value(ind).unwrap_or_else(|| {f64::NAN}),
            SimulationData::__3DOF(dof3, _) => dof3.get_value(ind).unwrap_or_else(|| {f64::NAN}),
            _ => (f64::NAN),
        }
    }
}
impl Iterator for SimulationData {
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
//##################################################################################################
//##################################################################################################
//##################################################################################################
#[derive(Debug, Clone)]
pub(crate) struct SimData1Dof {
    len: u64,
    time: Vec<f64>,
    data: Vec<[f64; 3]>,
    //Variables for iterator
    index: usize,
    col: usize,
}

impl SimData1Dof {
    const INITCAP: usize = 10000;
    fn new() -> Self {
        Self {
            len: 0,
            time: Vec::with_capacity(Self::INITCAP),
            data: Vec::with_capacity(Self::INITCAP),
            index: 0,
            col: 0,
        }
    }
    fn add_row(&mut self, new_u: [f64; 2], new_du: [f64; 2], time: f64) {
        self.len += 1; // Can maybe speed up by adding this at very end (simulation iter #)
        self.time.push(time);
        self.data.push([new_u[0], new_u[1], new_du[1]]);
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
    fn set_col(&mut self, col: usize) {
        self.col = col;
    }
}
//##################################################################################################
//##################################################################################################
//##################################################################################################
#[derive(Debug, Clone)]
pub(crate) struct SimData3Dof {
    len: u64,
    time: Vec<f64>,
    data: Vec<[f64; 9]>,
    index: usize,
    col: usize,
}

impl SimData3Dof {
    const INITCAP: usize = 10000;
    fn new() -> Self {
        Self {
            len: 0,
            time: Vec::with_capacity(Self::INITCAP),
            data: Vec::with_capacity(Self::INITCAP),
            index: 0,
            col: 0,
        }
    }
    fn add_row(&mut self, new_u: [f64; 6], new_du: [f64; 6], time: f64) {
        self.len += 1; // Can maybe speed up by adding this at very end (simulation iter #)
        self.time.push(time);
        self.data.push([
            new_u[0], new_u[1], new_u[2], new_u[3], new_u[4], new_u[5], new_du[3], new_du[4],
            new_du[5],
        ]);
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
    fn set_col(&mut self, col: usize) {
        self.col = col;
    }
}
