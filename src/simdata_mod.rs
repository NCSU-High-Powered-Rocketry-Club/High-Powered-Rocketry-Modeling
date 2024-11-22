use crate::state_mod::{State, StateVector};

pub(crate) enum SimulationData {
    __1DOF(SimData1Dof),
    __3DOF(SimData3Dof),
}
impl SimulationData {
    pub(crate) fn new(&state: &State) -> Self {
        match state { 
            State::__1DOF(_dof1) => SimulationData::__1DOF(SimData1Dof::new()),
            State::__3DOF(_dof3) => SimulationData::__3DOF(SimData3Dof::new()),
        }
    }
    pub(crate) fn add_row(&mut self, new_data_vec: (StateVector,StateVector), time: f64) {
        //new_data_vec = (u, dudt) = (state variables, state derivatives)
        match (self, new_data_vec.0, new_data_vec.1) {
            (SimulationData::__1DOF(sim_data),
                StateVector::__1DOF(new_u),
                StateVector::__1DOF(new_du)) => sim_data.add_row(new_u, new_du, time),
            (SimulationData::__3DOF(sim_data),
                StateVector::__3DOF(new_u),
                StateVector::__3DOF(new_du)) => sim_data.add_row(new_u, new_du, time),
            _ => panic!("Incorrect combination of Simulation Data and State vector!!! (SimulationData::add_row)"),
        }
    }
}
//##################################################################################################
//##################################################################################################
//##################################################################################################
struct SimData1Dof {
    len: u64,
    time: Vec<f64>,
    data: Vec<[f64; 3]>,
}

impl SimData1Dof {
    const INITCAP: usize = 10000;
    fn new() -> Self {
        Self {
            len: 0,
            time: Vec::with_capacity(Self::INITCAP),
            data: Vec::with_capacity(Self::INITCAP),
        }
    }
    fn add_row(&mut self, new_u: [f64;2], new_du: [f64; 2], time: f64) {
        self.len += 1; // Can maybe speed up by adding this at very end (simulation iter #)
        self.time.push(time);
        self.data.push([new_u[0], new_u[1], new_du[1]]);
    }
}
//##################################################################################################
//##################################################################################################
//##################################################################################################
struct SimData3Dof {
    len: u64,
    time: Vec<f64>,
    data: Vec<[f64; 9]>,
}

impl SimData3Dof {
    const INITCAP: usize = 10000;
    fn new() -> Self {
        Self {
            len: 0,
            time: Vec::with_capacity(Self::INITCAP),
            data: Vec::with_capacity(Self::INITCAP),
        }
    }
    fn add_row(&mut self, new_u: [f64;6], new_du: [f64; 6], time: f64) {
        self.len += 1; // Can maybe speed up by adding this at very end (simulation iter #)
        self.time.push(time);
        self.data.push([new_u[0], new_u[1], new_u[2], new_u[3], new_u[4], new_u[5],
            new_du[3],new_du[4],new_du[5]]);
    }
}