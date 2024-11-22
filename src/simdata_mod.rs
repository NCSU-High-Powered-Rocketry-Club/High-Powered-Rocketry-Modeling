

trait SimulationData {
    
}

pub(crate) struct SimData1Dof {
    len: u64,
    time: Vec<f64>,
    u: (Vec<f64>, Vec<f64>),
    dudt: Vec<f64>,
}
impl SimData1Dof {
    const INITCAP:usize = 10000;
    pub(crate) fn new() -> Self {
        Self {
            len: 0,
            time: Vec::with_capacity(Self::INITCAP),
            u: (Vec::with_capacity(Self::INITCAP), Vec::with_capacity(Self::INITCAP)),
            dudt: Vec::with_capacity(Self::INITCAP),
        }
    }
}