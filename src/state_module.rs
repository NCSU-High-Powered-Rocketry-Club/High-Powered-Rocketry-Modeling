use crate::{physics_module, Rocket};
use std::process::exit;
/*
pub(crate) trait StateSpaceFunctions<T> {
    fn print_state(&self, i: u64) -> ();
    fn get_altitude(&self) -> f64;
    fn get_vertical_velocity(&self) -> f64;
    fn get_time(&self) -> f64;
    fn get_derivs(&mut self) -> T;
    fn update(&mut self, du: T, dt: f64);
}
*/

#[derive(Debug, Clone, Copy)]
pub(crate) enum State {
    __1DOF(Dof1),
    __3DOF(),
}

#[derive(Debug, Clone, Copy)]
pub(crate) enum StateVector {
    __1DOF([f64; 2]),
}

const NDIM1DOF: usize = 2;

impl State {
    pub(crate) fn print_state(&self, i: u64) {
        match self {
            State::__1DOF(dof1) => dof1.print_state_1dof(i),
            _ => println!("Ignoring, State:print_state"),
        }
    }
    pub(crate) fn get_ndim(&self) -> usize {
        match self {
            State::__1DOF(_dof1) => NDIM1DOF,
            _ => {
                println!("Ignoring, State:get_ndim");
                0usize
            }
        }
    }
    pub(crate) fn get_altitude(&self) -> f64 {
        match self {
            State::__1DOF(dof1) => dof1.get_height(),
            _ => {
                println!("Ignoring, State:get_altitude");
                f64::NAN
            }
        }
    }
    pub(crate) fn get_vertical_velocity(&self) -> f64 {
        match self {
            State::__1DOF(dof1) => dof1.get_velocity(),
            _ => {
                println!("Ignoring Invalid State, State:get_vertical_velocity");
                f64::NAN
            }
        }
    }
    pub(crate) fn get_time(&self) -> f64 {
        match self {
            State::__1DOF(dof1) => dof1.get_time_1dof(),
            _ => {
                println!("Ignoring Invalid State, State:get_altitude");
                f64::NAN
            }
        }
    }
    pub(crate) fn get_derivs(&mut self) -> StateVector {
        match self {
            State::__1DOF(dof1) => StateVector::__1DOF(dof1.get_derivs_1dof()),
            _ => {
                println!("Unrecoverable Invalid State, State:get_derivs"); //This could be improved w/ option
                exit(0)
            }
        }
    }
    pub(crate) fn update(&mut self, du_vec: StateVector, dt: f64) -> () {
        match (self, du_vec) {
            (State::__1DOF(dof1), StateVector::__1DOF(du)) => dof1.update_state(du, dt),
            _ => {
                println!("Ignoring Invalid State, State:multiply");
            }
        }
    }
    pub(crate) fn multiply(&self, u_vec: StateVector, k: f64) -> StateVector {
        match (self, u_vec) {
            (State::__1DOF(dof1), StateVector::__1DOF(u)) => {
                StateVector::__1DOF(dof1.multiply(u, k))
            }
            _ => {
                println!("Ignoring Invalid State, State:multiply");
                exit(1)
            }
        }
    }
    pub(crate) fn add(&self, u_vec: StateVector, v_vec: StateVector) -> StateVector {
        match (self, u_vec, v_vec) {
            (State::__1DOF(dof1), StateVector::__1DOF(u), StateVector::__1DOF(v)) => {
                StateVector::__1DOF(dof1.add(u, v))
            }
            _ => {
                println!("Ignoring Invalid State, State:multiply");
                exit(1)
            }
        }
    }
}

//##################################################################################################
//##################################################################################################
//##################################################################################################

#[derive(Debug, Clone, Copy)]
pub(crate) struct Dof1 {
    u: [f64; 2],    // (height, velocity)
    dudt: [f64; 2], // (d_height, d_velocity)
    rocket: Rocket,
    is_current: bool,
    time: f64,
    ndim: u32,
}
/*
impl StateSpaceFunctions<[f64; 2]> for Dof1{
    fn print_state(&self, i: u64) -> (){
        self.print_state_1dof(i)
    }
    fn get_altitude(&self) -> f64{
        self.get_height()
    }
    fn get_vertical_velocity(&self) -> f64{
        self.get_velocity()
    }
    fn get_time(&self) -> f64{
        self.get_time_1dof()
    }
    fn get_derivs(&mut self) -> [f64; 2]{
        self.get_derivs_1dof()
    }
    fn update(&mut self, du: [f64; 2], dt: f64) {
        self.update_state(du,dt)
    }
}
*/

impl Dof1 {
    //Private Routines
    pub(crate) fn new(u: [f64; 2], rocket: Rocket) -> Self {
        const NDIM2: u32 = 2;
        Self {
            u,
            dudt: [f64::NAN, f64::NAN],
            rocket,
            is_current: false,
            time: 0.0,
            ndim: NDIM2,
        }
    }
    fn get_velocity(&self) -> f64 {
        self.u[1]
    }
    fn get_height(&self) -> f64 {
        self.u[0]
    }
    fn get_derivs_1dof(&mut self) -> [f64; 2] {
        if !self.is_current {
            self.update_state_derivatives();
        }
        self.dudt
    }
    fn get_time_1dof(&self) -> f64 {
        self.time
    }
    fn print_state_1dof(&self, i: u64) {
        println!(
            "Iter:{:6},    Time:{:5.2}(s),    Altitude:{:8.2}(m),    Velocity:{:8.2}(m/s)    Acceleration:{:8.2}(m/ss)\n",
            i,
            self.get_time_1dof(),
            self.get_height(),
            self.get_velocity(),
            self.dudt[1]
        );
    }
    fn multiply(&self, u: [f64; 2], k: f64) -> [f64; 2] {
        [u[0] * k, u[1] * k]
    }
    fn add(&self, u: [f64; 2], v: [f64; 2]) -> [f64; 2] {
        [u[0] + v[0], u[1] + v[1]]
    }
    fn update_state(&mut self, du: [f64; 2], dt: f64) {
        self.u[0] += du[0];
        self.u[1] += du[1];
        self.time += dt;
        self.is_current = false;
    }
    fn update_state_derivatives(&mut self) {
        let force_drag =
            physics_module::calc_drag_force(self.u[1], self.rocket.cd, self.rocket.area);
        let g = physics_module::gravity();

        // dhdt = velocity
        let dhdt = self.u[1];

        //a = F/m + g
        let dvdt = force_drag / self.rocket.mass + g;

        self.dudt = [dhdt, dvdt];
        self.is_current = true;
    }
}
