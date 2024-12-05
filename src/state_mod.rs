use crate::math::Norm;
use crate::{math::vec_ops::MathVector, physics_mod, Rocket};
use std::f64::consts::PI;
use std::process::exit;

#[derive(Debug, Clone, Copy)]
pub(crate) enum State {
    // Enumeration to unify the different available state spaces / ODE models
    // Each of these specifies what the simulation is really all about: what equations you are
    //         actually solving, what data types, and the number of variables that are needed,...
    __1DOF(Dof1),
    __3DOF(Dof3),
}

#[derive(Debug, Clone, Copy)]
pub(crate) enum StateVector {
    // Data type which represents an actual vector(rust::array) of the state space for a given model
    __1DOF([f64; 2]),
    __3DOF([f64; 6]),
}

impl State {
    pub(crate) fn print_state(&self, i: u64) {
        match self {
            // Custom printout to let the user know the status of the state during iterations
            State::__1DOF(dof1) => dof1.print_state_1dof(i),
            State::__3DOF(dof3) => dof3.print_state_3dof(i),
            _ => println!("Ignoring, State:print_state"),
        }
    }
    pub(crate) fn get_state_vec(&self) -> StateVector {
        // Return the current values of the state variables using that StateVector enum
        match self {
            State::__1DOF(dof1) => StateVector::__1DOF(dof1.u),
            State::__3DOF(dof3) => StateVector::__3DOF(dof3.u),
        }
    }
    pub(crate) fn get_ndim(&self) -> usize {
        // Get number of dimensions
        match self {
            State::__1DOF(_dof1) => 2usize,
            State::__3DOF(_dof3) => 6usize,
            _ => {
                println!("Ignoring, State:get_ndim");
                0usize
            }
        }
    }
    pub(crate) fn get_altitude(&self) -> f64 {
        // get the current elevation/height
        match self {
            State::__1DOF(dof1) => dof1.get_height(),
            State::__3DOF(dof3) => dof3.get_height(),
            _ => {
                println!("Ignoring, State:get_altitude");
                f64::NAN
            }
        }
    }
    pub(crate) fn get_vertical_velocity(&self) -> f64 {
        // get the velocity in the vertical direction
        match self {
            State::__1DOF(dof1) => dof1.get_velocity(),
            State::__3DOF(dof3) => dof3.get_y_velocity(),
            _ => {
                println!("Ignoring Invalid State, State:get_vertical_velocity");
                f64::NAN
            }
        }
    }
    pub(crate) fn get_time(&self) -> f64 {
        // return the value of the time variable
        match self {
            State::__1DOF(dof1) => dof1.get_time_1dof(),
            State::__3DOF(dof3) => dof3.get_time_3dof(),
            _ => {
                println!("Ignoring Invalid State, State:get_altitude");
                f64::NAN
            }
        }
    }
    pub(crate) fn get_derivs(&mut self) -> StateVector {
        // Return a vector which contains the derivatives of the model/state variables
        match self {
            State::__1DOF(dof1) => StateVector::__1DOF(dof1.get_derivs_1dof()),
            State::__3DOF(dof3) => StateVector::__3DOF(dof3.get_derivs_3dof()),
            _ => {
                println!("Unrecoverable Invalid State, State:get_derivs"); //This could be improved w/ option
                exit(0)
            }
        }
    }
    pub(crate) fn update(&mut self, du_vec: StateVector, dt: f64) -> () {
        // Used by the math module to modify the value of the current state once the timestep has
        //  been calculated by the OdeIterator
        match (self, du_vec) {
            (State::__1DOF(dof1), StateVector::__1DOF(du)) => dof1.update_state(du, dt),
            (State::__3DOF(dof3), StateVector::__3DOF(du)) => dof3.update_state(du, dt),
            _ => {
                println!("Ignoring Invalid State, State:update");
            }
        }
    }
    pub(crate) fn multiply(&self, u_vec: StateVector, k: f64) -> StateVector {
        // Function for multiplying a state vector by a scalar
        match (self, u_vec) {
            (State::__1DOF(dof1), StateVector::__1DOF(u)) => {
                StateVector::__1DOF(dof1.multiply(u, k))
            }
            (State::__3DOF(dof3), StateVector::__3DOF(u)) => {
                StateVector::__3DOF(dof3.multiply(u, k))
            }
            _ => {
                println!("Ignoring Invalid State, State:multiply");
                exit(1)
            }
        }
    }
    pub(crate) fn add(&self, u_vec: StateVector, v_vec: StateVector) -> StateVector {
        // Function for adding two state vectors to each other
        match (self, u_vec, v_vec) {
            (State::__1DOF(dof1), StateVector::__1DOF(u), StateVector::__1DOF(v)) => {
                StateVector::__1DOF(dof1.add(u, v))
            }
            (State::__3DOF(dof3), StateVector::__3DOF(u), StateVector::__3DOF(v)) => {
                StateVector::__3DOF(dof3.add(u, v))
            }
            _ => {
                println!("Ignoring Invalid State, State:add");
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
    // This model is a simple 1D, (position,velocity) model
    // The assumtion is that the rocket is flying perfectly vertical and that there are no
    // considerations about rotation or anything which would not be 1D in nature.
    u: [f64; 2],    // (height, velocity)
    dudt: [f64; 2], // (d_height, d_velocity)
    rocket: Rocket,
    is_current: bool,
    time: f64,
    ndim: u32,
}

impl Dof1 {
    //Private Routines
    pub(crate) fn new(u: [f64; 2], rocket: Rocket) -> Self {
        Self {
            u,
            dudt: [f64::NAN, f64::NAN],
            rocket,
            is_current: false,
            time: 0.0,
            ndim: 2,
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
            "Iter:{:6},    Time:{:5.2}(s),    Altitude:{:8.2}(m),    Velocity:{:8.2}(m/s)    Acceleration:{:8.2}(m/ss)",
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
            physics_mod::calc_drag_force(self.u[1], self.rocket.cd, self.rocket.area_drag);
        let g = physics_mod::gravity();

        // dhdt = velocity
        let dhdt = self.u[1];

        //a = F/m + g
        let dvdt = force_drag / self.rocket.mass + g;

        self.dudt = [dhdt, dvdt];
        self.is_current = true;
    }
}
//##################################################################################################
//##################################################################################################
//##################################################################################################

#[derive(Debug, Clone, Copy)]
pub(crate) struct Dof3 {
    // This model is a 3 Degree of Freedom model which has 2 spatial dimensions
    // (x=horizontal, y=vertical) and a 3rd variable for the rotation of the rocket
    // within that 2D space.
    u: [f64; 6], // (x position, y position, angle(ccw), x velocity, y velocity, angular veloicty)
    dudt: [f64; 6], // (dx, dy, dang, dvx, dvy, dvang)
    rocket: Rocket,
    is_current: bool,
    time: f64,
    ndim: u32,
}

impl Dof3 {
    //Private Routines
    pub(crate) fn new(u: [f64; 6], rocket: Rocket) -> Self {
        Self {
            u,
            dudt: [f64::NAN; 6],
            rocket,
            is_current: false,
            time: 0.0,
            ndim: 6,
        }
    }
    fn get_y_velocity(&self) -> f64 {
        self.u[4]
    }
    fn get_height(&self) -> f64 {
        self.u[1]
    }
    fn get_derivs_3dof(&mut self) -> [f64; 6] {
        if !self.is_current {
            self.update_state_derivatives();
        }
        self.dudt
    }
    fn get_time_3dof(&self) -> f64 {
        self.time
    }
    fn print_state_3dof(&self, i: u64) {
        println!(
            "Iter:{:6},    Time:{:5.2}(s),    Altitude:{:8.2}(m),    X Velocity:{:8.2}(m/s)    Y Velocity::{:8.2}(m/s)    AngularVelo:{:8.2}(rad/s)",
            i,
            self.get_time_3dof(),
            self.get_height(),
            self.u[3],
            self.get_y_velocity(),
            self.u[5]
        );
    }
    fn multiply(&self, u: [f64; 6], k: f64) -> [f64; 6] {
        MathVector::scale(&MathVector::new(u), k).data
        //[u[0] * k, u[1] * k, u[2] * k, u[3] * k, u[4] * k, u[5] * k]
    }
    fn add(&self, u: [f64; 6], v: [f64; 6]) -> [f64; 6] {
        (MathVector::new(u) + MathVector::new(v)).data
    }
    fn update_state(&mut self, du: [f64; 6], dt: f64) {
        for i in 0..self.ndim {
            self.u[i as usize] += du[i as usize]
        }
        self.time += dt;
        self.is_current = false;
    }
    fn update_state_derivatives(&mut self) {
        // Find vector representing the rocket's orientation cand velocity
        let ox = -1.0 * f64::sin(self.u[2]);
        let oy = 1.0 * f64::cos(self.u[2]);
        let orientation = MathVector::new([ox, oy]);
        let velocity = MathVector::new([self.u[3], self.u[4]]);

        // ========== Find Angle of attack
        //
        let vmag = velocity.norm_2();
        //
        // used to get the direction of angle of attack (pos = orientation ccw of velocity)
        let cross_prod = MathVector::cross_2d(&velocity, &orientation);
        let alpha_dir = cross_prod.signum();
        //
        // find component of velocity in direction of rocket
        let vel_comp_in_ori = MathVector::dot(&velocity, &orientation);
        //
        // Use trig to find the angle between the two vectors
        // Will give radians, with the convention being that the rocket pointing CCW of the velocity
        // is positive.
        let alpha = (vel_comp_in_ori / vmag).acos() * alpha_dir;

        // ========== Forces
        //
        let force_drag =
            physics_mod::calc_drag_force(vmag, self.rocket.cd, self.rocket.area_drag);
        let drag_vec = velocity.scale(force_drag / vmag);
        //
        let force_lift =
            physics_mod::calc_lift_force(vmag, self.rocket.cl_a, alpha, self.rocket.area_drag);
        let lift_vec = velocity
            .rotate_2d(&(0.5 * PI * alpha_dir))
            .scale(force_lift / vmag);
        //
        let sum_force = lift_vec + drag_vec;

        // ========== Moments
        // assuming that all aerodynamic forces are acting on the center of pressure of the rocket
        let moment_arm = orientation.scale(self.rocket.stab_margin_dimensional);
        let sum_moment = MathVector::cross_2d(&sum_force, &moment_arm);

        // ========== 2nd Order Derivatives of ODE System
        //Linear Acceleration
        let accel = sum_force.scale(1.0/self.rocket.mass);
        let dvxdt = accel.data[0] / self.rocket.mass;
        let dvydt = accel.data[1] / self.rocket.mass + physics_mod::gravity();

        //Angular Acceleration
        let domegadt = sum_moment / self.rocket.inertia_z;

        // 1st order terms
        let dxdt = self.u[3];
        let dydt = self.u[4];
        let omega = self.u[5];

        self.dudt = [dxdt, dydt, omega, dvxdt, dvydt, domegadt];
        self.is_current = true;
    }
}
