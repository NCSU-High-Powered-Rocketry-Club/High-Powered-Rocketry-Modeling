use crate::math::vec_ops::{MathVector, VectorOperations};
use crate::math::Norm;
use crate::physics_mod;
use crate::rocket_mod::Rocket;
use std::f64::consts::PI;

#[derive(Debug, Clone, Copy)]
pub(crate) struct Dof6 {
    // This model is a 6 Degree of Freedom model which has 2 spatial dimensions
    pub(super) u: MathVector<12>, //[f64; 12], // (x y z, angle x y z;;; x y z velocity, anfgle x y z velocity)
    pub(super) dudt: MathVector<12>, // (dx, dy, dang, dvx, dvy, dvang)
    rocket: Rocket,
    is_current: bool,
    pub(super) time: f64,
    pub(super) ndim: u32,
}

/*
impl Dof6 {
    pub(crate) const NLOG: usize = 18;
    //Private Routines
    pub(crate) fn new(u: [f64; 12], rocket: Rocket) -> Self {
        Self {
            u: MathVector::new(u),
            dudt: MathVector::new([f64::NAN; 12]),
            rocket,
            is_current: false,
            time: 0.0,
            ndim: 12,
        }
    }
    pub(super) fn get_y_velocity(&self) -> f64 {
        self.u.data[7]
    }
    pub(super) fn get_height(&self) -> f64 {
        self.u.data[1]
    }
    pub(super) fn get_derivs_6dof(&mut self) -> MathVector<12> {
        if !self.is_current {
            self.update_state_derivatives();
        }
        self.dudt
    }
    pub(super) fn get_time_6dof(&self) -> f64 {
        self.time
    }
    pub(super) fn print_state_6dof(&self, i: u64) {
        println!(
            "Iter:{:6},    Time:{:5.2}(s),    Altitude:{:8.2}(m),    X Velocity:{:8.2}(m/s)    Y Velocity::{:8.2}(m/s)    Z Velocity:{:8.2}(rad/s)",
            i,
            self.get_time_6dof(),
            self.get_height(),
            self.u.data[7],
            self.get_y_velocity(),
            self.u.data[9]
        );
    }
    pub(super) fn get_logrow(&self) -> MathVector<18> {
        let mut row = [0.0; 18];
        row[0..12].copy_from_slice(&self.u.data[..]);
        row[12..18].copy_from_slice(&self.dudt.data[7..12]);
        MathVector::new(row)
    }
    pub(super) fn update_state(&mut self, du: MathVector<12>, dt: f64) {
        self.u += du;
        self.time += dt;
        self.is_current = false;
    }
    //
    pub(super) fn update_state_derivatives(&mut self) {
        // Find vector representing the rocket's orientation cand velocity
        let orientation = MathVector::new([self.u[3], self.u[4], self.u[5]]);
        let velocity = MathVector::new([self.u[6], self.u[7], self.u[8]]);

        // ========== Find Anglqe of attack
        //
        let vmag = velocity.norm_2();
        //
        // used to get the direction of angle of attack (pos = orientation ccw of velocity)
        let cross_prod = velocity.cross_3d(&orientation);
        // Need sub for alpha direction
        //
        // find component of velocity in direction of rocket
        let vel_comp_in_ori = velocity.dot(&orientation);
        //
        // Use trig to find the angle between the two vectors
        // Will give radians, with the convention being that the rocket pointing CCW of the velocity
        // is positive.
        let alpha = //(vel_comp_in_ori / vmag).acos() * alpha_dir;

        // ========== Forces
        //
        let force_drag = physics_mod::calc_drag_force(vmag, self.rocket.cd, self.rocket.area_drag);
        let drag_vec = //velocity.scale(force_drag / vmag);
        //
        let force_lift =
            physics_mod::calc_lift_force(vmag, self.rocket.cl_a, alpha, self.rocket.area_drag);
        let lift_vec = /*velocity
            .rotate_2d(&(0.5 * PI * alpha_dir))
            .scale(force_lift / vmag);
        //    */
        let sum_force = lift_vec + drag_vec;

        // ========== Moments
        // assuming that all aerodynamic forces are acting on the center of pressure of the rocket
        let moment_arm = orientation.scale(self.rocket.stab_margin_dimensional);
        let sum_moment = sum_force.cross_2d(&moment_arm);

        // ========== 2nd Order Derivatives of ODE System
        //Linear Acceleration
        let accel = sum_force.scale(1.0 / self.rocket.mass);
        let dvxdt = accel.data[0] / self.rocket.mass;
        let dvydt = accel.data[1] / self.rocket.mass + physics_mod::gravity();

        //Angular Acceleration
        let domegadt = sum_moment / self.rocket.inertia_z;

        // 1st order terms
        let dxdt = self.u[3];
        let dydt = self.u[4];
        let omega = self.u[5];

        self.dudt.data = [dxdt, dydt, omega, dvxdt, dvydt, domegadt];
        self.is_current = true;
    }
}
*/
