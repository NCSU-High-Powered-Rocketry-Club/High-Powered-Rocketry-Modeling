use crate::math::Norm;
use crate::physics_mod;
use crate::rocket_mod::Rocket;
use std::f64::consts::PI;
extern crate nalgebra as na;
use nalgebra::{Rotation2, SVector, Vector2, Vector3, Vector6};

#[derive(Debug, Clone, Copy)]
pub(crate) struct Dof3 {
    // This model is a 3 Degree of Freedom model which has 2 spatial dimensions
    // (x=horizontal, y=vertical) and a 3rd variable for the rotation of the rocket
    // within that 2D space.
    pub(super) u: Vector6<f64>, //[f64; 6], // (x position, y position, angle(ccw), x velocity, y velocity, angular veloicty)
    pub(super) dudt: Vector6<f64>, // (dx, dy, dang, dvx, dvy, dvang)
    pub(crate) rocket: Rocket,
    pub(crate) is_current: bool,
    pub(super) time: f64,
}

impl Dof3 {
    pub(crate) const NLOG: usize = 9;
    //Private Routines
    pub(crate) fn new(u: [f64; 6], rocket: Rocket) -> Self {
        Self {
            u: Vector6::new(u[0], u[1], u[2], u[3], u[4], u[5]),
            dudt: Vector6::new(f64::NAN, f64::NAN, f64::NAN, f64::NAN, f64::NAN, f64::NAN),
            rocket,
            is_current: false,
            time: 0.0,
        }
    }
    pub(super) fn get_y_velocity(&self) -> f64 {
        self.u[4]
    }
    pub(super) fn get_height(&self) -> f64 {
        self.u[1]
    }
    pub(super) fn get_derivs_3dof(&mut self) -> Vector6<f64> {
        if !self.is_current {
            self.update_state_derivatives();
        }
        self.dudt
    }
    pub(super) fn get_time_3dof(&self) -> f64 {
        self.time
    }
    pub(super) fn print_state_3dof(&self, i: u64) {
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
    pub(super) fn get_logrow(&self) -> SVector<f64, 9> {
        let mut row = [0.0; 9];
        row[0..6].copy_from_slice(&self.u.as_slice()[..]);
        row[6..9].copy_from_slice(&self.dudt.as_slice()[3..6]);
        SVector::<f64, 9>::from_row_slice(&row)
    }
    pub(super) fn update_state(&mut self, du: Vector6<f64>, dt: f64) {
        self.u += du;
        self.time += dt;
        self.is_current = false;
    }
    //
    pub(super) fn update_state_derivatives(&mut self) {
        // Find vector representing the rocket's orientation cand velocity
        let ox = -1.0 * f64::sin(self.u[2]);
        let oy = 1.0 * f64::cos(self.u[2]);
        let orientation = Vector2::new(ox, oy);
        let velocity = Vector2::new(self.u[3], self.u[4]);

        // ========== Find Angle of attack
        //
        let vmag = velocity.norm();
        //
        // used to get the direction of angle of attack (pos = orientation ccw of velocity)
        let cross_prod = velocity.perp(&orientation);
        let alpha_dir = cross_prod.signum();
        //
        // find component of velocity in direction of rocket
        let vel_comp_in_ori = velocity.dot(&orientation);
        //
        // Use trig to find the angle between the two vectors
        // Will give radians, with the convention being that the rocket pointing CCW of the velocity
        // is positive.
        let alpha = (vel_comp_in_ori / vmag).acos() * alpha_dir;

        // ========== Forces
        //
        let cd_total = self.rocket.cd + self.rocket.cl_a*alpha.abs();//crappy estimation for drag increasing with AoA

        let force_drag = physics_mod::calc_drag_force(vmag, cd_total, self.rocket.area_drag);
        let drag_vec = velocity * (force_drag / vmag);
        //
        let force_lift =
            physics_mod::calc_lift_force(vmag, self.rocket.cl_a, alpha.abs(), self.rocket.area_drag);
        let lift_vec = Rotation2::new(0.5 * PI * alpha_dir) * velocity * (force_lift / vmag);
        //
        let sum_force = lift_vec + drag_vec;

        // ========== Moments
        // assuming that all aerodynamic forces are acting on the center of pressure of the rocket
        let moment_arm = orientation * (self.rocket.stab_margin_dimensional);
        let sum_moment = sum_force.perp(&moment_arm);

        // ========== 2nd Order Derivatives of ODE System
        //Linear Acceleration
        let accel = sum_force * (1.0 / self.rocket.mass);
        let dvxdt = accel[0];
        let dvydt = accel[1] + physics_mod::gravity();

        //Angular Acceleration
        let domegadt = sum_moment / self.rocket.inertia_z;

        // 1st order terms
        let dxdt = self.u[3];
        let dydt = self.u[4];
        let omega = self.u[5];

        self.dudt = Vector6::new(dxdt, dydt, omega, dvxdt, dvydt, domegadt);
        self.is_current = true;
    }
}
