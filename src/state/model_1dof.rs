//use crate::math::vec_ops::MathVector;
use crate::physics_mod;
use crate::rocket_mod::Rocket;
use nalgebra::{Vector2, Vector3};

#[derive(Debug, Clone, Copy)]
pub(crate) struct Dof1 {
    // This model is a simple 1D, (position,velocity) model
    // The assumtion is that the rocket is flying perfectly vertical and that there are no
    // considerations about rotation or anything which would not be 3D in nature.
    /// (height, velocity)
    pub(super) u: Vector2<f64>,    
    /// (d_height/dt, d_velocity/dt)
    pub(super) dudt: Vector2<f64>,
    rocket: Rocket,
    is_current: bool,
    pub(super) time: f64,
}

impl Dof1 {
    pub(crate) const NLOG: usize = 3;
    //
    pub(crate) fn new(u: Vector2<f64>, rocket: Rocket) -> Self {
        Self {
            u,
            dudt: Vector2::new(f64::NAN, f64::NAN),
            rocket,
            is_current: false,
            time: 0.0,
        }
    }
    pub(super) fn get_velocity(&self) -> f64 {
        self.u[1]
    }
    pub(super) fn get_height(&self) -> f64 {
        self.u[0]
    }
    pub(super) fn get_derivs_1dof(&mut self) -> Vector2<f64> {
        if !self.is_current {
            self.update_state_derivatives();
        }
        self.dudt
    }
    pub(super) fn get_time_1dof(&self) -> f64 {
        self.time
    }
    pub(super) fn print_state_1dof(&self, i: u64) {
        println!(
            "Iter:{:6},    Time:{:5.2}(s),    Altitude:{:8.2}(m),    Velocity:{:8.2}(m/s)    Acceleration:{:8.2}(m/ss)",
            i,
            self.get_time_1dof(),
            self.get_height(),
            self.get_velocity(),
            self.dudt[1]
        );
    }
    pub(super) fn get_logrow(&self) -> Vector3<f64> {
        Vector3::new(self.u[0], self.u[1], self.dudt[1])
    }
    pub(super) fn update_state(&mut self, du: Vector2<f64>, dt: f64) {
        self.u += du;
        self.time += dt;
        self.is_current = false;
    }
    pub(super) fn update_state_derivatives(&mut self) {
        let force_drag =
            physics_mod::calc_drag_force(self.u[1], self.rocket.cd, self.rocket.area_drag);
        let g = physics_mod::gravity();

        // dhdt = velocity
        let dhdt = self.u[1];

        //a = F/m + g
        let dvdt = force_drag / self.rocket.mass + g;

        self.dudt = Vector2::new(dhdt, dvdt);
        self.is_current = true;
    }
}
