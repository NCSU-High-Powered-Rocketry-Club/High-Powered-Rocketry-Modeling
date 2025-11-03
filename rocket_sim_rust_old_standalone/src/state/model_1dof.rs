use crate::math::vec_ops::MathVector;
use crate::physics_mod;
use crate::rocket_mod::Rocket;

#[derive(Debug, Clone, Copy)]
pub(crate) struct Dof1 {
    // This model is a simple 1D, (position,velocity) model
    // The assumtion is that the rocket is flying perfectly vertical and that there are no
    // considerations about rotation or anything which would not be 1D in nature.
    pub(super) u: MathVector<2>,    // (height, velocity)
    pub(super) dudt: MathVector<2>, // (d_height, d_velocity)
    rocket: Rocket,
    is_current: bool,
    pub(super) time: f64,
    pub(super) ndim: u32,
}

impl Dof1 {
    pub(crate) const NLOG: usize = 3;
    //Private Routines
    pub(crate) fn new(u: [f64; 2], rocket: Rocket) -> Self {
        Self {
            u: MathVector::new(u),
            dudt: MathVector::new([f64::NAN, f64::NAN]),
            rocket,
            is_current: false,
            time: 0.0,
            ndim: 2,
        }
    }
    pub(super) fn get_velocity(&self) -> f64 {
        self.u.data[1]
    }
    pub(super) fn get_height(&self) -> f64 {
        self.u.data[0]
    }
    pub(super) fn get_derivs_1dof(&mut self) -> MathVector<2> {
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
            self.dudt.data[1]
        );
    }
    pub(super) fn get_logrow(&self) -> MathVector<3> {
        MathVector::new([self.u[0], self.u[1], self.dudt[1]])
    }
    pub(super) fn update_state(&mut self, du: MathVector<2>, dt: f64) {
        self.u += du;
        self.time += dt;
        self.is_current = false;
    }
    pub(super) fn update_state_derivatives(&mut self) {
        let force_drag =
            physics_mod::calc_drag_force(self.u.data[1], self.rocket.cd, self.rocket.area_drag);
        let g = physics_mod::gravity();

        // dhdt = velocity
        let dhdt = self.u.data[1];

        //a = F/m + g
        let dvdt = force_drag / self.rocket.mass + g;

        self.dudt.data = [dhdt, dvdt];
        self.is_current = true;
    }
}
