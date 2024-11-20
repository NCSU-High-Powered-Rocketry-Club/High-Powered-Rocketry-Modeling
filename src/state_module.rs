use crate::{physics_module, Rocket};
use std::process::exit;

#[derive(Debug, Clone, Copy)]
pub(crate) enum State {
    S1D(Dof1),
    S2D(),
    S3D(),
    S4D(),
    S6D(),
}
impl State {
    // Functions in this block serve the purpose of breaking down general function calls to the
    // State enum down into what they mean for each individual state
    pub(crate) fn print_state(&self, i: u64) {
        match self {
            State::S1D(_Dof1) => self.print_state_dof1(i),
            _ => println!(
                "Undefined Operation(print_state) for given State({:?})",
                self
            ),
        }
    }
    pub(crate) fn get_height(&self) -> f64 {
        //This returns the altitude of the rocket
        match self {
            State::S1D(_Dof1) => self.get_height(),
            _ => {
                println!(
                    "Undefined Operation(get_height) for given State({:?})",
                    self
                );
                f64::NAN
            }
        }
    }
    pub(crate) fn get_vertical_velocity(&self) -> f64 {
        //This returns the altitude of the rocket
        match self {
            State::S1D(_Dof1) => self.get_velocity(),
            _ => {
                println!(
                    "Undefined Operation(get_height) for given State({:?})",
                    self
                );
                f64::NAN
            }
        }
    }
    pub(crate) fn get_time(&self) -> f64 {
        match self {
            State::S1D(_Dof1) => self.get_time(),
            _ => f64::NAN,
        }
    }
}

//##################################################################################################
//##################################################################################################
//##################################################################################################

#[derive(Debug, Clone, Copy)]
pub(crate) struct Dof1 {
    u: (f64, f64),    // (height, velocity)
    dudt: (f64, f64), // (d_height, d_velocity)
    rocket: Rocket,
    is_current: bool,
    time: f64,
    ndim: usize,
}

impl Dof1 {
    // Public Routines
    pub(crate) fn new(u: (f64, f64), rocket: Rocket) -> Self {
        Self {
            u,
            dudt: (f64::NAN, f64::NAN),
            rocket,
            is_current: false,
            time: 0.0,
            ndim: 2,
        }
    }
    pub(crate) fn get_velocity(&self) -> f64 {
        self.u.1
    }
    pub(crate) fn get_height(&self) -> f64 {
        self.u.0
    }
    pub(crate) fn get_derivs(&mut self) -> (f64, f64) {
        if !self.is_current {
            self.update_state_derivatives();
        }
        self.dudt
    }
    pub(crate) fn print_state_dof1(&self, i: i64) {
        println!(
            "Iter:{:6},    Time:{:5.2}(s),    Altitude:{:8.2}(m),    Velocity:{:8.2}(m/s)    Acceleration:{:8.2}(m/ss)\n",
            i,
            self.get_time(),
            self.get_height(),
            self.get_velocity(),
            self.dvdt().1
        );
    }

    pub(crate) fn update_dof1(&mut self, du: (f64,f64), dt: f64) {
        self.u.0 += du.0;
        self.u.1 += du.1;
        self.time += dt;
        self.is_current = false;
    }

    //Private Routines
    fn update_state_derivatives(&mut self) {
        let force_drag =
            physics_module::calc_drag_force(self.u.1, self.rocket.cd, self.rocket.area);
        let g = physics_module::gravity();

        // dhdt = velocity
        let dhdt = self.u.1;

        //a = F/m + g
        let dvdt = force_drag / self.rocket.mass + g;

        self.dudt = (dhdt, dvdt);
        self.is_current = true;
    }
}
