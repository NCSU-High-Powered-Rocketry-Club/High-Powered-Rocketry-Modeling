use crate::{physics_module, Rocket};

pub(crate) struct State {
    height: f64,
    velocity: f64,
    dhdt: f64,
    dvdt: f64,
    rocket: Rocket,
    is_current: bool,
    time: f64,
}
impl State {
    // Public Routines
    pub(crate) fn new(height: f64, velocity: f64, rocket: Rocket) -> State {
        State {
            height,
            velocity,
            dhdt: f64::NAN,
            dvdt: f64::NAN,
            rocket,
            is_current: false,
            time: 0.0,
        }
    }
    pub(crate) fn get_velocity(&self) -> f64 {
        self.velocity
    }
    pub(crate) fn get_height(&self) -> f64 {
        self.height
    }
    pub(crate) fn get_time(&self) -> f64 {
        self.time
    }
    pub(crate) fn get_derivs(&mut self) -> (f64, f64) {
        if !self.is_current {
            self.update_state_derivatives();
        }
        (self.dhdt, self.dvdt)
    }
    pub(crate) fn update(&mut self, dh: f64, dv: f64, dt: f64) {
        self.height += dh;
        self.velocity += dv;
        self.time += dt;
        self.is_current = false;
    }
}

impl State {
    //Private Routines
    fn update_state_derivatives(&mut self) {
        let force_drag =
            physics_module::calc_drag_force(self.velocity, self.rocket.cd, self.rocket.area);
        let g = physics_module::gravity();

        // dhdt = velocity
        self.dhdt = self.velocity;

        //a = F/m + g
        self.dvdt = force_drag / self.rocket.mass + g;

        self.is_current = true;
    }
}
