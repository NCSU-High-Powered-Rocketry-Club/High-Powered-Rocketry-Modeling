//use crate::math::vec_ops::MathVector;
use crate::{Rocket, physics_mod};
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

#[cfg(test)]
mod tests {
    use super::*;
    use nalgebra::{Vector2, Vector3};

    // Small helper for floating-point comparisons.
    fn assert_approx(a: f64, b: f64, tol: f64) {
        let diff = (a - b).abs();
        assert!(
            diff <= tol,
            "Expected {a} ≈ {b} (tol={tol}), but |a-b|={diff}"
        );
    }

    fn assert_vec2_approx(a: Vector2<f64>, b: Vector2<f64>, tol: f64) {
        assert_approx(a[0], b[0], tol);
        assert_approx(a[1], b[1], tol);
    }

    fn assert_vec3_approx(a: Vector3<f64>, b: Vector3<f64>, tol: f64) {
        assert_approx(a[0], b[0], tol);
        assert_approx(a[1], b[1], tol);
        assert_approx(a[2], b[2], tol);
    }

    // Make a rocket with known parameters.
    //
    // NOTE:
    // If Rocket::default() doesn't exist, replace this with whatever constructor
    // your crate provides (or a full struct literal with all required fields).
    fn make_rocket(mass: f64, cd: f64, area_drag: f64) -> Rocket {
        Rocket {
            mass,
            cd,
            area_drag,
            area_lift: 0.0,
            inertia_z: 0.0,
            stab_margin_dimensional: 0.0,
            cl_a: 0.0,
        }
    }

    #[test]
    fn new_sets_expected_initial_state() {
        let u0 = Vector2::new(123.0, -4.5);
        let rocket = make_rocket(10.0, 0.6, 0.01);

        let dof = Dof1::new(u0, rocket);

        // State
        assert_eq!(dof.u, u0);

        // dudt starts as NaN, NaN
        assert!(dof.dudt[0].is_nan());
        assert!(dof.dudt[1].is_nan());

        // time starts at 0.0
        assert_eq!(dof.time, 0.0);

        // is_current starts false
        assert!(!dof.is_current);
    }

    #[test]
    fn getters_return_components() {
        let u0 = Vector2::new(50.0, 12.34);
        let rocket = make_rocket(5.0, 0.5, 0.02);

        let dof = Dof1::new(u0, rocket);

        assert_eq!(dof.get_height(), 50.0);
        assert_eq!(dof.get_velocity(), 12.34);
        assert_eq!(dof.get_time_1dof(), 0.0);
    }

    #[test]
    fn update_state_advances_u_and_time_and_invalidates_derivs() {
        let u0 = Vector2::new(1.0, 2.0);
        let rocket = make_rocket(5.0, 0.5, 0.02);
        let mut dof = Dof1::new(u0, rocket);

        // Force derivatives to be current first
        let _ = dof.get_derivs_1dof();
        assert!(dof.is_current);

        let du = Vector2::new(0.25, -0.5);
        let dt = 0.1;
        dof.update_state(du, dt);

        assert_vec2_approx(dof.u, Vector2::new(1.25, 1.5), 0.0);
        assert_approx(dof.time, 0.1, 0.0);

        // should invalidate cached derivatives
        assert!(!dof.is_current);
    }

    #[test]
    fn update_state_derivatives_matches_physics_mod() {
        let h = 100.0;
        let v = 50.0;
        let mass = 10.0;
        let cd = 0.75;
        let area = 0.02;

        let rocket = make_rocket(mass, cd, area);
        let mut dof = Dof1::new(Vector2::new(h, v), rocket);

        dof.update_state_derivatives();

        // dhdt = v
        assert_approx(dof.dudt[0], v, 1e-12);

        // dvdt = drag/m + g (using the same functions as production code)
        let drag = physics_mod::calc_drag_force(v, cd, area);
        let g = physics_mod::gravity();
        let expected_dvdt = drag / mass + g;

        assert_approx(dof.dudt[1], expected_dvdt, 1e-12);
        assert!(dof.is_current);
    }

    #[test]
    fn get_derivs_1dof_computes_when_stale_and_caches() {
        let h = 10.0;
        let v = 20.0;
        let mass = 2.0;
        let cd = 0.5;
        let area = 0.01;

        let rocket = make_rocket(mass, cd, area);
        let mut dof = Dof1::new(Vector2::new(h, v), rocket);

        // Initially stale
        assert!(!dof.is_current);
        assert!(dof.dudt[0].is_nan());
        assert!(dof.dudt[1].is_nan());

        // First call should compute
        let d1 = dof.get_derivs_1dof();
        assert!(dof.is_current);
        assert!(!d1[0].is_nan());
        assert!(!d1[1].is_nan());

        // Second call should return the same values (cached)
        let d2 = dof.get_derivs_1dof();
        assert_vec2_approx(d1, d2, 0.0);
    }

    #[test]
    fn after_update_state_get_derivs_1dof_recomputes() {
        let mass = 3.0;
        let cd = 0.8;
        let area = 0.015;

        let rocket = make_rocket(mass, cd, area);
        let mut dof = Dof1::new(Vector2::new(0.0, 10.0), rocket);

        let d_before = dof.get_derivs_1dof();
        assert!(dof.is_current);

        // Change velocity only; invalidate cache
        dof.update_state(Vector2::new(0.0, 5.0), 0.0);
        assert!(!dof.is_current);

        let d_after = dof.get_derivs_1dof();

        // dhdt should change because velocity changed
        assert!(d_after[0] != d_before[0]);

        // dvdt should also generally change because drag depends on v
        assert!(d_after[1] != d_before[1]);
    }

    #[test]
    fn get_logrow_returns_h_v_a() {
        let h = 42.0;
        let v = -7.0;
        let rocket = make_rocket(5.0, 0.5, 0.02);
        let mut dof = Dof1::new(Vector2::new(h, v), rocket);

        // Ensure dudt is computed so logrow isn't using NaN accel
        dof.update_state_derivatives();

        let row = dof.get_logrow();
        assert_approx(row[0], h, 1e-12);
        assert_approx(row[1], v, 1e-12);
        assert_approx(row[2], dof.dudt[1], 1e-12);
    }
}
