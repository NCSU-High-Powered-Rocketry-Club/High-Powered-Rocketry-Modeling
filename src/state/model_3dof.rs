use crate::{physics_mod, Rocket};
use nalgebra::{Rotation2, SVector, Vector2, Vector3, Vector6};
use std::f64::consts::PI;

#[derive(Debug, Clone, Copy)]
pub(crate) struct Dof3 {
    // This model is a 3 Degree of Freedom model which has 2 spatial dimensions
    // (x=horizontal, y=vertical) and a 3rd variable for the rotation of the rocket
    // within that 2D space.
    /// (x,y,angle,vx,vy,angular rate)
    pub(super) u: Vector6<f64>,
    /// (dxdt,dydt,d_angle_dt,dvxdt,dvydt,d_angular rate_dt)
    pub(super) dudt: Vector6<f64>,
    pub(crate) rocket: Rocket,
    pub(crate) is_current: bool,
    pub(super) time: f64,
}

impl Dof3 {
    pub(crate) fn new(u: Vector6<f64>, rocket: Rocket) -> Self {
        Self {
            u,
            dudt: Vector6::from_element(f64::NAN),
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
        self.update_state_derivatives();
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
        if self.is_current {
            return;
        }
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
        let cd_total = self.rocket.cd + self.rocket.cl_a * alpha.abs(); //crappy estimation for drag increasing with AoA

        let force_drag = physics_mod::calc_drag_force(vmag, cd_total, self.rocket.area_drag);
        let drag_vec = velocity * (force_drag / vmag);
        //
        let force_lift = physics_mod::calc_lift_force(
            vmag,
            self.rocket.cl_a,
            alpha.abs(),
            self.rocket.area_drag,
        );
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

#[cfg(test)]
mod tests {
    use super::*;
    use nalgebra::{Rotation2, SVector, Vector2, Vector6};
    use std::f64::consts::PI;

    fn assert_approx(a: f64, b: f64, tol: f64) {
        let diff = (a - b).abs();
        assert!(
            diff <= tol,
            "Expected {a} ≈ {b} (tol={tol}), but |a-b|={diff}"
        );
    }

    fn assert_vec6_approx(a: Vector6<f64>, b: Vector6<f64>, tol: f64) {
        for i in 0..6 {
            assert_approx(a[i], b[i], tol);
        }
    }

    fn assert_svec9_approx(a: SVector<f64, 9>, b: SVector<f64, 9>, tol: f64) {
        for i in 0..9 {
            assert_approx(a[i], b[i], tol);
        }
    }

    // Makes a rocket with known parameters for 3DOF tests.
    fn make_rocket() -> Rocket {
        Rocket {
            mass: 10.0,
            cd: 0.6,
            cl_a: 5.0,
            area_lift: 0.0,
            area_drag: 0.02,
            stab_margin_dimensional: 0.3,
            inertia_z: 2.5,
        }
    }

    #[test]
    fn new_sets_expected_initial_state() {
        let u0 = Vector6::new(1.0, 2.0, 0.1, 3.0, 4.0, 0.5);
        let rocket = make_rocket();
        let dof = Dof3::new(u0, rocket);

        assert_eq!(dof.u, u0);

        // dudt starts as all NaNs
        for i in 0..6 {
            assert!(dof.dudt[i].is_nan());
        }

        assert_eq!(dof.time, 0.0);
        assert!(!dof.is_current);
    }

    #[test]
    fn getters_return_expected_components() {
        let u0 = Vector6::new(0.0, 123.4, 0.0, -1.0, 9.87, 0.0);
        let rocket = make_rocket();
        let dof = Dof3::new(u0, rocket);

        assert_eq!(dof.get_height(), 123.4);
        assert_eq!(dof.get_y_velocity(), 9.87);
        assert_eq!(dof.get_time_3dof(), 0.0);
    }

    #[test]
    fn update_state_advances_u_and_time_and_invalidates_cache() {
        let u0 = Vector6::new(1.0, 2.0, 0.3, 4.0, 5.0, 0.6);
        let rocket = make_rocket();
        let mut dof = Dof3::new(u0, rocket);

        // Make derivatives current first
        dof.update_state_derivatives();
        assert!(dof.is_current);

        let du = Vector6::new(0.1, -0.2, 0.0, 1.0, -1.0, 0.05);
        dof.update_state(du, 0.25);

        assert_vec6_approx(dof.u, u0 + du, 0.0);
        assert_approx(dof.time, 0.25, 0.0);

        // should invalidate derivative cache
        assert!(!dof.is_current);
    }

    #[test]
    fn update_state_derivatives_is_cached_when_is_current_true() {
        // Makes a new state and rocket
        let u0 = Vector6::new(0.0, 0.0, 0.2, 30.0, 10.0, 0.1);
        let rocket = make_rocket();
        let mut dof = Dof3::new(u0, rocket);

        assert!(!dof.is_current);

        // First call: should compute dudt
        dof.update_state_derivatives();
        assert!(dof.is_current);

        let dudt_first = dof.dudt;

        // Call again: should early-return and not change dudt
        dof.update_state_derivatives();
        assert!(dof.is_current);
        let dudt_second = dof.dudt;

        assert_vec6_approx(dudt_first, dudt_second, 0.0);
    }

    #[test]
    fn get_derivs_3dof_always_updates_and_returns_dudt() {
        // Makes a new state and rocket
        let u0 = Vector6::new(0.0, 10.0, 0.0, 40.0, 0.0, 0.0);
        let rocket = make_rocket();
        let mut dof = Dof3::new(u0, rocket);

        let d1 = dof.get_derivs_3dof();
        assert!(dof.is_current);
        // dh/dt etc shouldn't be NaN
        for i in 0..6 {
            assert!(!d1[i].is_nan());
        }

        // second call should return identical result (cached)
        let d2 = dof.get_derivs_3dof();
        assert_vec6_approx(d1, d2, 0.0);
    }

    #[test]
    fn update_state_derivatives_matches_expected_formula_using_physics_mod() {
        // This test manually computes the expected dudt values doing the same physics calculations
        // and verifies dudt matches.

        // We choose a nonzero angle and both vx,vy nonzero to exercise alpha sign.
        let u0 = Vector6::new(0.0, 100.0, 0.4, 50.0, 20.0, 0.7);

        let rocket = make_rocket();
        let mut dof = Dof3::new(u0, rocket);

        dof.update_state_derivatives();
        let got = dof.dudt;

        // Get the expected values by doing the same physics calculations here
        let ox = -1.0 * f64::sin(u0[2]);
        let oy = 1.0 * f64::cos(u0[2]);
        let orientation = Vector2::new(ox, oy);

        let velocity = Vector2::new(u0[3], u0[4]);
        let vmag = velocity.norm();

        // Angle of attack sign
        let cross_prod = velocity.perp(&orientation);
        let alpha_dir = cross_prod.signum();

        let vel_comp_in_ori = velocity.dot(&orientation);
        let alpha = (vel_comp_in_ori / vmag).acos() * alpha_dir;

        let cd_total = dof.rocket.cd + dof.rocket.cl_a * alpha.abs();

        let force_drag = physics_mod::calc_drag_force(vmag, cd_total, dof.rocket.area_drag);
        let drag_vec = velocity * (force_drag / vmag);

        let force_lift =
            physics_mod::calc_lift_force(vmag, dof.rocket.cl_a, alpha.abs(), dof.rocket.area_drag);
        let lift_vec = Rotation2::new(0.5 * PI * alpha_dir) * velocity * (force_lift / vmag);

        let sum_force = lift_vec + drag_vec;

        let moment_arm = orientation * dof.rocket.stab_margin_dimensional;
        let sum_moment = sum_force.perp(&moment_arm);

        let accel = sum_force * (1.0 / dof.rocket.mass);
        let dvxdt = accel[0];
        let dvydt = accel[1] + physics_mod::gravity();
        let domegadt = sum_moment / dof.rocket.inertia_z;

        let expected = Vector6::new(
            u0[3], // dxdt
            u0[4], // dydt
            u0[5], // d(angle)/dt = omega
            dvxdt, dvydt, domegadt,
        );

        assert_vec6_approx(got, expected, 1e-12);
    }

    #[test]
    fn get_logrow_layout_is_correct() {
        // Test that the logrow contains the expected components in the expected order.
        let u0 = Vector6::new(1.0, 2.0, 0.3, 4.0, 5.0, 0.6);
        let rocket = make_rocket();
        let mut dof = Dof3::new(u0, rocket);

        dof.update_state_derivatives();

        let row = dof.get_logrow();

        let expected = SVector::<f64, 9>::from_row_slice(&[
            dof.u[0],
            dof.u[1],
            dof.u[2],
            dof.u[3],
            dof.u[4],
            dof.u[5],
            dof.dudt[3],
            dof.dudt[4],
            dof.dudt[5],
        ]);

        assert_svec9_approx(row, expected, 1e-12);
    }

    #[test]
    fn height_and_y_velocity_derivs_match_state_by_definition() {
        // Basic ODE structure checks:
        // dxdt == vx, dydt == vy, d(angle)/dt == omega.
        let u0 = Vector6::new(0.0, 10.0, 1.2, -3.0, 8.0, -0.4);
        let rocket = make_rocket();
        let mut dof = Dof3::new(u0, rocket);

        dof.update_state_derivatives();

        assert_approx(dof.dudt[0], u0[3], 1e-12);
        assert_approx(dof.dudt[1], u0[4], 1e-12);
        assert_approx(dof.dudt[2], u0[5], 1e-12);
    }
}
