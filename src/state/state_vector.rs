use nalgebra::{SVector, Vector2, Vector3, Vector6};

use std::ops::{Add, AddAssign, Mul, MulAssign, Sub, SubAssign};

/// Data type which represents an actual vector(rust::array) of the state space for a given model. It
/// is the struct used to hold the state that the simulation is in as well as do the vector math for the
/// state space. It contains Log variants of the state vector, which are used to log the state of the
/// simulation at each timestep, and contain more information than the state vector used for the actual
/// simulation (e.g. the 1DOFLog contains acceleration, which is not needed for the actual simulation,
/// but is useful for the end user).
#[derive(Debug, Clone, Copy, PartialEq)]
pub(crate) enum StateVector {
    /// One DOF state vector, contains [altitude, velocity]
    OneDOF(Vector2<f64>),
    /// One DOF log state vector, contains [altitude, velocity, acceleration]
    OneDOFLog(Vector3<f64>),
    /// Three DOF state vector, contains [x, y, angle, vx, vy, angular rate]
    ThreeDOF(Vector6<f64>),
    /// Three DOF log state vector, contains [x, y, angle, vx, vy, angular rate, ax, ay, angular acceleration]
    ThreeDOFLog(SVector<f64, 9>),
}

impl Add for StateVector {
    type Output = Self;

    fn add(self, rhs: Self) -> Self::Output {
        match (self, rhs) {
            (StateVector::OneDOF(avec), StateVector::OneDOF(bvec)) => {
                StateVector::OneDOF(avec + bvec)
            }
            (StateVector::ThreeDOF(avec), StateVector::ThreeDOF(bvec)) => {
                StateVector::ThreeDOF(avec + bvec)
            }
            _ => {
                panic!("Invalid addition, mismatching State Vectors.")
            }
        }
    }
}

impl AddAssign for StateVector {
    fn add_assign(&mut self, rhs: Self) {
        match (self, rhs) {
            (StateVector::OneDOF(avec), StateVector::OneDOF(bvec)) => *avec += bvec,
            (StateVector::ThreeDOF(avec), StateVector::ThreeDOF(bvec)) => *avec += bvec,
            _ => {
                panic!("Invalid addition, mismatching State Vectors.")
            }
        }
    }
}

impl Sub for StateVector {
    type Output = Self;

    fn sub(self, rhs: Self) -> Self::Output {
        match (self, rhs) {
            (StateVector::OneDOF(avec), StateVector::OneDOF(bvec)) => {
                StateVector::OneDOF(avec - bvec)
            }
            (StateVector::ThreeDOF(avec), StateVector::ThreeDOF(bvec)) => {
                StateVector::ThreeDOF(avec - bvec)
            }
            _ => {
                panic!("Invalid addition, mismatching State Vectors.")
            }
        }
    }
}

impl SubAssign for StateVector {
    fn sub_assign(&mut self, rhs: Self) {
        match (self, rhs) {
            (StateVector::OneDOF(avec), StateVector::OneDOF(bvec)) => *avec -= bvec,
            (StateVector::ThreeDOF(avec), StateVector::ThreeDOF(bvec)) => *avec -= bvec,
            _ => {
                panic!("Invalid addition, mismatching State Vectors.")
            }
        }
    }
}

impl Mul for StateVector {
    type Output = Self;

    fn mul(self, rhs: Self) -> Self::Output {
        match (self, rhs) {
            (StateVector::OneDOF(avec), StateVector::OneDOF(bvec)) => {
                StateVector::OneDOF(avec.component_mul(&bvec))
            }
            (StateVector::ThreeDOF(avec), StateVector::ThreeDOF(bvec)) => {
                StateVector::ThreeDOF(avec.component_mul(&bvec))
            }
            _ => {
                panic!("Invalid addition, mismatching State Vectors.")
            }
        }
    }
}

impl MulAssign for StateVector {
    fn mul_assign(&mut self, rhs: Self) {
        match (self, rhs) {
            (StateVector::OneDOF(avec), StateVector::OneDOF(bvec)) => {
                avec.component_mul_assign(&bvec)
            }
            (StateVector::ThreeDOF(avec), StateVector::ThreeDOF(bvec)) => {
                avec.component_mul_assign(&bvec)
            }
            _ => {
                panic!("Invalid addition, mismatching State Vectors.")
            }
        }
    }
}

impl StateVector {
    pub fn dot(&self, b: &Self) -> f64 {
        match (self, b) {
            (StateVector::OneDOF(avec), StateVector::OneDOF(bvec)) => avec.dot(bvec),
            (StateVector::ThreeDOF(avec), StateVector::ThreeDOF(bvec)) => avec.dot(bvec),
            _ => {
                panic!("Invalid Dot Product, mismatching State Vectors.")
            }
        }
    }

    pub fn scale(&self, k: f64) -> Self {
        match self {
            StateVector::OneDOF(avec) => StateVector::OneDOF(avec * k),
            StateVector::ThreeDOF(avec) => StateVector::ThreeDOF(avec * k),
            _ => {
                panic!("State Vector Scale Impl")
            }
        }
    }

    #[allow(dead_code)]
    pub fn cross_2d(&self, in2: &Vector2<f64>) -> f64 {
        match self {
            StateVector::OneDOF(avec) => avec.perp(in2),
            StateVector::ThreeDOF(_) => panic!("Requires 2d math vector"),
            StateVector::OneDOFLog(_) => panic!("Requires 2d math vector"),
            StateVector::ThreeDOFLog(_) => panic!("Requires 2d math vector"),
        }
    }

    #[allow(dead_code)]
    pub fn cross_3d(&self, _in2: &Vector3<f64>) -> Vector3<f64> {
        match self {
            StateVector::OneDOF(_) => panic!("Requires 3d math vector"),
            StateVector::ThreeDOF(_) => panic!("Requires 3d math vector"),
            StateVector::OneDOFLog(_) => panic!("Requires 3d math vector"),
            StateVector::ThreeDOFLog(_) => panic!("Requires 3d math vector"),
        }
    }

    #[allow(dead_code)]
    pub fn rotate_2d(&self, angle: &f64) -> Vector2<f64> {
        match self {
            StateVector::OneDOF(_) => {
                //assert_eq!(L, 2);
                let a = self.as_array();
                let mut out = [0.0f64; 2];
                //
                out[0] = a[0] * angle.cos() - a[1] * angle.sin();
                out[1] = a[0] * angle.sin() + a[1] * angle.cos();
                //
                Vector2::new(out[0], out[1])
            }
            StateVector::ThreeDOF(_) => panic!("Requires 2d math vector"),
            StateVector::OneDOFLog(_) => panic!("Requires 2d math vector"),
            StateVector::ThreeDOFLog(_) => panic!("Requires 2d math vector"),
        }
    }
}

impl StateVector {
    pub(crate) fn as_array(&self) -> &[f64] {
        match self {
            StateVector::OneDOF(avec) => avec.as_slice(),
            StateVector::ThreeDOF(avec) => avec.as_slice(),
            StateVector::OneDOFLog(avec) => avec.as_slice(),
            StateVector::ThreeDOFLog(avec) => avec.as_slice(),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::{assert_abs_diff_eq, assert_relative_eq};
    use nalgebra::{SVector, Vector2, Vector3, Vector6};

    #[test]
    fn add_statevector_adds_matching_variants() {
        // 1DOF
        let a1 = StateVector::OneDOF(Vector2::new(1.0, 2.0));
        let b1 = StateVector::OneDOF(Vector2::new(3.0, -4.0));
        assert_eq!(a1 + b1, StateVector::OneDOF(Vector2::new(4.0, -2.0)));

        // 3DOF
        let a3 = StateVector::ThreeDOF(Vector6::new(1.0, 2.0, 3.0, 4.0, 5.0, 6.0));
        let b3 = StateVector::ThreeDOF(Vector6::new(-1.0, 0.0, 10.0, -4.0, 2.0, 0.5));
        assert_eq!(
            a3 + b3,
            StateVector::ThreeDOF(Vector6::new(0.0, 2.0, 13.0, 0.0, 7.0, 6.5))
        );
    }

    #[test]
    #[should_panic(expected = "Invalid addition, mismatching State Vectors.")]
    fn add_panics_on_mismatch() {
        let _ =
            StateVector::OneDOF(Vector2::new(0.0, 0.0)) + StateVector::ThreeDOF(Vector6::zeros());
    }

    #[test]
    fn add_assign_updates_in_place_for_matching_variants() {
        let mut a1 = StateVector::OneDOF(Vector2::new(1.0, 2.0));
        a1 += StateVector::OneDOF(Vector2::new(3.0, -4.0));
        assert_eq!(a1, StateVector::OneDOF(Vector2::new(4.0, -2.0)));

        let mut a3 = StateVector::ThreeDOF(Vector6::new(1.0, 2.0, 3.0, 4.0, 5.0, 6.0));
        a3 += StateVector::ThreeDOF(Vector6::new(-1.0, 0.0, 10.0, -4.0, 2.0, 0.5));
        assert_eq!(
            a3,
            StateVector::ThreeDOF(Vector6::new(0.0, 2.0, 13.0, 0.0, 7.0, 6.5))
        );
    }

    #[test]
    #[should_panic(expected = "Invalid addition, mismatching State Vectors.")]
    fn add_assign_panics_on_mismatch() {
        let mut a = StateVector::OneDOF(Vector2::new(1.0, 2.0));
        a += StateVector::ThreeDOF(Vector6::zeros());
    }

    #[test]
    fn sub_statevector_subtracts_matching_variants() {
        let a1 = StateVector::OneDOF(Vector2::new(5.0, 1.0));
        let b1 = StateVector::OneDOF(Vector2::new(2.0, -3.0));
        assert_eq!(a1 - b1, StateVector::OneDOF(Vector2::new(3.0, 4.0)));

        let a3 = StateVector::ThreeDOF(Vector6::new(1.0, 2.0, 3.0, 4.0, 5.0, 6.0));
        let b3 = StateVector::ThreeDOF(Vector6::new(1.5, 0.0, 10.0, -4.0, 2.0, 0.5));
        assert_eq!(
            a3 - b3,
            StateVector::ThreeDOF(Vector6::new(-0.5, 2.0, -7.0, 8.0, 3.0, 5.5))
        );
    }

    #[test]
    #[should_panic(expected = "Invalid addition, mismatching State Vectors.")]
    fn sub_panics_on_mismatch() {
        let _ =
            StateVector::ThreeDOF(Vector6::zeros()) - StateVector::OneDOF(Vector2::new(0.0, 0.0));
    }

    #[test]
    fn sub_assign_updates_in_place_for_matching_variants() {
        let mut a1 = StateVector::OneDOF(Vector2::new(5.0, 1.0));
        a1 -= StateVector::OneDOF(Vector2::new(2.0, -3.0));
        assert_eq!(a1, StateVector::OneDOF(Vector2::new(3.0, 4.0)));

        let mut a3 = StateVector::ThreeDOF(Vector6::new(1.0, 2.0, 3.0, 4.0, 5.0, 6.0));
        a3 -= StateVector::ThreeDOF(Vector6::new(1.5, 0.0, 10.0, -4.0, 2.0, 0.5));
        assert_eq!(
            a3,
            StateVector::ThreeDOF(Vector6::new(-0.5, 2.0, -7.0, 8.0, 3.0, 5.5))
        );
    }

    #[test]
    #[should_panic(expected = "Invalid addition, mismatching State Vectors.")]
    fn sub_assign_panics_on_mismatch() {
        let mut a = StateVector::ThreeDOF(Vector6::zeros());
        a -= StateVector::OneDOF(Vector2::new(1.0, 2.0));
    }

    #[test]
    fn mul_statevector_componentwise_for_matching_variants() {
        let a1 = StateVector::OneDOF(Vector2::new(2.0, -3.0));
        let b1 = StateVector::OneDOF(Vector2::new(4.0, 10.0));
        assert_eq!(a1 * b1, StateVector::OneDOF(Vector2::new(8.0, -30.0)));

        let a3 = StateVector::ThreeDOF(Vector6::new(1.0, 2.0, 3.0, 4.0, 5.0, 6.0));
        let b3 = StateVector::ThreeDOF(Vector6::new(2.0, -1.0, 0.5, 0.0, 10.0, -2.0));
        assert_eq!(
            a3 * b3,
            StateVector::ThreeDOF(Vector6::new(2.0, -2.0, 1.5, 0.0, 50.0, -12.0))
        );
    }

    #[test]
    #[should_panic(expected = "Invalid addition, mismatching State Vectors.")]
    fn mul_panics_on_mismatch() {
        let _ =
            StateVector::OneDOF(Vector2::new(1.0, 2.0)) * StateVector::ThreeDOF(Vector6::zeros());
    }

    #[test]
    fn mul_assign_componentwise_updates_in_place_for_matching_variants() {
        let mut a1 = StateVector::OneDOF(Vector2::new(2.0, -3.0));
        a1 *= StateVector::OneDOF(Vector2::new(4.0, 10.0));
        assert_eq!(a1, StateVector::OneDOF(Vector2::new(8.0, -30.0)));

        let mut a3 = StateVector::ThreeDOF(Vector6::new(1.0, 2.0, 3.0, 4.0, 5.0, 6.0));
        a3 *= StateVector::ThreeDOF(Vector6::new(2.0, -1.0, 0.5, 0.0, 10.0, -2.0));
        assert_eq!(
            a3,
            StateVector::ThreeDOF(Vector6::new(2.0, -2.0, 1.5, 0.0, 50.0, -12.0))
        );
    }

    #[test]
    #[should_panic(expected = "Invalid addition, mismatching State Vectors.")]
    fn mul_assign_panics_on_mismatch() {
        let mut a = StateVector::OneDOF(Vector2::new(1.0, 2.0));
        a *= StateVector::ThreeDOF(Vector6::zeros());
    }

    #[test]
    fn dot_computes_for_matching_variants() {
        let a1 = StateVector::OneDOF(Vector2::new(1.0, 2.0));
        let b1 = StateVector::OneDOF(Vector2::new(3.0, -4.0));
        assert_relative_eq!(a1.dot(&b1), 1.0 * 3.0 + 2.0 * (-4.0), epsilon = 1e-12);

        let a3 = StateVector::ThreeDOF(Vector6::new(1.0, 2.0, 3.0, 4.0, 5.0, 6.0));
        let b3 = StateVector::ThreeDOF(Vector6::new(-1.0, 0.0, 10.0, -4.0, 2.0, 0.5));
        let expected = -1.0 + 2.0 * 0.0 + 3.0 * 10.0 + 4.0 * (-4.0) + 5.0 * 2.0 + 6.0 * 0.5;

        assert_relative_eq!(a3.dot(&b3), expected, epsilon = 1e-12);
    }

    #[test]
    #[should_panic(expected = "Invalid Dot Product, mismatching State Vectors.")]
    fn dot_panics_on_mismatch() {
        let a = StateVector::OneDOF(Vector2::new(1.0, 2.0));
        let b = StateVector::ThreeDOF(Vector6::zeros());
        let _ = a.dot(&b);
    }

    #[test]
    fn scale_scales_supported_variants() {
        let a1 = StateVector::OneDOF(Vector2::new(1.0, -2.0));
        assert_eq!(a1.scale(2.5), StateVector::OneDOF(Vector2::new(2.5, -5.0)));

        let a3 = StateVector::ThreeDOF(Vector6::new(1.0, 2.0, 3.0, 4.0, -5.0, 6.0));
        assert_eq!(
            a3.scale(-2.0),
            StateVector::ThreeDOF(Vector6::new(-2.0, -4.0, -6.0, -8.0, 10.0, -12.0))
        );
    }

    #[test]
    #[should_panic(expected = "State Vector Scale Impl")]
    fn scale_panics_on_unsupported_variant() {
        let a = StateVector::OneDOFLog(Vector3::new(1.0, 2.0, 3.0));
        let _ = a.scale(2.0);
    }

    #[test]
    fn cross_2d_works_for_1dof() {
        // a.perp(b) == ax*by - ay*bx
        let a = StateVector::OneDOF(Vector2::new(2.0, 3.0));
        let b = Vector2::new(5.0, 7.0);
        let expected = 2.0 * 7.0 - 3.0 * 5.0; // -1
        assert_relative_eq!(a.cross_2d(&b), expected, epsilon = 1e-12);
    }

    #[test]
    #[should_panic(expected = "Requires 2d math vector")]
    fn cross_2d_panics_for_3dof() {
        let a = StateVector::ThreeDOF(Vector6::zeros());
        let _ = a.cross_2d(&Vector2::new(1.0, 0.0));
    }

    #[test]
    #[should_panic(expected = "Requires 3d math vector")]
    fn cross_3d_panics_for_1dof() {
        let a = StateVector::OneDOF(Vector2::new(1.0, 2.0));
        let _ = a.cross_3d(&Vector3::new(1.0, 0.0, 0.0));
    }

    #[test]
    #[should_panic(expected = "Requires 3d math vector")]
    fn cross_3d_panics_for_3dof() {
        let a = StateVector::ThreeDOF(Vector6::zeros());
        let _ = a.cross_3d(&Vector3::new(1.0, 0.0, 0.0));
    }

    #[test]
    fn rotate_2d_rotates_1dof_vector() {
        let v = StateVector::OneDOF(Vector2::new(1.0, 0.0));
        let angle = std::f64::consts::FRAC_PI_2;
        let out = v.rotate_2d(&angle);

        assert_relative_eq!(out[0], 0.0, epsilon = 1e-12);
        assert_relative_eq!(out[1], 1.0, epsilon = 1e-12);
    }

    #[test]
    #[should_panic(expected = "Requires 2d math vector")]
    fn rotate_2d_panics_for_3dof() {
        let v = StateVector::ThreeDOF(Vector6::zeros());
        let angle = 0.1;
        let _ = v.rotate_2d(&angle);
    }
}
