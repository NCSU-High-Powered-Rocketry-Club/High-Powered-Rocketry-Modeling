use nalgebra::{SVector, Vector2, Vector6, Vector3};

use std::ops::{Add, AddAssign, Mul, MulAssign, Sub, SubAssign};

#[derive(Debug, Clone, Copy)]
pub(crate) enum StateVector {
    // Data type which represents an actual vector(rust::array) of the state space for a given model
    __1DOF(Vector2<f64>),
    __1DLOG(Vector3<f64>),
    __3DOF(Vector6<f64>),
    __3DLOG(SVector<f64, 9>),
}

impl Add for StateVector {
    type Output = Self;

    fn add(self, rhs: Self) -> Self::Output {
        match (self, rhs) {
            (StateVector::__1DOF(avec), StateVector::__1DOF(bvec)) => {
                StateVector::__1DOF(avec + bvec)
            }
            (StateVector::__3DOF(avec), StateVector::__3DOF(bvec)) => {
                StateVector::__3DOF(avec + bvec)
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
            (StateVector::__1DOF(mut avec), StateVector::__1DOF(bvec)) => avec += bvec,
            (StateVector::__3DOF(mut avec), StateVector::__3DOF(bvec)) => avec += bvec,
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
            (StateVector::__1DOF(avec), StateVector::__1DOF(bvec)) => {
                StateVector::__1DOF(avec - bvec)
            }
            (StateVector::__3DOF(avec), StateVector::__3DOF(bvec)) => {
                StateVector::__3DOF(avec - bvec)
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
            (StateVector::__1DOF(mut avec), StateVector::__1DOF(bvec)) => avec -= bvec,
            (StateVector::__3DOF(mut avec), StateVector::__3DOF(bvec)) => avec -= bvec,
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
            (StateVector::__1DOF(avec), StateVector::__1DOF(bvec)) => {
                StateVector::__1DOF(avec.component_mul(&bvec))
            }
            (StateVector::__3DOF(avec), StateVector::__3DOF(bvec)) => {
                StateVector::__3DOF(avec.component_mul(&bvec))
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
            (StateVector::__1DOF(mut avec), StateVector::__1DOF(bvec)) => avec.component_mul_assign(&bvec),
            (StateVector::__3DOF(mut avec), StateVector::__3DOF(bvec)) => avec.component_mul_assign(&bvec),
            _ => {
                panic!("Invalid addition, mismatching State Vectors.")
            }
        }
    }
}

impl StateVector {
    pub fn dot(&self, b: &Self) -> f64 {
        match (self, b) {
            (StateVector::__1DOF(avec), StateVector::__1DOF(bvec)) => avec.dot(&bvec),
            (StateVector::__3DOF(avec), StateVector::__3DOF(bvec)) => avec.dot(&bvec),
            _ => {
                panic!("Invalid Dot Product, mismatching State Vectors.")
            }
        }
    }
    pub fn scale(&self, k: f64) -> Self {
        match self {
            StateVector::__1DOF(avec) => StateVector::__1DOF(avec * k),
            StateVector::__3DOF(avec) => StateVector::__3DOF(avec * k),
            _ => {
                panic!("State Vector Scale Impl")
            }
        }
    }
    pub fn cross_2d(&self, in2: &Vector2<f64>) -> f64 {
        match self {
            StateVector::__1DOF(avec) => avec.perp(in2),
            StateVector::__3DOF(avec) => panic!("Requires 2d math vector"),
            _ => {
                panic!("Requires 2d math vector")
            }
        }
    }
    pub fn cross_3d(&self, in2: &Vector3<f64>) -> Vector3<f64> {
        match self {
            StateVector::__1DOF(avec) => panic!("Requires 3d math vector"),
            StateVector::__3DOF(avec) => panic!("Requires 3d math vector"),
            _ => {
                panic!("Requires 3d math vector")
            }
        }
    } 
    pub fn rotate_2d(&self, angle: &f64) -> Vector2<f64> {
        match self {
            StateVector::__1DOF(avec) => {
                //assert_eq!(L, 2);
                let a = self.as_array();
                let mut out = [0.0f64; 2];
                //
                out[0] = a[0] * angle.cos() - a[1] * angle.sin();
                out[1] = a[0] * angle.sin() + a[1] * angle.cos();
                //
                Vector2::new(out[0], out[1])
            },
            StateVector::__3DOF(avec) => panic!("Requires 2d math vector"),
            _ => {
                panic!("Requires 2d math vector")
            }
        }
    }
}

impl StateVector {
    pub(crate) fn as_array(&self) -> &[f64] {
        match self {
            StateVector::__1DOF(avec) => avec.as_slice(),
            StateVector::__3DOF(avec) => avec.as_slice(),
            StateVector::__1DLOG(avec) => avec.as_slice(),
            StateVector::__3DLOG(avec) => avec.as_slice(),
            _ => {
                panic!("Invalid conversion to array, mismatching or unsupported State Vector type.")
            }
        }
    }
}
