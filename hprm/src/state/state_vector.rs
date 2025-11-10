use crate::math::vec_ops::{MathVector, VectorOperations};
use std::ops::{Add, AddAssign, Mul, MulAssign, Sub, SubAssign};

#[derive(Debug, Clone, Copy)]
pub(crate) enum StateVector {
    // Data type which represents an actual vector(rust::array) of the state space for a given model
    __1DOF(MathVector<2>),
    __1DLOG(MathVector<3>),
    __3DOF(MathVector<6>),
    __3DLOG(MathVector<9>),
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
                StateVector::__1DOF(avec * bvec)
            }
            (StateVector::__3DOF(avec), StateVector::__3DOF(bvec)) => {
                StateVector::__3DOF(avec * bvec)
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
            (StateVector::__1DOF(mut avec), StateVector::__1DOF(bvec)) => avec *= bvec,
            (StateVector::__3DOF(mut avec), StateVector::__3DOF(bvec)) => avec *= bvec,
            _ => {
                panic!("Invalid addition, mismatching State Vectors.")
            }
        }
    }
}

impl VectorOperations for StateVector {
    fn dot(&self, b: &Self) -> f64 {
        match (self, b) {
            (StateVector::__1DOF(avec), StateVector::__1DOF(bvec)) => avec.dot(bvec),
            (StateVector::__3DOF(avec), StateVector::__3DOF(bvec)) => avec.dot(bvec),
            _ => {
                panic!("Invalid Dot Product, mismatching State Vectors.")
            }
        }
    }
    fn scale(&self, k: f64) -> Self {
        match self {
            StateVector::__1DOF(avec) => StateVector::__1DOF(avec.scale(k)),
            StateVector::__3DOF(avec) => StateVector::__3DOF(avec.scale(k)),
            _ => {
                panic!("State Vectore Scale Impl")
            }
        }
    }
    fn cross_2d(&self, in2: &MathVector<2>) -> f64 {
        match self {
            StateVector::__1DOF(avec) => avec.cross_2d(in2),
            StateVector::__3DOF(avec) => panic!("Requires 2d math vector"),
            _ => {
                panic!("cRequires 2d math vector")
            }
        }
    }
    fn cross_3d(&self, in2: &MathVector<3>) -> MathVector<3> {
        match self {
            StateVector::__1DOF(avec) => panic!("Requires 3d math vector"),
            StateVector::__3DOF(avec) => panic!("Requires 3d math vector"),
            _ => {
                panic!("Requires 3d math vector")
            }
        }
    } 
    fn rotate_2d(&self, angle: &f64) -> MathVector<2> {
        match self {
            StateVector::__1DOF(avec) => avec.rotate_2d(angle),
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
            StateVector::__1DOF(avec) => avec.data.as_slice(),
            StateVector::__3DOF(avec) => avec.data.as_slice(),
            StateVector::__1DLOG(avec) => avec.data.as_slice(),
            StateVector::__3DLOG(avec) => avec.data.as_slice(),
            _ => {
                panic!("Invalid Dot Product, mismatching State Vectors.")
            }
        }
    }
}
