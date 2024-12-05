// Doing operations on math vectors of unknown size and floating point type
use super::{Max, Norm, Sum};
use std::ops::{Add, AddAssign, Deref, Mul, MulAssign, Sub, SubAssign};
use std::os::unix::fs::lchown;

pub(crate) struct MathVector<const L: usize> {
    pub(crate) len : usize,
    pub(crate) data: [f64; L],
}

// Associated Method Implimentations
impl<const L: usize> MathVector<L> {
    //
    pub(crate) fn new(data: [f64; L]) -> Self {
        Self {
            len: data.len(),
            data,
        }
    }
    //
    pub(crate) fn copy(&self) -> Self {
        Self { len: self.len, data: self.data.clone() }
    }
    //
    pub(crate) fn dot(a: &Self, b: &Self) -> f64 {
        let aa = a.copy();
        let bb = b.copy();
        (aa * bb).sum()
    }
    //
    pub(crate) fn scale(&self, b: f64) -> Self {
        let mut out = [0.0f64; L];
        //
        for i in 0..L {
            out[i] = self.data[i] * b;
        }
        //
        Self { len: L, data: out }
    }
    //
    pub(crate) fn cross_2d(in1: &Self, in2: &Self) -> f64 {
        assert_eq!(L, 2);
        let a = in1.data;
        let b = in2.data;
        a[0] * b[1] - a[1] * b[0]
    }
    //
    pub(crate) fn cross_3d(in1: &Self, in2: &Self) -> Self {
        assert_eq!(L, 3);
        let a = in1.data;
        let b = in2.data;
        let mut out = [0.0f64; L];

        out[0] = a[1] * b[2] - a[2] * b[1];
        out[1] = a[2] * b[0] - a[0] * b[2];
        out[2] = a[0] * b[1] - a[1] * b[0];

        Self { len: L, data: out }
    }
    //
    pub(crate) fn rotate_2d(&self, angle: &f64) -> Self {
        assert_eq!(L, 2);
        let a = self.data;
        let mut out = [0.0f64; L];
        //
        out[0] = a[0] * angle.cos() - a[1] * angle.sin();
        out[1] = a[0] * angle.sin() + a[1] * angle.cos();
        //
        Self{len: L, data: out}
    }
}

// Custom Trait Implimentations
impl<const L: usize> Sum<f64> for MathVector<L> {
    fn sum(self: &Self) -> f64 {
        let mut out = 0.0f64;
        //
        for i in 0..L {
            out += self.data[i];
        }
        out
    }
}
impl<const L: usize> Max<f64> for MathVector<L> {
    fn max(self: &Self) -> f64 {
        let mut out = 0.0f64;
        //
        for i in 0..L {
            out = if self.data[i] > out { self.data[i] } else { out };
        }
        out
    }
    fn max_mag(self: &Self) -> f64 {
        let mut out = 0.0f64;
        //
        for i in 0..L {
            out = if  self.data[i].abs() > out {  self.data[i].abs() } else { out };
        }
        out
    }
}
impl<const L: usize> Norm<f64> for MathVector<L> {
    fn norm(self: &Self) -> f64 {
        self.norm_2()
    }
    fn norm_1(self: &Self) -> f64 {
        self.sum()
    }
    fn norm_2(self: &Self) -> f64 {
        let mut out = 0.0f64;
        //
        for i in 0..L {
            out +=  self.data[i] * self.data[i];
        }
        //
        out.sqrt()
    }
    fn norm_infinity(self: &Self) -> f64 {
        self.max_mag()
    }
}

//Standard Trait Implimentations
impl<const L: usize> Add for MathVector<L> {
    type Output = Self;
    fn add(self, other: Self) -> Self::Output {
        let mut out = [0.0f64; L];
        //
        for i in 0..L {
            out[i] =  self.data[i] + other.data[i];
        }
        //
        Self { len: L, data: out }
    }
}
impl<const L: usize> AddAssign for MathVector<L> {
    fn add_assign(&mut self, other: Self) -> () {
        for i in 0..L {
            self.data[i] += other.data[i];
        }
    }
}
impl<const L: usize> Mul for MathVector<L> {
    type Output = Self;

    fn mul(self, other: Self) -> Self::Output {
        let mut out = [0.0f64; L];
        //
        for i in 0..L {
            out[i] = self.data[i] * other.data[i];
        }
        //
        Self { len: L, data: out }
    }
}
impl<const L: usize> MulAssign for MathVector<L> {
    fn mul_assign(&mut self, other: Self) -> () {
        //
        for i in 0..L {
            self.data[i] *= other.data[i];
        }
    }
}
