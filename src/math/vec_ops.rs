// Doing operations on math vectors of unknown size and floating point type
use super::{Max, Norm, Sum};
use std::ops::{Add, AddAssign, Deref, Div, Mul, MulAssign, Sub, SubAssign};

#[derive(Debug, Clone, Copy)]
pub(crate) struct MathVector<const L: usize> {
    pub(crate) len: usize,
    pub(crate) data: [f64; L],
}

pub(crate) trait VectorOperations: Add + AddAssign + Sub + SubAssign + Mul + MulAssign + Sized {
    fn dot(&self, b: &Self) -> f64;
    fn scale(&self, k: f64) -> Self;
    fn cross_2d(&self, in2: &MathVector<2>) -> f64;
    fn cross_3d(&self, in2: &MathVector<3>) -> MathVector<3>;
    fn rotate_2d(&self, angle: &f64) -> MathVector<2>;
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
        Self {
            len: self.len,
            data: self.data.clone(),
        }
    }
    //\
}
impl<const L: usize> VectorOperations for MathVector<L> {
    fn dot(&self, b: &Self) -> f64 {
        let aa = self.copy();
        let bb = b.copy();
        (aa * bb).sum()
    }
    //
    fn scale(&self, b: f64) -> Self {
        let mut out = [0.0f64; L];
        //
        for i in 0..L {
            out[i] = self.data[i] * b;
        }
        //
        Self { len: L, data: out }
    }
    //
    fn cross_2d(&self, in2: &MathVector<2>) -> f64 {
        assert_eq!(L, 2);
        let a = self.data;
        let b = in2.data;
        a[0] * b[1] - a[1] * b[0]
    }
    //
    fn cross_3d(&self, in2: &MathVector<3>) -> MathVector<3> {
        assert_eq!(L, 3);
        let a = self.data;
        let b = in2.data;
        let mut out = [0.0f64; 3];

        out[0] = a[1] * b[2] - a[2] * b[1];
        out[1] = a[2] * b[0] - a[0] * b[2];
        out[2] = a[0] * b[1] - a[1] * b[0];

        MathVector { len: 3, data: out }
    }
    //
    fn rotate_2d(&self, angle: &f64) -> MathVector<2> {
        assert_eq!(L, 2);
        let a = self.data;
        let mut out = [0.0f64; 2];
        //
        out[0] = a[0] * angle.cos() - a[1] * angle.sin();
        out[1] = a[0] * angle.sin() + a[1] * angle.cos();
        //
        MathVector::<2> { len: 2, data: out }
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
            out = if self.data[i] > out {
                self.data[i]
            } else {
                out
            };
        }
        out
    }
    fn max_mag(self: &Self) -> f64 {
        let mut out = 0.0f64;
        //
        for i in 0..L {
            out = if self.data[i].abs() > out {
                self.data[i].abs()
            } else {
                out
            };
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
            out += self.data[i] * self.data[i];
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
            out[i] = self.data[i] + other.data[i];
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
impl<const L: usize> Sub for MathVector<L> {
    type Output = Self;
    fn sub(self, other: Self) -> Self::Output {
        let mut out = [0.0f64; L];
        //
        for i in 0..L {
            out[i] = self.data[i] + other.data[i];
        }
        //
        Self { len: L, data: out }
    }
}
impl<const L: usize> SubAssign for MathVector<L> {
    fn sub_assign(&mut self, other: Self) -> () {
        for i in 0..L {
            self.data[i] -= other.data[i];
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
impl<const L: usize> Deref for MathVector<L> {
    type Target = [f64; L];

    fn deref(&self) -> &Self::Target {
        &self.data
    }
}
