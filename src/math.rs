// Module file which contains methods relating to solving ordinary differential equations.
pub(crate) mod ode;

//Module file which contains functions to do operations on (math)vectors, represented as

pub trait Sum<T> {
    // Sum of all elements of a vector
    fn sum(self: &Self) -> T;
}
pub trait Max<T> {
    // Maximum value in an vector
    fn max(self: &Self) -> T;
    // Value with the highest magnitude in the Vector
    fn max_mag(self: &Self) -> T;
}
pub trait Norm<T> {
    //Use as the standard norm for whatever object you're implimenting. Can be one of the other fns
    // or whatever else it needs to be
    fn norm(self: &Self) -> T;

    //Some standard norms you should impliment when applicable
    fn norm_1(self: &Self) -> T; //        Sum of all values (impl using Sum trait)
    fn norm_2(self: &Self) -> T; //        Euclidean norm/distance
                                 //fn norm_n(self: &Self) -> T {f64::NAN as T} //        N-degree norm (literally no reason to impliment this, idk why I'm putting it here)
    fn norm_infinity(self: &Self) -> T; // Maximum magnitude value (impl using max_mag trait)
}
