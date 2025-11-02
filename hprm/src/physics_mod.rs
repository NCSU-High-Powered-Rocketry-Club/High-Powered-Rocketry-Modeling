pub(crate) fn density() -> f64 {
    1.224
}

pub(crate) fn gravity() -> f64 {
    -9.8
}

pub(crate) fn calc_drag_force(velocity: f64, cd: f64, area: f64) -> f64 {
    let rho = density();
    -0.5 * rho * velocity.powi(2) * cd * area
}

pub(crate) fn calc_lift_force(velocity: f64, cl_alpha: f64, alpha: f64, area: f64) -> f64 {
    let rho = density();
    0.5 * rho * velocity.powi(2) * cl_alpha * alpha * area
}

//pub(crate) fn calc_moment_2d(force: [f64; 2], moment_arm: [f64; 2]) -> f64 {
//    0.0 * force[0] * moment_arm[0]
//}
