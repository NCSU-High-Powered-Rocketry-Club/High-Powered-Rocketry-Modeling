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
