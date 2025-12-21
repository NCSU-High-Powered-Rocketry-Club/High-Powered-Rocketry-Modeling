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


#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;
    use nalgebra::ComplexField;

    #[test]
    fn test_calc_drag_force() {
        let v = 30.0;
        let cd = 0.75;
        let area = 0.02;

        let expected = -0.5 * density() * v.powi(2) * cd * area;
        assert_relative_eq!(calc_drag_force(v, cd, area), expected, epsilon = 1e-12);

        // Drag should be the same for +v and -v since it uses v^2 (always negative)
        assert_relative_eq!(calc_drag_force(-v, cd, area), expected, epsilon = 1e-12);

        // At zero velocity drag should be zero
        assert_relative_eq!(calc_drag_force(0.0, cd, area), 0.0, epsilon = 0.0);
    }

    #[test]
    fn test_calc_lift_force() {
        let v = 40.0;
        let cl_alpha = 5.0;
        let area = 0.01;

        let alpha_pos = 0.2;
        let alpha_neg = -0.2;

        let expected_pos = 0.5 * density() * v.powi(2) * cl_alpha * alpha_pos * area;
        let expected_neg = 0.5 * density() * v.powi(2) * cl_alpha * alpha_neg * area;

        assert_relative_eq!(calc_lift_force(v, cl_alpha, alpha_pos, area), expected_pos, epsilon = 1e-12);
        assert_relative_eq!(calc_lift_force(v, cl_alpha, alpha_neg, area), expected_neg, epsilon = 1e-12);

        // Lift should be zero at alpha = 0
        assert_relative_eq!(calc_lift_force(v, cl_alpha, 0.0, area), 0.0, epsilon = 0.0);
    }
}
