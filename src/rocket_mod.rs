#[derive(Debug, Clone, Copy)]
pub(crate) struct Rocket {
    pub(crate) mass: f64,
    pub(crate) cd: f64,
    pub(crate) area: f64,
    pub(crate) inertia_z: f64,
    pub(crate) stab_margin_dimensional: f64,
}

impl Rocket {
    pub(crate) fn new(mass: f64, cd: f64, area: f64, inertia_z: f64, stab_margin_dimensional: f64) -> Self {
        Self { mass, cd, area, inertia_z, stab_margin_dimensional}
    }
}
