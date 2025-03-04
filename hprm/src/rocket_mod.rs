use pyo3::prelude::*;

#[pyclass]#[derive(Debug, Clone, Copy)]
pub(crate) struct Rocket {
    pub(crate) mass: f64,
    pub(crate) cd: f64,
    pub(crate) area_drag: f64,
    pub(crate) area_lift: f64,
    pub(crate) inertia_z: f64,
    pub(crate) stab_margin_dimensional: f64,
    pub(crate) cl_a: f64,
}

#[pymethods]
impl Rocket {
    #[new]
    pub(crate) fn new(
        mass: f64,
        cd: f64,
        area_drag: f64,
        area_lift: f64,
        inertia_z: f64,
        stab_margin_dimensional: f64,
        cl_a: f64,
    ) -> Self {
        Self {
            mass,
            cd,
            area_drag,
            area_lift,
            inertia_z,
            stab_margin_dimensional,
            cl_a,
        }
    }
}
