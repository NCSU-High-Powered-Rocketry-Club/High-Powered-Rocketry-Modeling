use pyo3::prelude::*;

#[pyclass(dict,get_all,set_all)]
#[derive(Debug, Clone, Copy)]
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
    // For `__repr__` we want to return a string that Python code could use to recreate
    // the `Name`, like `Name(5)` for example.
    fn __repr__(&self) -> String {
        // We use the `format!` macro to create a string. Its first argument is a
        // format string, followed by any number of parameters which replace the
        // `{}`'s in the format string.
        //
        // /// Add more details here
        "Rocket_Structure".to_string()
    }
    // `__str__` is generally used to create an "informal" representation, so we
    // just forward to `ToString` trait implementation to print a bare number.
    fn __str__(&self) -> String {
        "Rocket_Structure".to_string()
    }
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
