pub(crate) struct Rocket {
    pub(crate) mass: f64,
    pub(crate) cd: f64,
    pub(crate) area: f64,
}

impl Rocket {
    pub(crate) fn new(mass: f64, cd: f64, area: f64) -> Rocket {
        Rocket { mass, cd, area }
    }

    pub(crate) fn copy(&self) -> Rocket {
        Rocket {
            mass: self.mass,
            cd: self.cd,
            area: self.area,
        }
    }
}
