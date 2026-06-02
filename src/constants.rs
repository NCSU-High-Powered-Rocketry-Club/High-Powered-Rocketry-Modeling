pub mod ode_constants {
    /// The default timestep for the ODE solver in seconds
    pub const DEFAULT_TIMESTEP : f64 = 0.01;
    /// The minimum and maximum timestep limits for the adaptive ODE solver in seconds
    pub const DEFAULT_MIN_TIMESTEP: f64 = 1e-6;
    /// The maximum timestep limit for the adaptive ODE solver in seconds
    pub const DEFAULT_MAX_TIMESTEP: f64 = 0.1;
    /// The default error tolerance for the adaptive ODE solver
    pub const DEFAULT_TOLERANCE: f64 = 1e-2;
    /// The safety factor used in the adaptive ODE solver to prevent overshooting (between 0 and 1)
    pub const SAFETY_FACTOR: f64 = 0.5;
}

pub mod physics_constants {
    /// Acceleration due to gravity at sea level in m/s^2
    pub const GRAVITY_M_S_2: f64 = 9.80665;
    /// ISA air density at sea level in kg/m^3
    pub const SEA_LEVEL_AIR_DENSITY_KG_M_3: f64 = 1.225;
}

pub mod simulation_constants {
    /// Number of data columns in SimulationData (matches StateVector length)
    pub const DATA_LENGTH: usize = 18; 
    /// The initial number of rows to pre-allocate in SimulationData
    pub const INITIAL_DATA_CAPACITY: usize = 1000;
    /// The velocity threshold (m/s) below which the rocket is considered to have reached apogee
    pub const APOGEE_VELOCITY_THRESHOLD_M_S: f64 = 0.5;
}