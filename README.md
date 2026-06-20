# High Powered Rocket Modeling (HPRM)

HPRM is a Python library built on a very fast Rust backend designed for the quick, lightweight modeling and analysis of rocket trajectories. Its primary purpose is to be integrated directly into rocket flight computers to continuously run predictive simulations mid-flight (for example, we use it for the control logic in our [AirbrakesV2](https://github.com/NCSU-High-Powered-Rocketry-Club/AirbrakesV2) project).

### Quick Start: Predicting Apogee
Running a prediction is as simple as defining your rocket's physical parameters and passing your initial launch conditions:

```python
from hprm import Rocket, OdeMethod

# Define your rocket's physical characteristics
rocket = Rocket(
    mass=17.0,
    cd=0.39,
    area_drag=0.0182,
    area_lift=0.0182,
    moment_of_inertia=11.5,
    stab_margin_dimensional=0.5,
    cl_a=0.2,
)

# Very fast apogee prediction using a 1DOF model
apogee = rocket.predict_apogee_1dof(
    initial_height=0.0,
    initial_velocity=150.0,
    integration_method=OdeMethod.RK45
)
```

### Examples Roadmap
To get familiar with the library, we recommend checking out the provided example scripts (located in the `examples/` directory) in this specific order:

1. **`predict_apogee.py`**: The absolute basics—defining a rocket and getting a single peak altitude number.
2. **`simulate_flight.py`**: Running a full simulation, extracting the state arrays, and plotting the 1DOF/3DOF trajectories.
3. **`compare_methods.py`**: A visual look at how different ODE solvers (RK45, RK3, Euler) impact the simulation path.
4. **`compare_rocket_params.py`**: Co-plotting the effects of changing mass, drag, stability margins, MOI, and lift.
5. **`adaptive_timestep_demo.py`**: Showing library execution speeds by tweaking the adaptive solver's error tolerances.

---

## Developing for the Library

The long-term vision of this project is to be a toolbox for testing out different rocket models with data-fitted and uncertainty-estimated parameters. While currently supporting 1D-1DoF and 2D-3DoF formats (with a 3D-6DoF format planned), future functionality will allow training the model directly to flight data.

### Install Rust
Follow [this guide](https://www.geeksforgeeks.org/installation-guide/how-to-setup-rust-in-vscode/) to get Rust setup in VS Code, or figure out how to set it up in your dev environment of choice.

### Install uv
We use `uv` to handle the Python side of this project. It's like pip but much faster. Install it [here](https://docs.astral.sh/uv/getting-started/installation/).

Once you've installed uv, run `uv sync --all-extras` in the project root to install all the Python dependencies.

### Python Bindings (PyPI)
To publish this on PyPi, you need to first build wheels for each platform. Right now the workflow is to do this locally and then upload to PyPI. At a minimum, we build for Linux x86_64 and aarch64 for Python versions 3.10+, including free threaded wheels.

1. Always bump the version in `firm_python/Cargo.toml` before publishing.
2. Build the wheels using the provided scripts:

```bash
# If you're on Linux
./compile.sh

# If you're on Windows
.\compile.ps1
```

*This will create wheels in the `target/wheels` directory for Python versions 3.10 to 3.14, for both x86_64 and aarch64.*

3. Make sure you also generate a source distribution:

```bash
uv run maturin sdist
```

4. Use `uv` to publish these wheels to PyPI. Make sure you are part of the HPRC organization on PyPI so you have access to the project and can publish new versions.

```bash
uv publish target/wheels/*
```