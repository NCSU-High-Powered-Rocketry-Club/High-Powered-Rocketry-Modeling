# Tools for simulating and modeling rockets.

A Python library written in Rust for quick and efficient modeling and analysis of rocket data.

![image](graphics/HPRM.png)

Currently it has a 1D-1Dof, and 2D-3Dof model formats, with a 3D-6Dof model format planned. The model parameters are input conditions; planned functionality is to be able to train a model to data.

The long-term vision of this project is to be a toolbox for testing out different rocket models with data-fitted and uncertainty-estimated parameters. The primary intended use case is to be a means to do on-the-ground modeling work relavent to past and future rockets; but, it will be performant enough that it could be used in-the-loop in some launch vehicle applications.

# Quick Start

## Install Rust
Follow [this guide](https://www.geeksforgeeks.org/installation-guide/how-to-setup-rust-in-vscode/) to get Rust setup in VS Code, or figure out how to set it up in your dev environment of choice.

## Install uv
We use `uv` to handle the python side of this project. It's like pip but better, install it [here](https://docs.astral.sh/uv/getting-started/installation/).

## Python Bindings (PyPI)

We need to to first build wheels for each platform, right now the workflow is to do this locally
and then upload to PyPI. At the minimum, we build for Linux x86_64 and aarch64 for python versions
3.10+, including free threaded wheels.

1. We should always bump the version in `firm_python/Cargo.toml`

2. Build the wheels:

```bash
# If you're on linux
./compile.sh
# If you're on windows
.\compile.ps1
```

This will create wheels in the `target/wheels` directory, for Python versions 3.10 to 3.14,
for both x86_64 and aarch64.

3. Make sure you also have a source distribution:

```bash
uv run maturin sdist
```

4. We will use `uv` to publish these wheels to PyPI. Make sure you are part of the HPRC
   organization on PyPI, so you have access to the project and can publish new versions.

```bash
uv publish target/wheels/*
```
