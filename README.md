# Tools for simulating and modeling rockets.

A Python library written in Rust for quick and efficient modeling and analysis of rocket data.

Currently it has a 1D-1Dof, and 2D-3Dof model formats, with a 3D-6Dof model format planned. The model parameters are input conditions; planned functionality is to be able to train a model to data.

The long-term vision of this project is to be a toolbox for testing out different rocket models with data-fitted and uncertainty-estimated parameters. The primary intended use case is to be a means to do on-the-ground modeling work relavent to past and future rockets; but, it will be performant enough that it could be used in-the-loop in some launch vehicle applications.

# Quick Start

## Install Rust
Follow [this guide](https://www.geeksforgeeks.org/installation-guide/how-to-setup-rust-in-vscode/) to get Rust setup in VS Code, or figure out how to set it up in your dev environment of choice.

## Install uv
We use `uv` to handle the python side of this project. It's like pip but better, install it [here](https://docs.astral.sh/uv/getting-started/installation/).

##### uv Workflow:
```bash
# First time setup
uv sync --all-extras --inexact

# Build Rust extension
uv run maturin develop --uv

# Run the Demo
uv run examples/demo.py
```

