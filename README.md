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

# Run the Demo
uv run examples/demo.py
```
<!-- 
## Publishing
Releases are automated via GitHub Actions. To publish a new version:
1. Update version in `Cargo.toml`.
2. Merge changes to `main`.
3. Push a tag (e.g., `v0.1.0`).

```bash
git tag v0.1.0
git push origin v0.1.0 -->
```

# Building and Publishing Manually

## Prerequisites
Ensure you have `uv` installed. The project is configured to use `maturin` and `zig` (for cross-compilation) via `uv`.

## Building Wheels
To build wheels for Windows (x64), Linux (x86_64), and Linux (aarch64) for supported Python versions:

```powershell
.\build_wheels.ps1
```

This script will:
1. Clean the `target/wheels` directory.
2. Build Windows wheels.
3. Cross-compile Linux wheels using Zig.

## Publishing to PyPI
You will need a PyPI API token.

1. **Check Version**: Ensure `Cargo.toml` version is updated.
2. **Upload**:
   ```bash
   uv run maturin upload target/wheels/*
   ```
   - **Username**: `__token__`
   - **Password**: Your PyPI API token (starts with `pypi-`).

