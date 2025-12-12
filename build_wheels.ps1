# build_wheels.ps1

# Clean up old wheels to ensure a fresh build
$wheelDir = "target/wheels"
if (Test-Path $wheelDir) {
    Write-Host "Cleaning up old wheels..." -ForegroundColor Yellow
    Remove-Item -Path "$wheelDir\*" -Recurse -Force
}

# Define the Python versions you want to build for
# 3.13 is the current stable version.
# 3.14 is the target version. 3.14t is the free-threaded version.
$versions = @("3.13", "3.14", "3.14t")

# --- Windows x64 (Native) ---
Write-Host "Building for Windows x64..." -ForegroundColor Green
foreach ($v in $versions) {
    Write-Host "  Building for Python $v..."
    # We use 'uv run' to execute maturin from the project dependencies
    uv run maturin build --release -i $v --compatibility pypi
}

# --- Linux x86_64 (manylinux_2_34) ---
Write-Host "Building for Linux x86_64 (manylinux_2_34)..." -ForegroundColor Green
foreach ($v in $versions) {
    Write-Host "  Building for Python $v..."
    # Using Zig to cross-compile to Linux x86_64
    uv run maturin build --release -i $v --compatibility manylinux_2_34 --target x86_64-unknown-linux-gnu --zig
}

# --- Linux aarch64 (manylinux_2_31) ---
Write-Host "Building for Linux aarch64 (manylinux_2_31)..." -ForegroundColor Green
foreach ($v in $versions) {
    Write-Host "  Building for Python $v..."
    # Using Zig to cross-compile to Linux aarch64 (ARM64)
    uv run maturin build --release -i $v --compatibility manylinux_2_31 --target aarch64-unknown-linux-gnu --zig
}

Write-Host "`nBuild complete! Wheels are located in target/wheels/" -ForegroundColor Cyan
