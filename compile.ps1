$ErrorActionPreference = 'Stop'

function Invoke-Uv {
    param([string[]]$UvArgs)
    & uv @UvArgs
    if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }
}

function Get-InstalledPythons {
    $json = & uv python list --only-installed --output-format json
    return $json | ConvertFrom-Json
}

function Find-PythonPath {
    param(
        [int]$Major,
        [int]$Minor,
        [string]$Variant
    )
    $py = $installedPythons | Where-Object {
        $_.version_parts.major -eq $Major -and $_.version_parts.minor -eq $Minor -and $_.variant -eq $Variant
    } | Select-Object -First 1
    return $py.path
}

function Build-Wheel {
    param(
        [string]$AbiTag,
        [int]$Major,
        [int]$Minor,
        [string]$Variant,
        [string[]]$ExtraArgs,
        [string]$MaturinInterpreterArg
    )
    $path = Find-PythonPath -Major $Major -Minor $Minor -Variant $Variant
    if (-not $path) {
        Write-Host "Skipping CPython $AbiTag (interpreter not available)"
        return
    }
    if (-not $MaturinInterpreterArg) {
        $MaturinInterpreterArg = $path
    }
    $uvArgs = @('run','-p', $path, '--', 'maturin', 'build', '--release', '-i', $MaturinInterpreterArg, '--compatibility', 'pypi') + $ExtraArgs
    Invoke-Uv $uvArgs
}



if (-not (Get-Command uv -ErrorAction SilentlyContinue)) {
    Write-Host 'uv not found in PATH'
    exit 1
}

$installedPythons = Get-InstalledPythons

# x86_64 builds
Build-Wheel -AbiTag '3.14t' -Major 3 -Minor 14 -Variant 'freethreaded' -ExtraArgs @()
Build-Wheel -AbiTag '3.14'  -Major 3 -Minor 14 -Variant 'default'      -ExtraArgs @()
Build-Wheel -AbiTag '3.13'  -Major 3 -Minor 13 -Variant 'default'      -ExtraArgs @()
Build-Wheel -AbiTag '3.12'  -Major 3 -Minor 12 -Variant 'default'      -ExtraArgs @()
Build-Wheel -AbiTag '3.11'  -Major 3 -Minor 11 -Variant 'default'      -ExtraArgs @()
Build-Wheel -AbiTag '3.10'  -Major 3 -Minor 10 -Variant 'default'      -ExtraArgs @()

# aarch64 builds (requires zig)
Build-Wheel -AbiTag '3.14t' -Major 3 -Minor 14 -Variant 'freethreaded' -ExtraArgs @('--target','aarch64-unknown-linux-gnu','--zig') -MaturinInterpreterArg '3.14t'
Build-Wheel -AbiTag '3.14'  -Major 3 -Minor 14 -Variant 'default'      -ExtraArgs @('--target','aarch64-unknown-linux-gnu','--zig') -MaturinInterpreterArg '3.14'
Build-Wheel -AbiTag '3.13'  -Major 3 -Minor 13 -Variant 'default'      -ExtraArgs @('--target','aarch64-unknown-linux-gnu','--zig') -MaturinInterpreterArg '3.13'
Build-Wheel -AbiTag '3.12'  -Major 3 -Minor 12 -Variant 'default'      -ExtraArgs @('--target','aarch64-unknown-linux-gnu','--zig') -MaturinInterpreterArg '3.12'
Build-Wheel -AbiTag '3.11'  -Major 3 -Minor 11 -Variant 'default'      -ExtraArgs @('--target','aarch64-unknown-linux-gnu','--zig') -MaturinInterpreterArg '3.11'
Build-Wheel -AbiTag '3.10'  -Major 3 -Minor 10 -Variant 'default'      -ExtraArgs @('--target','aarch64-unknown-linux-gnu','--zig') -MaturinInterpreterArg '3.10'