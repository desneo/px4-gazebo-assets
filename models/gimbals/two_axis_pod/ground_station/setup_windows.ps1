[CmdletBinding()]
param(
    [switch]$InstallPython
)

$ErrorActionPreference = "Stop"
$GroundStationDir = $PSScriptRoot
$RepoRoot = (Resolve-Path (Join-Path $GroundStationDir "..\..\..\..")).Path
$VenvDir = Join-Path $RepoRoot ".venv-windows"
$VenvPython = Join-Path $VenvDir "Scripts\python.exe"

function Find-Python {
    $Candidates = @(
        (Join-Path $env:LOCALAPPDATA "Programs\Python\Python312\python.exe"),
        (Join-Path $env:LOCALAPPDATA "Programs\Python\Python313\python.exe")
    )
    foreach ($Candidate in $Candidates) {
        if (Test-Path -LiteralPath $Candidate) { return $Candidate }
    }
    $Command = Get-Command python.exe -ErrorAction SilentlyContinue
    if ($Command -and $Command.Source -notlike "*\WindowsApps\*") {
        return $Command.Source
    }
    return $null
}

$Python = Find-Python
if (-not $Python -and $InstallPython) {
    Write-Host "Python not found. Installing official Python 3.12 for the current user..."
    winget install --id Python.Python.3.12 --exact --scope user `
        --accept-package-agreements --accept-source-agreements
    if ($LASTEXITCODE -ne 0) { throw "Python installation failed with exit code $LASTEXITCODE" }
    $Python = Find-Python
}
if (-not $Python) {
    throw "Python not found. Run setup_windows.ps1 -InstallPython or install Python 3.12."
}

if (-not (Test-Path -LiteralPath $VenvPython)) {
    Write-Host "Creating project virtual environment: $VenvDir"
    & $Python -m venv $VenvDir
    if ($LASTEXITCODE -ne 0) { throw "Failed to create the virtual environment" }
}

& $VenvPython -m pip install --upgrade pip
if ($LASTEXITCODE -ne 0) { throw "Failed to update pip" }
& $VenvPython -m pip install -r (Join-Path $GroundStationDir "requirements-windows.txt")
if ($LASTEXITCODE -ne 0) { throw "Failed to install Windows ground station dependencies" }
& $VenvPython -c "import cv2, numpy, PIL; print('Windows ground station environment ready:', cv2.__version__)"
if ($LASTEXITCODE -ne 0) { throw "Dependency import check failed" }

Write-Host "Done. PyGObject, GStreamer, and system codecs were not installed."
Write-Host "Start with: .\models\gimbals\two_axis_pod\ground_station\run_windows.ps1"
