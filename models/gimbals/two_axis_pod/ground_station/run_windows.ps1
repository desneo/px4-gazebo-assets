[CmdletBinding()]
param(
    [Parameter(ValueFromRemainingArguments = $true)]
    [string[]]$GroundStationArgs
)

$ErrorActionPreference = "Stop"
$RepoRoot = (Resolve-Path (Join-Path $PSScriptRoot "..\..\..\..")).Path
$Python = Join-Path $RepoRoot ".venv-windows\Scripts\python.exe"
if (-not (Test-Path -LiteralPath $Python)) {
    throw "Windows environment is missing. Run setup_windows.ps1 first."
}

$env:PYTHONUTF8 = "1"
$env:OPENCV_VIDEOIO_PRIORITY_GSTREAMER = "0"
& $Python -u (Join-Path $PSScriptRoot "ground_station_windows.py") @GroundStationArgs
exit $LASTEXITCODE
