param(
    [string]$Port,
    [switch]$Flash,
    [switch]$Monitor
)

$ErrorActionPreference = "Stop"

$projectDir = Split-Path -Parent $MyInvocation.MyCommand.Path
Set-Location $projectDir

$possibleRoots = @(
    $env:IDF_PATH,
    (Join-Path $env:USERPROFILE "esp\esp-idf"),
    "C:\Espressif\frameworks\esp-idf-v5.0",
    "C:\Espressif\frameworks\esp-idf-v5.1",
    "C:\Espressif\frameworks\esp-idf-v5.2",
    "C:\Espressif\frameworks\esp-idf-v5.3",
    "C:\Espressif\frameworks\esp-idf-v5.4",
    "C:\Espressif\frameworks\esp-idf-v5.5"
)

$idfRoot = $null
foreach ($candidate in $possibleRoots) {
    if ($candidate -and (Test-Path $candidate)) {
        $idfRoot = $candidate
        break
    }
}

if (-not $idfRoot) {
    $installed = Get-ChildItem "C:\Espressif" -Directory -ErrorAction SilentlyContinue |
        Where-Object { $_.Name -like "esp-idf-*" } |
        Sort-Object Name -Descending

    if ($installed) {
        $idfRoot = $installed[0].FullName
    }
}

if (-not $idfRoot) {
    Write-Error "ESP-IDF was not found. Install ESP-IDF 5.x and then rerun this script."
    Write-Host "Typical install path: C:\Espressif\frameworks\esp-idf-v5.x"
    exit 1
}

$exportScript = Join-Path $idfRoot "export.ps1"
if (-not (Test-Path $exportScript)) {
    Write-Error "ESP-IDF export script not found in $idfRoot"
    exit 1
}

Write-Host "Using ESP-IDF from: $idfRoot"
. $exportScript

Write-Host "Setting target to esp32..."
idf.py set-target esp32

Write-Host "Building project..."
idf.py build

if ($Flash) {
    if (-not $Port) {
        Write-Error "A COM port is required when using -Flash. Example: -Port COM4"
        exit 1
    }
    Write-Host "Flashing to $Port..."
    idf.py -p $Port flash
}

if ($Monitor) {
    if (-not $Port) {
        $Port = "COM1"
    }
    Write-Host "Opening monitor on $Port..."
    idf.py -p $Port monitor
}
