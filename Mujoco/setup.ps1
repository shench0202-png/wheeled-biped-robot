$ErrorActionPreference = "Stop"

$ProjectRoot = Split-Path -Parent $PSScriptRoot
$VenvPython = Join-Path $ProjectRoot ".venv\Scripts\python.exe"

if (-not (Test-Path $VenvPython)) {
    py -3.12 -m venv (Join-Path $ProjectRoot ".venv")
}

& $VenvPython -m pip install --upgrade pip setuptools wheel

if (Get-Command nvidia-smi -ErrorAction SilentlyContinue) {
    Write-Host "NVIDIA GPU detected. Installing CUDA 12.6 PyTorch..."
    & $VenvPython -m pip install "torch==2.12.1+cu126" `
        --index-url https://download.pytorch.org/whl/cu126
}
else {
    Write-Host "No NVIDIA GPU detected. Installing CPU PyTorch..."
    & $VenvPython -m pip install "torch==2.12.1"
}

& $VenvPython -m pip install -r (Join-Path $PSScriptRoot "requirements.txt")
& $VenvPython (Join-Path $PSScriptRoot "smoke_test.py")
& $VenvPython (Join-Path $PSScriptRoot "rl_smoke_test.py")

Write-Host ""
Write-Host "MuJoCo environment is ready."
Write-Host "Activate:  .\.venv\Scripts\Activate.ps1"
Write-Host "Viewer:    python .\Mujoco\run_viewer.py"
Write-Host "RL test:   python .\Mujoco\rl_smoke_test.py"

