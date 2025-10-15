# Toggle Mock Mode Script
# Quickly switch between mock mode and backend mode

param(
    [Parameter(Mandatory=$false)]
    [ValidateSet("mock", "backend", "status")]
    [string]$Mode = "status"
)

$envFile = Join-Path $PSScriptRoot ".env"

if (-not (Test-Path $envFile)) {
    Write-Host "Error: .env file not found at $envFile" -ForegroundColor Red
    Write-Host "Tip: Copy .env.example to .env first" -ForegroundColor Yellow
    exit 1
}

$content = Get-Content $envFile -Raw

function Get-CurrentMode {
    if ($content -match "VITE_MOCK_MODE=true") {
        return "mock"
    } else {
        return "backend"
    }
}

function Set-MockMode {
    $newContent = $content -replace "VITE_MOCK_MODE=(true|false)", "VITE_MOCK_MODE=true"
    Set-Content -Path $envFile -Value $newContent -NoNewline
    Write-Host "Mock mode ENABLED" -ForegroundColor Green
    Write-Host "   - UI will use mock data" -ForegroundColor Gray
    Write-Host "   - No backend required" -ForegroundColor Gray
    Write-Host "   - Run: npm run dev" -ForegroundColor Cyan
}

function Set-BackendMode {
    $newContent = $content -replace "VITE_MOCK_MODE=(true|false)", "VITE_MOCK_MODE=false"
    Set-Content -Path $envFile -Value $newContent -NoNewline
    Write-Host "Backend mode ENABLED" -ForegroundColor Green
    Write-Host "   - UI will connect to backend" -ForegroundColor Gray
    Write-Host "   - Backend must be running on port 8080" -ForegroundColor Gray
    Write-Host "   - Run: npm run dev" -ForegroundColor Cyan
}

function Show-Status {
    $currentMode = Get-CurrentMode
    Write-Host "`nCurrent Configuration:" -ForegroundColor Cyan
    Write-Host "   Mode: " -NoNewline
    if ($currentMode -eq "mock") {
        Write-Host "MOCK" -ForegroundColor Yellow
        Write-Host "   Status: Using mock data, no backend required" -ForegroundColor Gray
    } else {
        Write-Host "BACKEND" -ForegroundColor Green
        Write-Host "   Status: Connecting to backend on localhost:8080" -ForegroundColor Gray
    }
    
    # Check if dev server is running
    $devServerRunning = $false
    try {
        $response = Invoke-WebRequest -Uri "http://localhost:5173" -TimeoutSec 2 -ErrorAction SilentlyContinue
        $devServerRunning = $true
    } catch {
        $devServerRunning = $false
    }
    
    Write-Host "   Dev Server: " -NoNewline
    if ($devServerRunning) {
        Write-Host "RUNNING" -ForegroundColor Green -NoNewline
        Write-Host " (http://localhost:5173)" -ForegroundColor Gray
    } else {
        Write-Host "NOT RUNNING" -ForegroundColor Red
        Write-Host "   Start with: npm run dev" -ForegroundColor Yellow
    }
    
    # Check backend if in backend mode
    if ($currentMode -eq "backend") {
        Write-Host "   Backend: " -NoNewline
        try {
            $response = Invoke-WebRequest -Uri "http://localhost:8080/api/health" -TimeoutSec 2 -ErrorAction SilentlyContinue
            Write-Host "ONLINE" -ForegroundColor Green
        } catch {
            Write-Host "OFFLINE" -ForegroundColor Red
            Write-Host "   Start backend in WSL: ./run.sh my_test_config.yaml" -ForegroundColor Yellow
        }
    }
    
    Write-Host ""
}

# Main logic
switch ($Mode) {
    "mock" {
        Set-MockMode
        Write-Host ""
        Show-Status
    }
    "backend" {
        Set-BackendMode
        Write-Host ""
        Show-Status
    }
    "status" {
        Show-Status
    }
}

Write-Host "Usage:" -ForegroundColor Cyan
Write-Host "   .\toggle-mock-mode.ps1 mock      # Enable mock mode" -ForegroundColor Gray
Write-Host "   .\toggle-mock-mode.ps1 backend   # Enable backend mode" -ForegroundColor Gray
Write-Host "   .\toggle-mock-mode.ps1 status    # Show current status" -ForegroundColor Gray
Write-Host ""