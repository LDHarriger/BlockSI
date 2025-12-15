# BlockSI Build and Flash Script
# Quick commands for common development tasks

param(
    [string]$Command = "help",
    [string]$Port = "COM3"
)

function Show-Help {
    Write-Host "BlockSI Build and Flash Script"
    Write-Host "==============================="
    Write-Host ""
    Write-Host "Usage: .\build.ps1 [command] [-Port COMx]"
    Write-Host ""
    Write-Host "Commands:"
    Write-Host "  config    - Open menuconfig"
    Write-Host "  build     - Build firmware"
    Write-Host "  flash     - Flash firmware to device"
    Write-Host "  monitor   - Open serial monitor"
    Write-Host "  all       - Build, flash, and monitor"
    Write-Host "  clean     - Clean build artifacts"
    Write-Host "  erase     - Erase flash completely"
    Write-Host "  help      - Show this help"
    Write-Host ""
    Write-Host "Examples:"
    Write-Host "  .\build.ps1 config"
    Write-Host "  .\build.ps1 build"
    Write-Host "  .\build.ps1 all -Port COM4"
    Write-Host "  .\build.ps1 monitor -Port COM3"
}

switch ($Command.ToLower()) {
    "config" {
        Write-Host "Opening menuconfig..."
        idf.py menuconfig
    }
    "build" {
        Write-Host "Building firmware..."
        idf.py build
    }
    "flash" {
        Write-Host "Flashing to $Port..."
        idf.py -p $Port flash
    }
    "monitor" {
        Write-Host "Opening monitor on $Port (Ctrl+] to exit)..."
        idf.py -p $Port monitor
    }
    "all" {
        Write-Host "Building, flashing, and monitoring..."
        idf.py build
        if ($LASTEXITCODE -eq 0) {
            idf.py -p $Port flash monitor
        }
    }
    "clean" {
        Write-Host "Cleaning build artifacts..."
        idf.py fullclean
    }
    "erase" {
        Write-Host "Erasing flash on $Port..."
        Write-Host "WARNING: This will erase all data including WiFi credentials!"
        $confirm = Read-Host "Continue? (yes/no)"
        if ($confirm -eq "yes") {
            idf.py -p $Port erase-flash
        } else {
            Write-Host "Cancelled."
        }
    }
    default {
        Show-Help
    }
}
