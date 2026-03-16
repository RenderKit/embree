# Prepare Embree test files from build artifacts (PowerShell)
param(
    [string]$DestDir = "./build/embree-release",
    [string]$Pattern = "./build/*.zip", 
    [string]$TestType = "ctest"
)

Write-Host "Preparing test files..."
Write-Host "Destination: $DestDir"
Write-Host "Archive pattern: $Pattern"  
Write-Host "Test type: $TestType"

# Create destination directory
New-Item -ItemType Directory -Path $DestDir -Force | Out-Null

if ($TestType -eq "integration") {
    # For integration tests, extract to embree_install
    $DestDir = "./build/embree_install"
    New-Item -ItemType Directory -Path $DestDir -Force | Out-Null
    
    Get-ChildItem -Path $Pattern | ForEach-Object {
        Expand-Archive -Path $_.FullName -DestinationPath $DestDir -Force
        Write-Host "Extracted: $($_.Name)"
    }
    
    # Find embree-config.cmake and set environment variable
    $EmbreeConfig = Get-ChildItem -Path $DestDir -Recurse -Name "embree-config.cmake" | Select-Object -First 1
    if ($EmbreeConfig) {
        $EmbreeDir = Split-Path -Parent (Get-ChildItem -Path $DestDir -Recurse -Filter "embree-config.cmake" | Select-Object -First 1).FullName
        $EmbreeDir = $EmbreeDir -replace [regex]::Escape((Get-Location).Path + "\"), ""
        Write-Host "EMBREE_DIR=$EmbreeDir" | Out-File -FilePath $env:GITHUB_ENV -Append
        Write-Host "Found embree-config.cmake at: $EmbreeDir"
    } else {
        Write-Error "embree-config.cmake not found"
        exit 1
    }
} else {
    # For CTest, extract to embree-release  
    Get-ChildItem -Path $Pattern | ForEach-Object {
        Expand-Archive -Path $_.FullName -DestinationPath $DestDir -Force
        Write-Host "Extracted: $($_.Name)"
    }
    Write-Host "EMBREE_DIR=$DestDir" | Out-File -FilePath $env:GITHUB_ENV -Append
    Write-Output $DestDir  # Output for immediate use
}

Write-Host "Test files prepared successfully"