# Run Embree tests (integration or CTest) - PowerShell
param(
    [string]$TestType = "ctest",
    [string]$BuildType = "Release", 
    [string]$CCompiler = "",
    [string]$CxxCompiler = "",
    [string]$EmbreeDir = "",
    [string]$CtestConfig = "Release"
)

Write-Host "Running tests..."
Write-Host "Test type: $TestType"
Write-Host "Build type: $BuildType"
Write-Host "Embree directory: $EmbreeDir"

if ($TestType -eq "integration") {
    Write-Host "Running integration tests..."
    
    if (-not $EmbreeDir) {
        Write-Error "EMBREE_DIR not specified for integration tests"
        exit 1
    }
    
    Set-Location "tests/integration/test_embree_release"
    
    # Build cmake command with compiler options
    $CmakeOpts = @("-B", "build", "-DCMAKE_BUILD_TYPE=$BuildType")
    
    if ($CCompiler) {
        $CmakeOpts += "-DCMAKE_C_COMPILER=$CCompiler"
    }
    
    if ($CxxCompiler) {
        $CmakeOpts += "-DCMAKE_CXX_COMPILER=$CxxCompiler"  
    }
    
    # Use absolute path for embree_DIR
    $RootDir = (Get-Location).Path + "/../../../"
    $CmakeOpts += "-Dembree_DIR=$RootDir/$EmbreeDir"
    
    Write-Host "CMake command: cmake $($CmakeOpts -join ' ')"
    & cmake @CmakeOpts
    if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }
    
    & cmake --build build --config $BuildType
    if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }
    
    & "./build/test"
    if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }
    
} elseif ($TestType -eq "ctest") {
    Write-Host "Running CTest..."
    
    if (-not $EmbreeDir) {
        Write-Error "EMBREE_DIR not specified for CTest"
        exit 1
    }
    
    Set-Location $EmbreeDir
    & cmake -S testing -B build
    if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }
    
    Set-Location build
    & ctest -C $CtestConfig --output-on-failure -VV  
    if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }
    
} else {
    Write-Error "Unknown test type '$TestType'. Use 'integration' or 'ctest'"
    exit 1
}

Write-Host "Tests completed successfully"