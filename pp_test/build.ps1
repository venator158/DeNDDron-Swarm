# PowerShell script to build the project

# Parameters
param (
    [switch]$UseClang = $false,
    [switch]$Debug = $false,
    [switch]$Clean = $false,
    [string]$VcpkgPath = ""  # Path to vcpkg installation
)

# Set build type
$BuildType = if ($Debug) { "Debug" } else { "Release" }

# Create build directory if it doesn't exist
if (-not (Test-Path "build")) {
    New-Item -ItemType Directory -Path "build" | Out-Null
}

# Clean build if requested
if ($Clean -and (Test-Path "build")) {
    Write-Host "Cleaning build directory..."
    Remove-Item -Path "build\*" -Recurse -Force
}

# Build CMake command
$CMakeCommand = "cmake -B build -S ."

# Add configuration
$CMakeCommand += " -DCMAKE_BUILD_TYPE=$BuildType"

# Add Clang option if requested
if ($UseClang) {
    $CMakeCommand += " -DUSE_CLANG=ON"
}

# Add vcpkg toolchain if path provided
if ($VcpkgPath -ne "") {
    $VcpkgToolchain = Join-Path -Path $VcpkgPath -ChildPath "scripts\buildsystems\vcpkg.cmake"
    if (Test-Path $VcpkgToolchain) {
        $CMakeCommand += " -DCMAKE_TOOLCHAIN_FILE=`"$VcpkgToolchain`""
    } else {
        Write-Host "Warning: Vcpkg toolchain file not found at $VcpkgToolchain"
    }
}

# Run CMake configure
Write-Host "Configuring project with: $CMakeCommand"
Invoke-Expression $CMakeCommand

# Build the project
Write-Host "Building project in $BuildType mode..."
cmake --build build --config $BuildType

if ($LASTEXITCODE -eq 0) {
    Write-Host "Build successful! Executable is in build\$BuildType\path_planning.exe"
} else {
    Write-Host "Build failed with exit code $LASTEXITCODE"
}
