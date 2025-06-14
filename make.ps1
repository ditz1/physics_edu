param(
    [string]$BuildType = "web"
)

if ($BuildType -eq "web" -or $BuildType -eq "") {
    New-Item -ItemType Directory -Force -Path "build" | Out-Null
    Set-Location "build"
    & emcmake cmake -DUSE_LOCAL=OFF -DCMAKE_TOOLCHAIN_FILE="$env:EMSDK/upstream/emscripten/cmake/Modules/Platform/Emscripten.cmake" ..
    & emmake make
}
elseif ($BuildType -eq "native") {
    New-Item -ItemType Directory -Force -Path "build" | Out-Null
    Set-Location "build"
    & cmake -G "MinGW Makefiles"..
    & make
}
elseif ($BuildType -eq "clean") {
    Remove-Item -Recurse -Force -Path "build" -ErrorAction SilentlyContinue
}
else {
    Write-Host "Invalid argument. Use 'native', 'clean', 'web', 'local', or 'rebuild'." -ForegroundColor Red
    exit 1
}