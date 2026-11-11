<#
.SYNOPSIS
  Configures and builds svo-pro-standalone (MSVC + vcpkg, CMake preset "windows-msvc").

.EXAMPLE
  powershell -ExecutionPolicy Bypass -File scripts\build.ps1
  powershell -ExecutionPolicy Bypass -File scripts\build.ps1 -Config RelWithDebInfo
#>
param(
  [ValidateSet("Release", "RelWithDebInfo", "Debug")]
  [string]$Config = "Release"
)
$ErrorActionPreference = "Stop"
$repo = Split-Path -Parent $PSScriptRoot
Push-Location $repo
try {
  $start = Get-Date
  cmake --preset windows-msvc
  if ($LASTEXITCODE -ne 0) { throw "CMake configure failed (is vcpkg installed? see scripts\setup_vcpkg.ps1)" }
  cmake --build build --config $Config --parallel
  if ($LASTEXITCODE -ne 0) { throw "Build failed" }
  $exe = Join-Path $repo "build\bin\$Config\svo_euroc.exe"
  if (-not (Test-Path $exe)) { throw "Build finished but $exe was not found" }
  Write-Host ""
  Write-Host ("Built {0} in {1:N0} s" -f $exe, ((Get-Date) - $start).TotalSeconds)
} finally {
  Pop-Location
}
