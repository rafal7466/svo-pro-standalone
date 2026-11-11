<#
.SYNOPSIS
  Runs svo_euroc.exe on one EuRoC sequence with one of the provided configurations.

.PARAMETER Dataset
  EuRoC sequence in ASL format: the folder that contains mav0\ (e.g. dataset\MH_01_easy)
  or the mav0 folder itself.
.PARAMETER Mode
  stereo_vio     stereo frontend + Ceres VIO backend   (config\euroc_vio_stereo.yaml)
  mono_vio       mono frontend + Ceres VIO backend     (config\euroc_vio_mono.yaml)
  stereo_vo      stereo frontend only, no IMU          (config\euroc_frontend_stereo.yaml)
  mono_frontend  mono frontend with IMU rotation prior (config\euroc_frontend_mono.yaml)
.PARAMETER Viz
  Show the OpenCV windows (tracked features, top view of the trajectory).
.PARAMETER Extra
  Additional command-line options passed to svo_euroc.exe, e.g. "--start_frame=850".

.EXAMPLE
  powershell -ExecutionPolicy Bypass -File scripts\run_euroc.ps1 -Dataset dataset\MH_01_easy -Mode stereo_vio
#>
param(
  [Parameter(Mandatory = $true)][string]$Dataset,
  [ValidateSet("stereo_vio", "mono_vio", "stereo_vo", "mono_frontend")]
  [string]$Mode = "stereo_vio",
  [string]$OutputDir = "results",
  [ValidateSet("Release", "RelWithDebInfo", "Debug")]
  [string]$Config = "Release",
  [switch]$Viz,
  [string[]]$Extra = @()
)
$ErrorActionPreference = "Stop"
$repo = Split-Path -Parent $PSScriptRoot
$cfg = @{
  stereo_vio    = "euroc_vio_stereo.yaml"
  mono_vio      = "euroc_vio_mono.yaml"
  stereo_vo     = "euroc_frontend_stereo.yaml"
  mono_frontend = "euroc_frontend_mono.yaml"
}[$Mode]

$exe = Join-Path $repo "build\bin\$Config\svo_euroc.exe"
if (-not (Test-Path $exe)) { throw "$exe not found - build first with scripts\build.ps1" }
if (-not (Test-Path $Dataset)) { throw "Dataset folder '$Dataset' not found (see the Dataset section of the README)" }

$vizFlag = if ($Viz.IsPresent) { "true" } else { "false" }
$cmdArgs = @("--dataset=$((Resolve-Path $Dataset).Path)",
          "--config=$(Join-Path $repo "config\$cfg")",
          "--output_dir=$OutputDir",
          "--viz=$vizFlag", "--wait_at_end=false") + $Extra
Write-Host "$exe $($cmdArgs -join ' ')"
& $exe @cmdArgs
if ($LASTEXITCODE -ne 0) { throw "svo_euroc.exe exited with code $LASTEXITCODE" }
