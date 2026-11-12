# Local fallback build for the current MC02 project. Does not flash hardware.
param(
    [string]$CubeIdeHome = 'C:\ST\STM32CubeIDE_2.1.1\STM32CubeIDE',
    [string]$BuildDir = 'build/BenchDebug'
)
$ErrorActionPreference = 'Stop'
$plugins = Join-Path $CubeIdeHome 'plugins'
$gccPlugin = Get-ChildItem -LiteralPath $plugins -Directory -Filter '*externaltools.gnu-tools-for-stm32*' |
    Sort-Object Name -Descending | Select-Object -First 1
if (!$gccPlugin) { throw "STM32 GCC plugin not found in $plugins" }
$toolBin = Join-Path $gccPlugin.FullName 'tools\bin'
& python (Join-Path $PSScriptRoot 'build_firmware.py') --toolchain-dir $toolBin --build-dir $BuildDir
if ($LASTEXITCODE -ne 0) { throw "Firmware build failed: $LASTEXITCODE" }
