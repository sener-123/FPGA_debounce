param([string]$Vsim = 'vsim')
$ErrorActionPreference = 'Stop'
$sourceRoot = (Resolve-Path (Join-Path $PSScriptRoot '..')).Path.Replace('\', '/')
$runDirectory = Join-Path ([IO.Path]::GetTempPath()) ('fpga_regression_' + [guid]::NewGuid().ToString('N'))
New-Item -ItemType Directory -Path $runDirectory | Out-Null
$logPath = Join-Path $runDirectory 'regression.log'
Push-Location $runDirectory
try {
    & $Vsim -c -do "set SRC {$sourceRoot}; do {$sourceRoot/sim/run_regression.do}" -l $logPath
    $simExit = $LASTEXITCODE
} finally { Pop-Location }
$log = Get-Content -LiteralPath $logPath -Raw
$tests = @('motion_estimate','motion_search','stab_core','trajectory_smooth','shift_compensate','shift_crop','shift_frame_lock','compensation_direction','disp_reader','hdmi_timing_direct','pixel_stream_cdc','dvi_tx','video_stab_hdmi')
$missing = @($tests | Where-Object { $log -notmatch "=== tb_$_ PASS" })
Write-Output "Simulation log: $logPath"
if ($simExit -ne 0 -or $missing.Count -gt 0 -or $log -match '(?m)^# .*\b(Fatal:|FAIL:|ERROR:)') {
    throw "Regression failed; incomplete tests: $($missing -join ', ')"
}
Write-Output 'All 13 regression testbenches passed.'
