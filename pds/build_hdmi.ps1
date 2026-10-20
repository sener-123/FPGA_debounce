param([string]$PdsHome = 'D:/PDS_2022.2-SP4.2-ads')
$ErrorActionPreference = 'Stop'
$sourceRoot = (Resolve-Path (Join-Path $PSScriptRoot '..')).Path
$buildDir = Join-Path $sourceRoot 'prj/davinci_hdmi'
New-Item -ItemType Directory -Force -Path $buildDir | Out-Null
$generator = Join-Path $PdsHome 'bin/ip_generate.exe'
$compiler = Join-Path $PdsHome 'bin/pds_shell.exe'
if (!(Test-Path -LiteralPath $generator) -or !(Test-Path -LiteralPath $compiler)) { throw 'PDS tools not found.' }
& $generator -i (Join-Path $PSScriptRoot 'ip/PLL/PLL.idf') -disable-syn
if ($LASTEXITCODE -ne 0) { throw 'PLL generation failed.' }
$buildLog = Join-Path $buildDir 'build_console.log'
& $compiler -file (Join-Path $PSScriptRoot 'build_hdmi.tcl') -work_dir $buildDir -project_name davinci_hdmi *> $buildLog
if ($LASTEXITCODE -ne 0) { throw "PDS build failed. See $buildLog" }
$log = Get-Content -LiteralPath $buildLog -Raw
if ($log -notmatch 'Executing : gen_bit_stream successfully') { throw 'Bitstream generation not confirmed by PDS.' }
$bitstream = Join-Path $buildDir 'generate_bitstream/davinci_hdmi_top.sbit'
if (!(Test-Path -LiteralPath $bitstream) -or (Get-Item -LiteralPath $bitstream).Length -eq 0) { throw 'Bitstream missing or empty.' }
Write-Output "Bitstream generated: $bitstream"
Write-Output "Project: $(Join-Path $buildDir 'davinci_hdmi.pds')"
Write-Output "Timing report: $(Join-Path $buildDir 'report_timing/davinci_hdmi_top.rtr')"
