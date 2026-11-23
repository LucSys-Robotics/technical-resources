param([string]$UV4 = 'C:\Keil_v5\UV4\UV4.exe', [string]$Output = "$PSScriptRoot\logs")
$ErrorActionPreference = 'Stop'
if (!(Test-Path $UV4)) { throw "Keil executable not found: $UV4" }
$Root = (Resolve-Path "$PSScriptRoot\..\..").Path
New-Item -ItemType Directory -Force -Path $Output | Out-Null
$Output = (Resolve-Path $Output).Path
$Rows = Get-Content "$PSScriptRoot\keil-projects.json" -Raw | ConvertFrom-Json
$Results = @()
$i = 0
foreach ($Row in $Rows) {
    $i++
    $Project = Join-Path $Root $Row.project
    $Log = Join-Path $Output ("{0:D2}-{1}.log" -f $i, $Row.target)
    $Args = '-r "{0}" -t "{1}" -o "{2}"' -f $Project, $Row.target, $Log
    $Process = Start-Process -FilePath $UV4 -ArgumentList $Args -WorkingDirectory (Split-Path $Project) -Wait -PassThru
    $Text = if (Test-Path $Log) { Get-Content $Log -Raw } else { '' }
    $Status = 'FAIL_OR_REVIEW'
    if ($Process.ExitCode -eq 0 -and $Text -match '0 Error\(s\)') { $Status = 'BUILT' }
    if ($Process.ExitCode -eq 1 -and $Text -match '0 Error\(s\)') { $Status = 'BUILT_WITH_WARNINGS_REVIEW' }
    $Results += [pscustomobject]@{ project=$Row.project; target=$Row.target; exit_code=$Process.ExitCode; status=$Status; log=$Log }
}
$Results | ConvertTo-Json -Depth 4 | Set-Content (Join-Path $Output 'results.json')
if ($Results.status -contains 'FAIL_OR_REVIEW') { exit 1 }
