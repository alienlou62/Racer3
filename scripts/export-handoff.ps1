param(
    [string]$RepoRoot = "C:\Users\JP\racer3-rmp-basic-motion",
    [string]$OutputRoot = "C:\Users\JP"
)

$ErrorActionPreference = "Stop"

if (-not (Test-Path $RepoRoot)) { throw "RepoRoot does not exist: $RepoRoot" }

$stamp = Get-Date -Format "yyyyMMdd-HHmmss"
$out = Join-Path $OutputRoot "racer3-handoff-$stamp"
$zip = Join-Path $OutputRoot "racer3-handoff-$stamp.zip"

Write-Host "`n== Racer3 handoff export =="
Write-Host "Repo: $RepoRoot"
Write-Host "Out:  $out"
Write-Host "Zip:  $zip"

Push-Location $RepoRoot
try {
    $branch = git branch --show-current
    $commit = git log -1 --oneline --decorate
    $status = git status --short
    $diffStat = git diff --stat

    Write-Host "`n== Git state =="
    Write-Host "Branch: $branch"
    Write-Host "Commit: $commit"

    if ([string]::IsNullOrWhiteSpace($status)) {
        Write-Host "Status: clean"
    } else {
        Write-Host "Status:"
        Write-Host $status
    }

    $manifest = @(
        "# Handoff Export Manifest",
        "",
        "Generated: $((Get-Date).ToString(""yyyy-MM-dd HH:mm:ss""))",
        "Repo: $RepoRoot",
        "Branch: $branch",
        "Latest commit: $commit",
        "Working tree status: $(if ([string]::IsNullOrWhiteSpace($status)) { ""clean"" } else { $status })",
        "Diff stat: $(if ([string]::IsNullOrWhiteSpace($diffStat)) { ""none"" } else { $diffStat })",
        "",
        "Start next chat with:",
        "Use this handoff as source of truth. Read CURRENT_HANDOFF_STATE.md, docs/PROJECT_JOURNAL.md, and docs/KNOWN_GOOD_COMMANDS.md first, then summarize current state before suggesting changes."
    )
    $manifest | Set-Content -Encoding UTF8 (Join-Path $RepoRoot "HANDOFF_EXPORT_MANIFEST.md")
}
finally {
    Pop-Location
}

Remove-Item $out -Recurse -Force -ErrorAction SilentlyContinue
Remove-Item $zip -Force -ErrorAction SilentlyContinue
New-Item -ItemType Directory -Force $out | Out-Null

robocopy $RepoRoot $out /E /XD .git build build-vs2022 build-vs2022-rttask artifacts bin obj .vs .vscode /XF *.zip *.7z *.log *.err.log *.out.log *.binlog *.pdb *.ilk *.exe *.dll *.lib *.obj *.cache *.tmp | Out-Host

if ($LASTEXITCODE -gt 7) { throw "robocopy failed with exit code $LASTEXITCODE" }

Compress-Archive -Path "$out\*" -DestinationPath $zip -CompressionLevel Optimal -Force

Write-Host "`n== Handoff zip created =="
Get-Item $zip | Select-Object FullName, Length, LastWriteTime
