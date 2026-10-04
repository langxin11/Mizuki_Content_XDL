param([switch]$SkipBuild)
$ErrorActionPreference = 'Stop'
$previewRoot = Split-Path $PSScriptRoot -Parent
$previewProject = Join-Path $previewRoot 'preview/shirone'
$previewNode = (Get-Command node.exe -ErrorAction SilentlyContinue).Source
$bundledPreviewNode = Join-Path $env:USERPROFILE '.cache/codex-runtimes/codex-primary-runtime/dependencies/node/bin/node.exe'
if (Test-Path -LiteralPath $bundledPreviewNode) { $previewNode = $bundledPreviewNode }
if (-not $previewNode) { throw 'Install Node.js 24 LTS first.' }
if (-not (Test-Path -LiteralPath (Join-Path $previewProject 'node_modules/astro/bin/astro.mjs'))) {
    throw 'Initialize preview/shirone with: npx.cmd --yes shirones@0.1.5 init'
}
$env:ASTRO_TELEMETRY_DISABLED = '1'
$env:PATH = (Split-Path $previewNode -Parent) + ';' + $env:PATH
if (-not $SkipBuild) {
    & $previewNode (Join-Path $PSScriptRoot 'sync-shirone-preview.mjs')
    if ($LASTEXITCODE -ne 0) { throw 'Content synchronization failed.' }
    Push-Location $previewProject
    try { & $previewNode node_modules/astro/bin/astro.mjs build }
    finally { Pop-Location }
    if ($LASTEXITCODE -ne 0) { throw 'Site build failed.' }
    & $previewNode (Join-Path $PSScriptRoot 'index-shirone-preview.mjs')
    if ($LASTEXITCODE -ne 0) { throw 'Search indexing failed.' }
}
Push-Location $previewProject
try { & $previewNode node_modules/astro/bin/astro.mjs preview --host 127.0.0.1 --port 4321 }
finally { Pop-Location }
