# Publish a committed main-branch revision and immediately start the checked Pages build.
param()
$ErrorActionPreference = 'Stop'
$contentRoot = [IO.Path]::GetFullPath((Join-Path $PSScriptRoot '..'))
Push-Location $contentRoot
try {
    if ((git branch --show-current) -ne 'main') { throw 'Switch to the main branch before publishing.' }
    if (git status --porcelain) { throw 'Commit the intended content changes before publishing.' }
    git -c 'credential.helper=' -c 'credential.helper=!gh auth git-credential' push https://github.com/langxin11/Mizuki_Content_XDL.git main
    if ($LASTEXITCODE -ne 0) { throw 'Content push failed; deployment was not triggered.' }
    $contentRevision = git rev-parse HEAD
    gh workflow run shirone-pages.yml -R langxin11/langxin11.github.io -f "content_ref=$contentRevision"
    if ($LASTEXITCODE -ne 0) { throw 'Deployment trigger failed. The hourly check remains available.' }
    Write-Output 'Publication requested. The site changes only after build and publication succeed.'
    Write-Output 'https://github.com/langxin11/langxin11.github.io/actions/workflows/shirone-pages.yml'
} finally { Pop-Location }
