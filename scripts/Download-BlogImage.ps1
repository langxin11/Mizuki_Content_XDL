param([Parameter(Mandatory)][string]$Url, [Parameter(Mandatory)][string]$Destination)
$ErrorActionPreference = 'Stop'
if (-not $Url.StartsWith('https://raw.githubusercontent.com/langxin11/Picture/main/blog/')) {
    throw 'Unexpected image source'
}
$imageRoot = [IO.Path]::GetFullPath((Join-Path $PSScriptRoot '../images/blog')) + [IO.Path]::DirectorySeparatorChar
$resolvedDestination = [IO.Path]::GetFullPath($Destination)
if (-not $resolvedDestination.StartsWith($imageRoot, [StringComparison]::OrdinalIgnoreCase)) {
    throw 'Image destination must remain in images/blog'
}
Invoke-WebRequest -Uri $Url -OutFile $resolvedDestination -TimeoutSec 60
