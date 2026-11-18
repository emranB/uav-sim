# Run in an Administrator PowerShell terminal after Docker content cleanup.
# Docker and its containers are briefly unavailable during compaction.
[CmdletBinding()]
param()

$ErrorActionPreference = 'Stop'
$identity = [Security.Principal.WindowsIdentity]::GetCurrent()
$principal = [Security.Principal.WindowsPrincipal]::new($identity)
if (-not $principal.IsInRole([Security.Principal.WindowsBuiltInRole]::Administrator)) {
    throw 'Open PowerShell as Administrator to compact the Docker virtual disk.'
}

$expectedDisk = Join-Path $env:LOCALAPPDATA 'Docker\wsl\disk\docker_data.vhdx'
$resolvedDisk = (Resolve-Path -LiteralPath $expectedDisk).ProviderPath
if ($resolvedDisk -ne [IO.Path]::GetFullPath($expectedDisk)) {
    throw 'The resolved Docker disk differs from the expected disk; refusing to continue.'
}
if ($resolvedDisk -match '["\r\n]') {
    throw 'The Docker disk path contains characters unsupported by DiskPart.'
}

$runningContainers = @(docker ps --format '{{.Names}}')
if ($LASTEXITCODE -ne 0) {
    throw 'Docker inventory failed. No host changes were made.'
}
$diskPartFile = [IO.Path]::GetTempFileName()
$restartDocker = $false
try {
    # Trim the filesystem before stopping the VM; no images or volumes are deleted here.
    wsl -d docker-desktop -- sh -lc 'fstrim -v /mnt/docker-desktop-disk'
    if ($LASTEXITCODE -ne 0) {
        throw 'Docker filesystem trim failed; compaction was not started.'
    }
    $restartDocker = $true
    docker desktop stop
    if ($LASTEXITCODE -ne 0) {
        throw 'Docker Desktop did not stop; compaction was not started.'
    }
    # Stop only the Docker distribution, leaving any other Linux distributions alone.
    wsl --terminate docker-desktop
    @"
select vdisk file="$resolvedDisk"
attach vdisk readonly
compact vdisk
detach vdisk
exit
"@ | Set-Content -LiteralPath $diskPartFile -Encoding ascii
    diskpart /s $diskPartFile
    if ($LASTEXITCODE -ne 0) {
        throw 'DiskPart reported an error. Review its output before retrying.'
    }
    Write-Host 'Review the DiskPart output above to confirm compaction succeeded.'
}
finally {
    Remove-Item -LiteralPath $diskPartFile -Force
    if ($restartDocker) {
        docker desktop start
        if ($LASTEXITCODE -eq 0 -and $runningContainers.Count -gt 0) {
            docker start @runningContainers
        }
    }
}

Get-PSDrive -PSProvider FileSystem | Select-Object Name, Free
