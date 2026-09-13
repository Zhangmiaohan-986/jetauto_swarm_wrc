param(
    [string]$VmAddress = '192.168.1.104',
    [string]$IdentityFile = 'C:\Users\10196\.ssh\id_ed25519_jetauto_vm',
    [switch]$Force
)

# VM runtime copy -> F: source. This never starts ROS or contacts robots.
$ErrorActionPreference = 'Stop'
$repoRoot = (Resolve-Path -LiteralPath (Join-Path $PSScriptRoot '..')).Path
$packagePath = Join-Path $repoRoot 'jetauto_multi'
if ((Split-Path $repoRoot -Leaf) -ne 'jetauto_swarm') { throw 'Unexpected repository directory' }
if (!(Test-Path -LiteralPath $packagePath)) { throw 'Local jetauto_multi package is missing' }
if (!(Test-Path -LiteralPath $IdentityFile)) { throw 'SSH identity file missing' }

if (!$Force) {
    $dirty = @(& git -C $repoRoot status --porcelain --untracked-files=all -- jetauto_multi)
    if ($LASTEXITCODE -ne 0) { throw 'Unable to inspect local Git status' }
    if ($dirty.Count -gt 0) {
        throw 'Local jetauto_multi has uncommitted changes. Commit/stash them, or rerun with -Force.'
    }
}

$destination = "ubuntu@$VmAddress"
$token = [guid]::NewGuid().ToString('N')
$remoteArchive = "/tmp/jetauto_multi_pull_$token.tar.gz"
$localArchive = Join-Path ([IO.Path]::GetTempPath()) "jetauto_multi_pull_$token.tar.gz"
$extractRoot = Join-Path ([IO.Path]::GetTempPath()) "jetauto_multi_pull_$token"

try {
    & ssh -i $IdentityFile -o BatchMode=yes -o ConnectTimeout=8 $destination "test -d /home/ubuntu/jetauto_real_dev_ws/src/jetauto_multi && tar -czf '$remoteArchive' --exclude=jetauto_multi/.git --exclude='*/__pycache__' --exclude='*.pyc' -C /home/ubuntu/jetauto_real_dev_ws/src jetauto_multi"
    if ($LASTEXITCODE -ne 0) { throw 'VM archive creation failed' }

    & scp -i $IdentityFile "${destination}:$remoteArchive" $localArchive
    if ($LASTEXITCODE -ne 0) { throw 'VM download failed' }

    New-Item -ItemType Directory -Path $extractRoot | Out-Null
    & tar -xzf $localArchive -C $extractRoot
    if ($LASTEXITCODE -ne 0) { throw 'Local archive extraction failed' }

    $incoming = Join-Path $extractRoot 'jetauto_multi'
    & robocopy $incoming $packagePath /MIR /COPY:DAT /DCOPY:DAT /R:1 /W:1 /XD .git __pycache__ .pytest_cache /XF '*.pyc' '*.pyo' | Out-Null
    if ($LASTEXITCODE -ge 8) { throw "robocopy failed with code $LASTEXITCODE" }

    & git -C $repoRoot status --short -- jetauto_multi
    Write-Output 'Pulled VM jetauto_multi into the F: source tree. Review and commit before pushing back.'
} finally {
    & ssh -i $IdentityFile -o BatchMode=yes -o ConnectTimeout=8 $destination "rm -f '$remoteArchive'" 2>$null
    if (Test-Path -LiteralPath $localArchive) { Remove-Item -LiteralPath $localArchive }
    if (Test-Path -LiteralPath $extractRoot) {
        $resolvedExtract = (Resolve-Path -LiteralPath $extractRoot).Path
        $resolvedTemp = (Resolve-Path -LiteralPath ([IO.Path]::GetTempPath())).Path
        if ($resolvedExtract.StartsWith($resolvedTemp, [StringComparison]::OrdinalIgnoreCase)) {
            Remove-Item -LiteralPath $resolvedExtract -Recurse -Force
        }
    }
}
