param(
    [string]$VmAddress = '192.168.1.104',
    [string]$IdentityFile = 'C:\Users\10196\.ssh\id_ed25519_jetauto_vm'
)
# F: source -> VM runtime copy only. Never starts ROS or contacts physical robots.
$ErrorActionPreference = 'Stop'
$packagePath = (Resolve-Path -LiteralPath (Join-Path $PSScriptRoot '../..')).Path
if ((Split-Path $packagePath -Leaf) -ne 'jetauto_multi') { throw 'Unexpected package directory' }
if (!(Test-Path -LiteralPath $IdentityFile)) { throw 'SSH identity file missing' }
$destination = "ubuntu@$VmAddress"
$archivePath = Join-Path ([IO.Path]::GetTempPath()) ('jetauto_sync_' + [guid]::NewGuid() + '.tar.gz')
try {
    & ssh -i $IdentityFile -o BatchMode=yes -o ConnectTimeout=8 $destination 'test -w /home/ubuntu/jetauto_real_dev_ws/src/jetauto_multi'
    if ($LASTEXITCODE -ne 0) { throw 'VM SSH or workspace write check failed' }
    & tar --format=ustar -czf $archivePath --exclude=__pycache__ --exclude='*.pyc' -C (Split-Path $packagePath -Parent) jetauto_multi
    if ($LASTEXITCODE -ne 0) { throw 'Archive creation failed' }
    & scp -i $IdentityFile $archivePath "${destination}:/tmp/jetauto_fleet_sync.tar.gz"
    if ($LASTEXITCODE -ne 0) { throw 'Upload failed' }
    $remoteScript = @'
set -e
target=/home/ubuntu/jetauto_real_dev_ws/src/jetauto_multi
test "$(readlink -f "$target")" = /home/ubuntu/jetauto_real_dev_ws/src/jetauto_multi
if pgrep -f '[s]cripts/fleet_vendor_omni/(supervisor|launch_fleet|sim_adapter).py' >/dev/null; then
  echo 'Stop the VM fleet launch before synchronizing.' >&2
  exit 3
fi
stamp=$(date +%Y%m%d_%H%M%S)
backup=/home/ubuntu/experiment_records/source_backups
mkdir -p "$backup"
tar -czf "$backup/jetauto_multi_$stamp.tar.gz" -C /home/ubuntu/jetauto_real_dev_ws/src jetauto_multi
incoming=$(mktemp -d /tmp/jetauto_fleet_sync.XXXXXX)
tar -xzf /tmp/jetauto_fleet_sync.tar.gz -C "$incoming"
rsync -a --delete "$incoming/jetauto_multi/" "$target/"
find "$target/scripts" "$target/src" -type f \( -name '*.py' -o -name '*.sh' \) -exec chmod +x {} +
source /opt/ros/melodic/setup.bash
cd /home/ubuntu/jetauto_real_dev_ws
catkin_make --pkg jetauto_multi -j4
echo "Synchronized. Previous VM package: $backup/jetauto_multi_$stamp.tar.gz"
'@
    $remoteScript.Replace("`r`n", "`n") | & ssh -i $IdentityFile $destination 'bash -s'
    if ($LASTEXITCODE -ne 0) { throw 'VM synchronization/build failed; inspect output' }
} finally {
    if (Test-Path -LiteralPath $archivePath) { Remove-Item -LiteralPath $archivePath }
}
