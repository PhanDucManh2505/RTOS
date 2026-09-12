# deploy.ps1 - build on Windows and copy the binaries onto the three VMs.
#
# Run from PowerShell in the project directory:
#     .\scripts\deploy.ps1
#
# It expects the QNX environment to be set up, which the SDP installs as
# qnxsdp-env.bat. If "make" is not on your PATH, run the build inside
# Momentics instead and then run this script with -SkipBuild.

param(
    [string]$VM1 = "192.168.56.110",
    [string]$VM2 = "192.168.56.111",
    [string]$VM3 = "192.168.56.112",
    [string]$Dir = "/tmp/manh",
    [switch]$SkipBuild
)

$ErrorActionPreference = "Stop"
$root = Split-Path -Parent $PSScriptRoot

if (-not $SkipBuild) {
    Write-Host "building..." -ForegroundColor Cyan
    Push-Location $root
    make
    Pop-Location
}

$targets = @(
    @{ ip = $VM1; files = @("central") ; script = "run_vm1.sh" },
    @{ ip = $VM2; files = @("intersection_i1","intersection_i2","intersection_i3",
                            "intersection_i4","intersection_i5","intersection_i6")
       script = "run_vm2.sh" },
    @{ ip = $VM3; files = @("railway") ; script = "run_vm3.sh" }
)

foreach ($t in $targets) {
    Write-Host "deploying to $($t.ip)" -ForegroundColor Green
    ssh "root@$($t.ip)" "mkdir -p $Dir /fs/rts"
    foreach ($f in $t.files) {
        scp "$root/bin/$f" "root@$($t.ip):$Dir/"
    }
    scp "$root/scripts/$($t.script)" "root@$($t.ip):$Dir/"
    scp "$root/scripts/start_qnet.sh" "root@$($t.ip):$Dir/"
    ssh "root@$($t.ip)" "chmod +x $Dir/*"
}
Write-Host "done" -ForegroundColor Cyan
