# rts_live.ps1 - open the three RTS terminals as tabs in one Windows Terminal window.
#
# Only VM1 accepts key login, so every tab SSHes into VM1; the VM2 and VM3
# processes are started from there over Qnet with "on -f <node>". Their
# keyboard and screen ride the ssh pty back through Qnet.
#
# Order follows README section 10: railway first, central last.
param(
    [string]$VM1 = "192.168.56.110",
    [string]$Dir = "/tmp/manh",
    [string]$Speed = "5"
)

# Speed rides in as RTS_SPEED so it reaches the six intersections too
# (run_vm2.sh starts them in the background without passing -s along).
$tabs = @(
    @{ Title = "VM3 railway";       Cmd = "on -f vm3 -e RTS_SPEED=$Speed $Dir/run_vm3.sh"; Wait = 3 },
    @{ Title = "VM2 intersections"; Cmd = "on -f vm2 -e RTS_SPEED=$Speed $Dir/run_vm2.sh"; Wait = 9 },
    @{ Title = "VM1 central";       Cmd = "RTS_SPEED=$Speed $Dir/run_vm1.sh";              Wait = 0 }
)

foreach ($t in $tabs) {
    $ssh = "ssh -t -o ServerAliveInterval=5 -o StrictHostKeyChecking=accept-new root@$VM1 `"$($t.Cmd)`""
    $args = "-w rts new-tab --title `"$($t.Title)`" --suppressApplicationTitle $ssh"
    Start-Process wt.exe -ArgumentList $args
    Start-Sleep -Seconds $t.Wait
}
