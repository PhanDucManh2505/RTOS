# rts_tile.ps1 - start the whole system and lay the three nodes out side by side
# so all three can be watched at once:
#
#   +----------------------------+--------------+
#   | VM1 central                | VM3 railway  |
#   +----------------------------+--------------+
#   | VM2 six intersections                     |
#   +-------------------------------------------+
#
# Every window SSHes into VM1, the only node that accepts key login; the VM2
# and VM3 processes are started from there over Qnet with "on -f <node>", so
# they really run on their own node. Start order follows README section 10:
# railway first, so the intersections learn the crossing state straight away.
#
# The programs never ask the terminal how wide it is, so each window is sized
# to what its output needs: central's rows are 93 columns, 104 with RAIL HOLD,
# and the railway title bar is 65. That is also why they are not shown on the
# VirtualBox consoles, which are 80 columns wide.
#
# VM2 runs run_vm2_lines.sh rather than run_vm2.sh so the six controllers'
# output reaches the shared window a whole line at a time (see that script).
#
# Running it again restarts everything cleanly: the old windows are closed and
# every RTS process on all three nodes is stopped first.
#
#   .\scripts\rts_tile.ps1            real time (90 s cycle)
#   .\scripts\rts_tile.ps1 -Speed 5   the demonstration speed (18 s cycle)
#   .\scripts\rts_stop.ps1            stop everything on all three nodes
param(
    [string]$VM1   = "192.168.56.110",
    [string]$Dir   = "/tmp/manh",
    [string]$Speed = "1"
)

Add-Type @'
using System;
using System.Runtime.InteropServices;
public static class RtsWin {
    [StructLayout(LayoutKind.Sequential)]
    public struct RECT { public int L, T, R, B; }
    [DllImport("user32.dll")] public static extern bool SetProcessDPIAware();
    [DllImport("user32.dll")] public static extern bool GetWindowRect(IntPtr h, out RECT r);
    [DllImport("user32.dll")] public static extern bool SetWindowPos(IntPtr h, IntPtr after,
        int x, int y, int cx, int cy, uint flags);
    [DllImport("user32.dll")] public static extern bool PostMessage(IntPtr h, uint msg,
        IntPtr w, IntPtr l);
    [DllImport("user32.dll")] public static extern bool SystemParametersInfo(int action,
        int param, out RECT r, int winIni);
    [DllImport("user32.dll", CharSet = CharSet.Unicode)]
    public static extern IntPtr FindWindow(string cls, string title);
}
'@
[RtsWin]::SetProcessDPIAware() | Out-Null     # work in real pixels, not scaled ones

$WT_CLASS = "CASCADIA_HOSTING_WINDOW_CLASS"
$titles   = "VM1 central", "VM2 intersections", "VM3 railway"

function Open-Node([string]$Name, [string]$Title, [string]$Cmd, [string]$Size) {
    $ssh  = "ssh -t -o ServerAliveInterval=5 -o StrictHostKeyChecking=accept-new root@$VM1 `"$Cmd`""
    $argv = "-w $Name --size $Size new-tab --title `"$Title`" --suppressApplicationTitle $ssh"
    Start-Process wt.exe -ArgumentList $argv
}

function Find-Node([string]$Title) {
    for ($i = 0; $i -lt 50; $i++) {
        $h = [RtsWin]::FindWindow($WT_CLASS, $Title)
        if ($h -ne [IntPtr]::Zero) { return $h }
        Start-Sleep -Milliseconds 200
    }
    throw "the '$Title' window never appeared"
}

# 1. clear out any previous run: its windows, then its processes
foreach ($t in $titles) {
    $h = [RtsWin]::FindWindow($WT_CLASS, $t)
    if ($h -ne [IntPtr]::Zero) {
        [RtsWin]::PostMessage($h, 0x0010, [IntPtr]::Zero, [IntPtr]::Zero) | Out-Null   # WM_CLOSE
    }
}
ssh -o BatchMode=yes root@$VM1 'for p in central railway intersection_i1 intersection_i2 intersection_i3 intersection_i4 intersection_i5 intersection_i6; do slay -f $p; on -f vm2 slay -f $p; on -f vm3 slay -f $p; done 2>/dev/null'
foreach ($t in $titles) {
    for ($i = 0; $i -lt 25 -and [RtsWin]::FindWindow($WT_CLASS, $t) -ne [IntPtr]::Zero; $i++) {
        Start-Sleep -Milliseconds 200
    }
}
Start-Sleep -Seconds 2          # let name_attach release the service names

# 2. start the three nodes in README order
# Each window opens at its final size: 105 columns is what central needs,
# and on this 2560 px screen 67 + 105 columns fill the top row and 178 the
# bottom one.
Open-Node "rts-vm3" "VM3 railway"       "on -f vm3 -e RTS_SPEED=$Speed $Dir/run_vm3.sh"       "67,24"
Start-Sleep -Seconds 3
Open-Node "rts-vm2" "VM2 intersections" "on -f vm2 -e RTS_SPEED=$Speed $Dir/run_vm2_lines.sh" "178,23"
Start-Sleep -Seconds 9          # the six starts are staggered one second apart
Open-Node "rts-vm1" "VM1 central"       "RTS_SPEED=$Speed $Dir/run_vm1.sh"                    "105,24"

$rail    = Find-Node "VM3 railway"
$inter   = Find-Node "VM2 intersections"
$central = Find-Node "VM1 central"
Start-Sleep -Milliseconds 800   # let the last window finish sizing itself

# 3. move them into place. Only move, never resize: a resize reaches the
#    running ssh client as a window change, and one of those once killed the
#    railway session outright ("poll: No error", exit 255), taking the
#    railway process down with it on SIGHUP.
$wa = New-Object RtsWin+RECT
[RtsWin]::SystemParametersInfo(0x30, 0, [ref]$wa, 0) | Out-Null    # SPI_GETWORKAREA
$r = New-Object RtsWin+RECT
[RtsWin]::GetWindowRect($central, [ref]$r) | Out-Null
$wC = $r.R - $r.L
$hC = $r.B - $r.T

$flags = 0x0045                 # SWP_NOSIZE | SWP_NOZORDER | SWP_SHOWWINDOW
[RtsWin]::SetWindowPos($central, [IntPtr]::Zero, $wa.L,       $wa.T,       0, 0, $flags) | Out-Null
[RtsWin]::SetWindowPos($rail,    [IntPtr]::Zero, $wa.L + $wC, $wa.T,       0, 0, $flags) | Out-Null
[RtsWin]::SetWindowPos($inter,   [IntPtr]::Zero, $wa.L,       $wa.T + $hC, 0, 0, $flags) | Out-Null

foreach ($p in @(@("central", $central), @("railway", $rail), @("intersections", $inter))) {
    $q = New-Object RtsWin+RECT
    [RtsWin]::GetWindowRect($p[1], [ref]$q) | Out-Null
    "{0,-13} at ({1},{2})  {3}x{4}" -f $p[0], $q.L, $q.T, ($q.R - $q.L), ($q.B - $q.T)
}
