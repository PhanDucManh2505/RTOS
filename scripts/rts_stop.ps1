# rts_stop.ps1 - stop every RTS process on all three nodes (through VM1 and Qnet).
param([string]$VM1 = "192.168.56.110")

ssh -o BatchMode=yes root@$VM1 'for p in central railway intersection_i1 intersection_i2 intersection_i3 intersection_i4 intersection_i5 intersection_i6; do slay -f $p; on -f vm2 slay -f $p; on -f vm3 slay -f $p; done 2>/dev/null; echo all RTS processes stopped'
