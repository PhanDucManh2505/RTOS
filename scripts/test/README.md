# Automated validation on the three VMs

`rts_tests.py` drives the real system on vm1, vm2 and vm3 and checks every
scenario of README section 11 plus the timing and railway rules, printing
PASS or FAIL per check (the check names are in Vietnamese, so they can go
straight into the report).

How it works: `rtst.sh` is copied to `/tmp/manh/rtst.sh` on vm1 and starts
the eight processes the same way `run_vm*.sh` does, but central and railway
read their keys from a FIFO so a test can "press" keys, and every screen is
captured as plain text (`RTS_COLOR=0`). Test runs log to `/tmp/manh/t` on
each node, so the demo logs in `/tmp/manh` are left alone.

Three things are read back:

* the **screens** of central and railway, parsed column by column;
* the **logs**, one line per lamp change, including `mv=` - all eight
  vehicle lamps at that instant, which is what the railway checks use;
* the **corridor region** on vm2 (`/dev/shmem/rts_corridor`), which is the
  only clock all six controllers share and therefore the one place the
  12 s green-wave offset can be measured exactly.

## Run

From this folder on Windows (needs Python 3 and key login to vm1):

```powershell
scp rtst.sh root@192.168.56.110:/tmp/manh/rtst.sh
ssh root@192.168.56.110 chmod +x /tmp/manh/rtst.sh
python rts_tests.py              # every group, speed 5, about 15 minutes
python rts_tests.py g2           # one group: railway pre-emption
$env:RTS_SPEED=1; python rts_tests.py g2   # the same at real time
```

Groups: `g1` start-up, nodes, safe sequence, monitoring · `g2` railway
pre-emption, the movements a train forbids, two trains · `g3` pedestrians ·
`g4` targeting, refused and updated patterns · `g5` FIXED timing and the
green wave · `g6` override · `g7` gate faults and gate commands · `g8`
central, railway and intersection failures.

`RTS_FIXED_CYCLES=8` sets how many cycles g5 watches, `RTS_OUT=<dir>` where
the run is saved. Results and the raw logs of each run land in
`run_<date>_<time>/`. The tests stop every RTS process when they finish, so
start the demo again with `rts_tile.ps1` afterwards.

## What the railway checks prove

`T04h` no movement that ends in the rail-side arm is ever released while the
crossing is not clear · `T04m` the clearing green shows only the movement
that comes out of that arm · `T04n` every other movement of the phase still
runs · `T04r` a movement caught mid-green still gets its whole 4 s amber ·
`T04o`/`T04p` under a long hold the cycle still steps A→B→C→D and still
measures 90 s · `T11b` the pair is exactly 12 s apart.

The tests open one SSH connection per command, and `sshd` on the target
occasionally sits on a new connection for minutes; `ssh()` therefore retries
a timed-out command once and says so on screen. If a whole group dies with
`TimeoutExpired`, check the VMs are still powered on before suspecting the
code.

Two notes on reading a failure. The VM host stalls for a few tens of
milliseconds now and then; at speed 5 that is a few tenths of a *design*
second, so a single green measuring 12.8 instead of 13 is the host, not the
controller (the checks allow one such outlier and say so). And the 3 s
watchdogs become 0.6 s at speed 5, so a stall can make a controller declare
the crossing unreadable; the test reports that separately as `T04j+`.

## Deploy a new build through vm1

Only vm1 accepts key login, so `deploy_via_vm1.sh` copies to vm2 and vm3
over Qnet and keeps the binaries it replaces in `bak_<tag>` on every node:

```powershell
ssh root@192.168.56.110 mkdir -p /tmp/manh/new
scp ..\..\bin\* deploy_via_vm1.sh root@192.168.56.110:/tmp/manh/new/
ssh root@192.168.56.110 sh /tmp/manh/new/deploy_via_vm1.sh my_tag
```

Stop the system first (`ssh root@192.168.56.110 /tmp/manh/rtst.sh stop`);
a running binary cannot be overwritten.
