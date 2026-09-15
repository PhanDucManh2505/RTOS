# EEET2588 Real-Time Systems — Traffic Light Control System

Team ANK — Phan Duc Manh (S4124156), Mai Quy Anh (S4118973), Vu Minh Khanh (S4117146)

Proof-of-concept implementation on QNX Neutrino across three nodes, written in C
using QNX native message passing, pulses, POSIX timers, pthreads,
mutexes with priority inheritance, and condition variables.

---

## 1. What runs where

| Node | Hostname | Processes | Role |
|---|---|---|---|
| VM1 | `vm1` | `central` | Control room. Monitors everything, sends pattern and override commands, can stop every train and acknowledges gate faults. Never drives a lamp, a train or a gate. |
| VM2 | `vm2` | `intersection_i1` … `intersection_i6`, `inter_panel` | Six independent local controllers, one process each. Each owns its own lamps and its own timing. `inter_panel` is VM2's screen and keyboard: it shows what the six print in a table that scrolls with the mouse wheel, and its keys stand in for the pedestrian push buttons and the car loop detectors. It controls nothing; the six run the same without it. |
| VM3 | `vm3` | `railway` | Three level crossings, the boom gates, the train line and the fault reporting. |

Crossings are shared between pairs:

```
X1 between I1 and I2     X2 between I3 and I4     X3 between I5 and I6
```

Trains run that line north to south and back on double track, and can only
enter at either end: a southbound (NS) train enters at X1 on track A and is
detected at X2 60 s later and at X3 90 s after that; a northbound (SN) train
enters at X3 on track B and runs the other way (`rts_timing.h`).

Each intersection is a crossroads of a north-south road and an east-west road.
The two north-south roads, R1 and R2, run parallel to the railway, one on each
side of it; the three east-west roads, R3, R4 and R5, cross it at X1, X2 and X3,
about 50 m from the stop line on either side. Because the crossing sits
*between* the two controllers of a pair, it is on opposite arms of them: the
tracks cross the **east** arm of I1, I3 and I5 and the **west** arm of I2, I4
and I6.

| Intersection | North-south road (phases A, B) | East-west road (phases C, D) | Tracks cross its | Crossing |
|---|---|---|---|---|
| I1 | R1 | R3 | east arm | X1 |
| I2 | R2 | R3 | west arm | X1 |
| I3 | R1 | R4 | east arm | X2 |
| I4 | R2 | R4 | west arm | X2 |
| I5 | R1 | R5 | east arm | X3 |
| I6 | R2 | R5 | west arm | X3 |

Each `intersection_iN.c` sets its row of this table in its `inter_cfg_t`
(`road_ns`, `road_ew`, `rail_arm`, `xing_id`); central shows the roads on the
line under its intersection table.

```
        N                      the tracks cross the east arm of I1:
        |                      WE and SE end in that arm, so a train
   W ---+--- E  ===  tracks    holds those two red. EW and EN come out
        |                      of it and are what empties the road in
        S                      front of the gates.
```

Node names are read from the environment (`RTS_NODE_CENTRAL`, `RTS_NODE_INTER`,
`RTS_NODE_RAIL`, defaults `vm1` / `vm2` / `vm3`), so moving a process to a
different machine never requires a rebuild.

---

## 2. Source layout

```
rts_traffic/
  Makefile                     builds all 9 executables into bin/
  common/
    rts_proto.h  rts_proto.c   message types, structs, enums, name helpers
    rts_names.h  rts_names.c   node names, service names, /net path building
    rts_timing.h               every timing value, plus the speed factor
    rts_color.h  rts_color.c   ANSI colour, fixed-width lamp strings, full-screen panels
    rts_util.h   rts_util.c    threads, channels, timers, timed MsgSend, links
    rts_log.h    rts_log.c     asynchronous logging to /fs
    rts_safety.h rts_safety.c  the conflicting-movement interlock
    intersection_core.h/.c     the whole local controller
  src/
    central.c                  VM1
    intersection_i1.c … i6.c   VM2, one main() and one executable each
    inter_panel.c              VM2, the screen and the keys beside the six
    railway.c                  VM3
  scripts/
    rts_tile.bat               double-click on Windows: start all three nodes in three windows, real time
    rts_tile_s5.bat            the same at the demonstration speed (-s 5)
    rts_stop.bat               stop every RTS process on all three nodes
    start_qnet.sh              bring Qnet up on a target
    run_vm1.sh  run_vm3.sh     VM1 central, VM3 railway
    run_vm2_panel.sh           VM2 with its panel, which reads the six's output from a FIFO
    deploy_via_vm1.sh          push a build from VM1 to VM2 and VM3 over Qnet, when only VM1 takes key login
```

The six intersection files each have their own `main()` and build into their own
separate executable, exactly as required. Their shared behaviour lives once in
`intersection_core.c`; each `intersection_iN.c` supplies only the identity and the
crossing it sits next to. Six copies of the same
900-line state machine would be six places for the same bug to hide.

---

## 3. Thread and priority design

**Local controller** (one process per intersection, five threads):

| Prio | Thread | Blocks on | Job |
|---|---|---|---|
| 21 | `t_preempt` | `rts_iN_evt` channel | Crossing state from the railway node, plus the crossing watchdog. |
| 15 | `t_phase` | private channel | Timer pulses, runs the light state machine, the **only** writer of the lamps. |
| 12 | `t_srv` | `rts_iN` channel | Commands from the control room and presses from the VM2 panel; validates, accepts or refuses. |
| 10 | `t_report` | condition variable | Sends a status message on every lamp change. |
| 5 | log writer | condition variable | Drains the log ring buffer into `/fs`. |

The order is the order of consequence: a train reacted to late is a collision, a
phase that overruns is a queue, a status message that arrives late is still only
a message. Putting communications at the bottom means that if the link to the
control room stalls, that thread simply stops being scheduled and the
intersection keeps cycling as though nothing had happened.

Each thread owns its own channel. A single QNX channel hands a message to
whichever receive-blocked thread has the highest priority; it cannot filter by
message type, so "one channel, three threads each waiting for its own work" is
not something QNX can do. Separate channels give each concern its own queue and
keep a burst of status traffic from delaying a reaction to a train.

**Railway** (`railway`): three `t_xing` threads at prio 20 (one crossing each),
`t_srv` 12, `t_train` 10, `t_op` 8, `t_display` 6, log writer 5.

**Central** (`central`): `t_srv` 12, `t_hb` 10, `t_op` 8, `t_display` 6, log writer 5.

**VM2 panel** (`inter_panel`): `t_lines` 10, `t_op` 8, `t_display` 6, log writer 5. `t_lines` sits above the rest so a controller is never kept waiting on a full FIFO.

---

## 4. IPC map

| Link | Mechanism | Why |
|---|---|---|
| central ↔ each intersection (commands, heartbeat, status) | native message passing over Qnet | Synchronous: one call tells the sender whether the message was accepted, refused, or never arrived. The reply doubles as proof the receiver is alive. |
| railway → intersections and central (crossing state) | native message passing over Qnet, pushed on change and repeated once a second | Qnet cannot carry shared memory, so the crossing state is pushed rather than published. See §8. |
| central → railway (stop or release the trains, acknowledge a gate fault) | native message passing over Qnet | The only way the control room reaches the railway, and it cannot drive a train or a gate through it. No traffic-light process is on this path. |
| VM2 panel → each intersection (push button, car on a loop) | native message passing, local on VM2 | The same call as a command: the reply says whether the controller took the press. The panel stands in for equipment beside the road, so it runs on the intersections' node. |
| six intersections → VM2 panel (their screen output) | a FIFO, one whole line per write | Only text for the operator. The controllers ignore `SIGPIPE`, so a panel that stops costs the picture, never the lights. |
| POSIX timers → `t_phase` | pulses | Non-blocking and fixed size. A timer must never hold up the thread it fires into. |
| `t_preempt` / `t_srv` → `t_phase` ("look again") | pulses | Same reason: the sender is never blocked by the state machine. |
| any thread → log writer | ring buffer, mutex + condition variable | Asynchronous, so nothing waits on the file system. |
| `intersection_state` inside a controller | mutex with `PTHREAD_PRIO_INHERIT` | Three threads share one block of data; priority inheritance keeps inversion bounded. |

Every server registers with `name_attach()` and is found by name:
`rts_central`, `rts_i1` … `rts_i6`, `rts_i1_evt` … `rts_i6_evt`, `rts_railway`.
A remote client opens `/net/<node>/dev/name/local/<service>`; the `MsgSend()`
call itself is identical either way.

### Message catalogue

| Type | From → To | Sent when | Carries |
|---|---|---|---|
| `MSG_HEARTBEAT` | central → intersection, railway | once a second | the random cars switch; the reply is the point |
| `MSG_SET_PATTERN` | central → intersection | operator or schedule | pattern id, four green times |
| `MSG_OVERRIDE` | central → intersection | dignitary or emergency | phase to hold green, seconds of green to give it |
| `MSG_PED_BUTTON` | VM2 panel → intersection | someone presses a push button (`p` `P`) | which crossing |
| `MSG_CAR_REQUEST` | VM2 panel → intersection | a car reaches a loop detector (`!` `@` `#` `$`) | which phase |
| `MSG_STATUS` | intersection → central | **every lamp change** | all lamps, phase, pattern, hold, link state |
| `MSG_XING_STATE` | railway → 2 intersections + central | on change, and every second | CLEAR / WARNING / CLOSED / FAULT, tracks, gates, train signal |
| `MSG_RAIL_CMD` | central → railway | operator stops or releases the trains; every gate fault report | stop, release, or fault acknowledged at a crossing |
| `MSG_GATE_FAULT` | railway → central | a gate misses its position | crossing, gate |

Replies are never separate messages: an accept or refuse is the `MsgReply()` to
the command, which is why the operator always learns the outcome.

---

## 5. Safety logic

Two rules drive everything.

**No two conflicting movements may show green at once.** `rts_lamps_safe()` in
`rts_safety.c` is the single place that decides. Movements are grouped by phase,
and green or amber counts as occupying the intersection; if two different phase
groups are ever showing anything, the commit is refused and the controller falls
back to all-red and says so in red on the terminal. Every single lamp change
goes through `lamps_commit()`, so the check cannot be bypassed.

**No vehicle may be driven towards a closing gate.** Movements are named
"from arm -> to arm", so the rule is one line: while a train is expected, every
movement that **ends** in the arm the tracks cross is held red
(`rts_rail_block_mask()`). That is one through movement and one right turn off
the other road — at I1, `WE` and `SE` — and they sit in two different phases, so
blocking by phase alone would have left the right turn running straight into the
queue in front of the gates. Everything else keeps running, including the
movements that come **out** of that arm, because they drive away from the
tracks. `rts_lamps_safe()` refuses to commit a green for a blocked movement, so a
missed mask ends as all-red rather than as a car on the crossing.

**A green that has already started still ends properly.** Amber (4 s) and
all-red (2 s) are compile-time constants and nothing may shorten them — not a
pattern command, not an override, not pre-emption. A movement caught mid-green
when the train is announced is given its whole amber; it is the pre-emption
budget (4 s amber + 2 s all-red + 21 s clearing green < 30 s of warning) that
makes room for it. A train does not wait out a pedestrian's 18 s either; the
crossing flashes through that same 4 s amber. The clearing green itself releases **only** the movement that comes out of
the rail-side arm, since the one driving towards the gates would be refilling
the road the clearance exists to empty.

**Nothing is crossed on trust.** A crossing that reports `FAULT`, or that has
gone quiet for longer than the watchdog, is treated exactly like a crossing with
a train on it. A controller **starts** in that held state and only releases the
movements into the rail-side arm once the railway has reported `CLEAR`.

Priority of intent, highest first: railway pre-emption → operator override →
selected pattern → the normal cycle. An override holds any one of the four
phases for 40 s of green: the clock runs only while that phase shows green,
so a train that cuts it pauses the clock rather than eating it. An override
for a phase that drives into the rail-side arm (C or B, depending on the
side) gives way completely: while the crossing is not clear the normal
rail hold runs and the override waits (central shows `OVR C wait`); it is
held only once the train has gone. A phase that never touches that arm
(A, D) resumes straight after the clearing green.

**Off peak (SENSOR) there is no cycle.** A green stays up indefinitely and is
only given up when another phase has asked for it and whoever asked for this
one has had their time: 18 s for a pedestrian, 5 s for a car to move off. It
can then go to any
phase, not only the next one: pedestrians first, the earliest button, then
vehicles, the earliest car on its loop. Cars turn up at random, about one per
intersection every 30 s, unless the control room turns that off (`r`); `!` `@`
`#` `$` on the VM2 panel place one by hand.

Commands are validated locally before use. `check_pattern()` refuses a green
below the pedestrian minimum, above the maximum, or a cycle that is too long,
and the reason travels back in the reply.

---

## 6. Timing

All values in `rts_timing.h` are the real-world seconds derived in section 7 of
the design report. At run time they are divided by a speed factor so a full cycle
fits in a demonstration; every relationship between the numbers is preserved, so
the safety argument holds at any speed.

```
-s 1     real time, 90 s cycle
-s 5     default, 18 s cycle
-s 10    9 s cycle, good for a quick sanity check
```

Amber 4 s · all-red 2 s · pedestrian hold 18 s, car 5 s (SENSOR) · greens 20/13/20/13 ·
cycle 90 s · train warning 30 s · heartbeat 1 s · offline after 3 s ·
trains X1 ↔ X2 60 s, X2 ↔ X3 90 s, one every 2 min (rush) or 4 min (off peak) ·
a random car about every 30 s (SENSOR) · no `MsgSend` blocks longer than 200 ms.

---

## 7. Build

### On the command line

```bash
# Windows: run the SDP environment script first
#   "C:\Users\<you>\qnx800\qnxsdp-env.bat"
cd rts_traffic
make
```

If `qcc -V` does not list `gcc_ntox86_64`, pass the one it does list:

```bash
make VARIANT=gcc_ntox86_64_gpp
```

### In Momentics (`ide-8.0.3-workspace`)

1. Copy the `rts_traffic` folder into your workspace directory.
2. **File → New → Project… → C/C++ → Makefile Project with Existing Code**.
3. *Existing Code Location*: the `rts_traffic` folder. *Toolchain*: **QNX
   Qcc**. Finish.
4. **Project → Build All**. The nine executables appear in `bin/`.
5. Add the three targets in the **QNX Target Navigator** (right-click → *New QNX
   Target*), one per VM. `qconn` must be running on each — note that restarting
   `io-pkt` for Qnet also kills `qconn`, so `start_qnet.sh` restarts it.

A Makefile project is used rather than nine managed projects because the brief
asks for one project containing separate programs, and nine separate build
configurations would be nine places to keep in step.

---

## 8. Deviations from the Initial Design Report

Two things changed when the deployment moved to *central / six intersections /
railway* on three machines. Both belong in the Implementation Note.

1. **Crossing state is a message, not shared memory.** The report published it in
   a shared region read by the two controllers either side. Qnet does not carry
   shared memory, and the railway now runs on a different node from the
   intersections. The railway therefore pushes `MSG_XING_STATE` on every change
   and repeats it once a second, and each controller runs a watchdog that treats
   silence as `FAULT`. The safety property is unchanged and is now stronger: a
   lost node is detected, not merely tolerated. The rule that no traffic-light
   process may command a crossing still holds, because the railway is the sender
   on this path and no message travels the other way.

2. **Condition variable replaced by pulses for wake-ups inside a controller.** A
   thread cannot wait on a channel and on a condition variable at the same time.
   `t_phase` blocks only on its channel; other threads nudge it with a pulse. The
   mutex still protects the shared state and still uses priority inheritance, and
   the condition variable is still used where it fits — between `t_report` and
   the state machine, and in the logger.

---

## 9. Network setup (once per VM)

On each target, over SSH:

```sh
# 1. hostname must be unique and must match RTS_NODE_*
hostname vm1                 # vm2 on the second, vm3 on the third

# 2. bring Qnet up (detached: io-pkt restarts and SSH drops for a moment)
on -d -s /tmp/manh/start_qnet.sh 192.168.56.110 vm1

# 3. reconnect and check
ls /net                      # must list vm1 vm2 vm3
```

The root filesystem of an `mkqnximage` target is in RAM, so the hostname and
anything else set by hand is lost on reboot. Re-run step 1 and 2 after every
boot. `/fs` is the persistent one and is where the logs go.

If Windows complains that the remote host identification has changed:
`ssh-keygen -R 192.168.56.110`.

### Colour in PowerShell

Windows Terminal handles ANSI already. In the older console host, enable it once:

```powershell
Set-ItemProperty HKCU:\Console VirtualTerminalLevel -Type DWORD 1
```

Then open one tab per VM:

```powershell
ssh root@192.168.56.110      # VM1, control room
ssh root@192.168.56.111      # VM2, the six intersections
ssh root@192.168.56.112      # VM3, railway
```

Set `RTS_COLOR=0` on the target if you want to capture plain text.

---

## 10. Deploy and run

Only VM1 accepts key login, so a build reaches VM2 and VM3 through VM1:
`deploy_via_vm1.sh` runs on VM1, keeps the binaries it replaces in `bak_<tag>`
on every node and copies the new ones over Qnet. Stop the system first
(`scripts\rts_stop.bat`); a running binary cannot be overwritten.

```powershell
ssh root@192.168.56.110 mkdir -p /tmp/manh/new
scp bin\* scripts\deploy_via_vm1.sh scripts\run_vm2_panel.sh root@192.168.56.110:/tmp/manh/new/
ssh root@192.168.56.110 sh /tmp/manh/new/deploy_via_vm1.sh my_tag
```

On Windows, double-click `scripts\rts_tile_s5.bat` to start all three nodes at
the demonstration speed (`-s 5`, an 18 s cycle), or `scripts\rts_tile.bat` for
real time; `scripts\rts_stop.bat` stops everything. Both open one window per
node through VM1 and start them in the order below.

By hand, start in this order — VM3 first so the intersections learn the
crossing state straight away, VM1 last:

```sh
# VM3
cd /tmp/manh && ./run_vm3.sh

# VM2: the six controllers and their panel
cd /tmp/manh && ./run_vm2_panel.sh

# VM1
cd /tmp/manh && ./run_vm1.sh
```

Any order works. Started out of order, the intersections simply sit in the
fail-safe hold, running only the road parallel to the tracks, until the railway
reports in.

### Keys — central (VM1)

| Key | Effect |
|---|---|
| `1`–`6` | pick one intersection · `0` all six |
| `f` `s` `u` | pattern: fixed (peak) · sensor-driven (off-peak) · updated, sending the green times on the UPDATED row. Every intersection starts on fixed. |
| `[` `]` `-` `+` | UPDATED row: pick phase A–D · take 1 s off its green · add 1 s. The row starts at the programmed 20/13/20/13 and nothing is sent until `u`. A green under 8 s (or over 60 s, or a cycle over 150 s) shows red, and the intersection refuses the whole update and keeps running what it had. Accepted times stay in force until another pattern is chosen, and the PATTERN column shows them as A/B/C/D. |
| `A` `B` `C` `D` `x` | override: hold that phase green at the target for 40 s of green · cancel. The PATTERN column shows `OVERRIDE C`, or `OVR C wait` while a train has the crossing and the hold is postponed. Pressing another letter ends the held phase with its full amber and all-red, then holds the new one; pressing the same letter again gives it a fresh 40 s. |
| `r` | random cars on or off at all six intersections, for testing (shown on the COMMAND row). The pedestrian buttons and the car loops are on the VM2 panel. |
| `e` `E` | tell the railway to stop every train (something is wrong on the line) · let them run again. The RAILWAY row shows `trains STOPPED` or `trains running`. Central cannot put a train on the line or move a gate: those keys are on the railway. Every gate fault the railway reports is acknowledged automatically. |
| `q` | quit |

### Keys — intersection panel (VM2)

The VM2 window is a panel like the railway's. Its table holds the last 500
lines the six controllers printed; the mouse wheel scrolls it back, and
scrolling down to the bottom follows the newest lines again. A line from the
intersection the keys act on is marked in its first column.

The panel's target is its own: picking an intersection here does not change the
control room's target, nor the other way round. A press is a message to the
controller, which decides what to do with it, so its effect shows up on
central's table like any other lamp change.

| Key | Effect |
|---|---|
| `1`–`6` | pick one intersection · `0` all six (TARGET row) |
| `p` `P` | a pedestrian presses the button on the north arm (phase C) · on the east arm (phase A) |
| `!` `@` `#` `$` | a car pulls up at the loop of phase A · B · C · D |

The SELECT row shows an intersection in red when it did not answer the last
press. There is no `q`: the panel stops with the rest of VM2
(`scripts\rts_stop.bat`), and if it stops on its own the six carry on without a
picture.

### Keys — railway (VM3)

| Key | Effect |
|---|---|
| `1` `2` `3` | select the crossing the gate keys act on |
| `a` `b` | a train enters at X1 and runs X1 → X2 → X3 on track A · enters at X3 and runs X3 → X2 → X1 on track B. There is no way to put a train on X2. |
| `n` `r` `o` | timetable: no trains (late night) · rush hour, a train every 2 min · off peak, every 4 min. The railway starts with none; a timetable picked later sends its first train one interval after the key. Timetabled trains alternate NS, SN, NS … |
| `f` `c` | inject a gate fault · clear it |
| `q` | quit |

---

## 11. Demonstration procedure

Run at `-s 5`. Each scenario is one claim you can be asked to back up.

| # | Scenario | Do this | What must happen | Grade band |
|---|---|---|---|---|
| 1 | Safe sequence | Straight after start (every intersection starts on FIXED), or central `0` then `f`; watch any intersection for two cycles | A → B → C → D, every green followed by 4 s amber then 2 s all-red. No two phase groups ever green together. | Pass |
| 2 | Distributed | `pidin` on VM2 shows the six controllers (and `inter_panel`); central shows all six | Six independent controller processes on one node, three nodes joined by Qnet | Pass |
| 3 | Railway pre-emption | Railway `a` (a train enters at X1) | I1 and I2 cut the current green, run amber + all-red in full, then a clearing green for the movement that comes out of the rail-side arm only (`EW` at I1). Both start that clearing green 6 s after the warning, the one already in amber or all-red waiting all red until then. The clearing green is 21 s on every pattern, so I1 and I2 show `RAIL HOLD` on central together even when one runs FIXED and the other SENSOR. The same train reaches X2 60 s later (I3, I4) and X3 90 s after that (I5, I6). | Pass |
| 4 | Two trains | Railway `b`, then `a` 30 s later | They meet at X2, one on each track: its gates stay down while either track is occupied and only rise when both are clear | HD |
| 5 | Off peak holds, any phase next | Central `s`, `r` (random cars off), then `$` on the VM2 panel | The intersection goes straight to phase D, whatever phase it was on, and then stays on D indefinitely: nothing asks, so nothing changes. | Credit |
| 6 | Pedestrians first | Still with random cars off, press `P`, `@`, `p` quickly on the VM2 panel | A for the east pedestrian, held 18 s, then C for the north one, 18 s, and only then B for the car that asked before them. The crossings walk with their phase and flash through its amber. | Credit |
| 7 | Monitoring | Watch the central dashboard | A status message on every lamp change, all six intersections and three crossings live | Credit |
| 8 | Pattern change | Central `0` then `f`, then `s` | All six accept and change at the end of the cycle | Distinction |
| 9 | Updateable pattern | Central `u` | New green times supplied by the control room, validated locally, then applied | HD |
| 10 | Command refused | Central: take a phase under 8 s on the UPDATED row, then `u` | Refused with "green below the safe minimum". The lights and the green times in force do not change. | HD |
| 11 | Override | Central `1` `C`, wait 40 s; then `B`, then `x` | Phase C is held green for 40 s of green and expires on its own; B is held next; `x` drops the hold and the normal cycle resumes | HD |
| 12 | Pre-emption outranks override | `1` `C`, then send a train | C is cut with its full amber and all-red, the clearing green and the rail hold run as if there were no override (`OVR C wait`), and C is held again, for the green it is still owed, only once X1 is clear | HD |
| 13 | Boom gate fault | Railway `f` | Gate never reaches position → `FAULT`, train given a **red**, control room told and it acknowledges straight away (railway event list), both controllers hold the tracks clear | HD |
| 14 | Central controller fails | `q` on VM1, or `slay central` | All six keep cycling on their last valid pattern, mark the link down within 3 s, and write status to `/fs`. Restart it: they resync. | HD |
| 15 | Node fails | On VM3, `slay railway` | Within the watchdog, both controllers on each crossing treat it as `FAULT` and hold. Restart it: they release. | HD |
| 16 | One intersection fails | On VM2, `slay intersection_i3` | Central marks I3 down after three missed heartbeats; I4 and everything else carry on | HD |
| 17 | Logs and test vectors | `cat /fs/rts/central.log` | Timestamped record of every state change on every node, including `mv=` — all eight vehicle lamps of that instant | HD |
| 18 | Cut by movement, not by phase | Send a train, then read the I1 lamp line | While the crossing is not clear, `we` and `se` (the two movements that end in the rail-side arm) are red in every phase, `EW` and `EN` still run, and the cycle still steps A → B → C → D at its full 90 s | HD |
| 19 | Incident on the line | Railway `a`, then central `e` a few seconds later; after a while central `E` | While stopped every crossing shows the trains **red**, the train does not reach X2 and railway `a` or `b` is refused. After `E` the train reaches X2 60 s after it entered plus the time it was stopped. | HD |

For scenario 14, `q` on the central terminal is the clean way; `slay central`
from a second SSH session proves the same thing without a graceful shutdown.

---

## 12. Where to look in the code during Q&A

| Question | File and function |
|---|---|
| "Show me your IPC" | `rts_util.c` → `rts_link_send()`, `rts_server_housekeeping()` |
| "Show me a pulse" | `intersection_core.c` → `MsgSendPulse(... PULSE_XING_CHANGED ...)` |
| "Show me a timer" | `rts_util.c` → `rts_timer_new()`, used in `t_phase` |
| "Where is the safety check" | `rts_safety.c` → `rts_lamps_safe()`, called from `lamps_commit()` |
| "How do you avoid deadlock" | `intersection_core.c` → every server replies before it acts; no message is sent from a reply path |
| "How do you avoid priority inversion" | `rts_util.c` → `rts_mutex_init()` with `PTHREAD_PRIO_INHERIT` |
| "What if a node disappears" | `rts_util.c` → `TimerTimeout()` before every `MsgSend()`; watchdog in `t_phase` |
| "What exactly does a train stop" | `rts_safety.c` → `rts_rail_block_mask()`, applied in `fsm_enter()` and re-checked in `lamps_commit()` |
| "Why three threads" | header comment in `intersection_core.h` |
