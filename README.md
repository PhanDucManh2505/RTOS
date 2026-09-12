# EEET2588 Real-Time Systems — Traffic Light Control System

Team ANK — Phan Duc Manh (S4124156), Mai Quy Anh (S4118973), Vu Minh Khanh (S4117146)

Proof-of-concept implementation on QNX Neutrino across three nodes, written in C
using QNX native message passing, pulses, POSIX timers, shared memory, pthreads,
mutexes with priority inheritance, and condition variables.

---

## 1. What runs where

| Node | Hostname | Processes | Role |
|---|---|---|---|
| VM1 | `vm1` | `central` | Control room. Monitors everything, sends pattern / override / gate commands. Never drives a lamp. |
| VM2 | `vm2` | `intersection_i1` … `intersection_i6` | Six independent local controllers, one process each. Each owns its own lamps and its own timing. |
| VM3 | `vm3` | `railway` | Three level crossings, the boom gates, the train line and the fault reporting. |

Crossings are shared between pairs:

```
X1 between I1 and I2     X2 between I3 and I4     X3 between I5 and I6
```

Each intersection is a crossroads of two roads: R1 on the north-south arms and
R3 on the east-west ones. The railway runs parallel to R1 and cuts R3 about 50 m
from the stop line. Because the crossing sits *between* the two controllers of a
pair, it is on opposite arms of them: the tracks cross the **east** arm of I1, I3
and I5 and the **west** arm of I2, I4 and I6 (`inter_cfg_t.rail_arm`).

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
  Makefile                     builds all 8 executables into bin/
  common/
    rts_proto.h  rts_proto.c   message types, structs, enums, name helpers
    rts_names.h  rts_names.c   node names, service names, /net path building
    rts_timing.h               every timing value, plus the speed factor
    rts_color.h  rts_color.c   ANSI colour, fixed-width lamp strings, full-screen panels
    rts_util.h   rts_util.c    threads, channels, timers, timed MsgSend, links
    rts_log.h    rts_log.c     asynchronous logging to /fs
    rts_safety.h rts_safety.c  the conflicting-movement interlock
    rts_shm.h    rts_shm.c     the VM2 corridor region (seqlock)
    intersection_core.h/.c     the whole local controller
  src/
    central.c                  VM1
    intersection_i1.c … i6.c   VM2, one main() and one executable each
    railway.c                  VM3
  scripts/
    start_qnet.sh              bring Qnet up on a target
    run_vm1.sh  run_vm2.sh  run_vm3.sh
    deploy.ps1                 build on Windows and scp to all three VMs
```

The six intersection files each have their own `main()` and build into their own
separate executable, exactly as required. Their shared behaviour lives once in
`intersection_core.c`; each `intersection_iN.c` supplies only the identity, the
crossing it sits next to, and the green-wave offset. Six copies of the same
900-line state machine would be six places for the same bug to hide.

---

## 3. Thread and priority design

**Local controller** (one process per intersection, five threads):

| Prio | Thread | Blocks on | Job |
|---|---|---|---|
| 21 | `t_preempt` | `rts_iN_evt` channel | Crossing state from the railway node, plus the crossing watchdog. |
| 15 | `t_phase` | private channel | Timer pulses, runs the light state machine, the **only** writer of the lamps. |
| 12 | `t_srv` | `rts_iN` channel | Commands from the control room; validates, accepts or refuses. |
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

---

## 4. IPC map

| Link | Mechanism | Why |
|---|---|---|
| central ↔ each intersection (commands, heartbeat, status) | native message passing over Qnet | Synchronous: one call tells the sender whether the message was accepted, refused, or never arrived. The reply doubles as proof the receiver is alive. |
| railway → intersections and central (crossing state) | native message passing over Qnet, pushed on change and repeated once a second | Qnet cannot carry shared memory, so the crossing state is pushed rather than published. See §8. |
| central → railway (gate commands) | native message passing over Qnet | The only way into a crossing. No traffic-light process is on this path. |
| POSIX timers → `t_phase` | pulses | Non-blocking and fixed size. A timer must never hold up the thread it fires into. |
| `t_preempt` / `t_srv` → `t_phase` ("look again") | pulses | Same reason: the sender is never blocked by the state machine. |
| intersection ↔ intersection (green wave) | shared memory with a seqlock | All six run on VM2, so a region works and needs no messages at all. |
| any thread → log writer | ring buffer, mutex + condition variable | Asynchronous, so nothing waits on the file system. |
| `intersection_state` inside a controller | mutex with `PTHREAD_PRIO_INHERIT` | Three threads share one block of data; priority inheritance keeps inversion bounded. |

Every server registers with `name_attach()` and is found by name:
`rts_central`, `rts_i1` … `rts_i6`, `rts_i1_evt` … `rts_i6_evt`, `rts_railway`.
A remote client opens `/net/<node>/dev/name/local/<service>`; the `MsgSend()`
call itself is identical either way.

### Message catalogue

| Type | From → To | Sent when | Carries |
|---|---|---|---|
| `MSG_HEARTBEAT` | central → intersection, railway | once a second | nothing; the reply is the point |
| `MSG_SET_PATTERN` | central → intersection | operator or schedule | pattern id, four green times, offset |
| `MSG_OVERRIDE` | central → intersection | dignitary or emergency | phase to hold green, timeout |
| `MSG_PED_BUTTON` | central → intersection | operator injects a press | which crossing |
| `MSG_QUERY_STATE` | central → intersection | on demand | reply carries the full state |
| `MSG_STATUS` | intersection → central | **every lamp change** | all lamps, phase, pattern, hold, link state |
| `MSG_XING_STATE` | railway → 2 intersections + central | on change, and every second | CLEAR / WARNING / CLOSED / FAULT, tracks, gates, train signal |
| `MSG_GATE_CMD` | central → railway | operator acts on a gate | crossing and action |
| `MSG_GATE_FAULT` | railway → central | a gate misses its position | crossing, gate, train stopped |

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
makes room for it. A pedestrian clearance that has started always runs to the
end. The clearing green itself releases **only** the movement that comes out of
the rail-side arm, since the one driving towards the gates would be refilling
the road the clearance exists to empty.

**Nothing is crossed on trust.** A crossing that reports `FAULT`, or that has
gone quiet for longer than the watchdog, is treated exactly like a crossing with
a train on it. A controller **starts** in that held state and only releases the
movements into the rail-side arm once the railway has reported `CLEAR`.

Priority of intent, highest first: railway pre-emption → operator override →
selected pattern → the normal cycle.

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

Amber 4 s · all-red 2 s · WALK 6 s · clearance 12 s · greens 20/13/20/13 ·
cycle 90 s · offset 12 s · train warning 30 s · heartbeat 1 s · offline after 3 s ·
no `MsgSend` blocks longer than 200 ms.

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
4. **Project → Build All**. The eight executables appear in `bin/`.
5. Add the three targets in the **QNX Target Navigator** (right-click → *New QNX
   Target*), one per VM. `qconn` must be running on each — note that restarting
   `io-pkt` for Qnet also kills `qconn`, so `start_qnet.sh` restarts it.

A Makefile project is used rather than eight managed projects because the brief
asks for one project containing separate programs, and eight separate build
configurations would be eight places to keep in step.

---

## 8. Deviations from the Initial Design Report

Three things changed when the deployment moved to *central / six intersections /
railway* on three machines. All three belong in the Implementation Note.

1. **Crossing state is a message, not shared memory.** The report published it in
   a shared region read by the two controllers either side. Qnet does not carry
   shared memory, and the railway now runs on a different node from the
   intersections. The railway therefore pushes `MSG_XING_STATE` on every change
   and repeats it once a second, and each controller runs a watchdog that treats
   silence as `FAULT`. The safety property is unchanged and is now stronger: a
   lost node is detected, not merely tolerated. The rule that no traffic-light
   process may command a crossing still holds, because the railway is the sender
   on this path and no message travels the other way.

2. **Shared memory moved to VM2.** All six controllers are on one node, so the
   corridor region is the natural place for the green-wave offset. Each
   controller writes only its own slot and reads its partner's. A seqlock is used
   instead of a read/write lock: taking a read lock *writes* to the lock object,
   which is impossible from a read-only mapping, so the report's combination of
   `pthread_rwlock_t` and `PROT_READ` could not have worked as written.

3. **Condition variable replaced by pulses for wake-ups inside a controller.** A
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

```powershell
.\scripts\deploy.ps1 -VM1 192.168.56.110 -VM2 192.168.56.111 -VM3 192.168.56.112
```

Start in this order — VM3 first so the intersections learn the crossing state
straight away, VM1 last:

```sh
# VM3
cd /tmp/manh && ./run_vm3.sh

# VM2
cd /tmp/manh && ./run_vm2.sh

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
| `f` `s` `u` | pattern: fixed (peak) · sensor-driven (off-peak) · updated (green times from the control room) |
| `x` | send a deliberately illegal pattern — it must be refused, with a reason |
| `o` `c` | override: hold phase A green · cancel |
| `p` `P` | inject a pedestrian button press (north arm · east arm) |
| `a` `b` | send a train to X1 on track A · track B |
| `g` `G` | inject a boom gate fault at X1 · clear it |
| `d` `n` | force the gates down · return to normal |
| `q` | quit |

### Keys — railway (VM3)

| Key | Effect |
|---|---|
| `1` `2` `3` | select crossing |
| `a` `b` | train on track A · track B of the selected crossing |
| `f` `c` | inject a gate fault · clear it |
| `t` | automatic train generation on/off |
| `n` | peak (a train every 2 min) / night (every 20 min) |
| `q` | quit |

---

## 11. Demonstration procedure

Run at `-s 5`. Each scenario is one claim you can be asked to back up.

| # | Scenario | Do this | What must happen | Grade band |
|---|---|---|---|---|
| 1 | Safe sequence | Watch any intersection for two cycles | A → B → C → D, every green followed by 4 s amber then 2 s all-red. No two phase groups ever green together. | Pass |
| 2 | Distributed | `pidin` on VM2 shows six processes; central shows all six | Six independent processes on one node, three nodes joined by Qnet | Pass |
| 3 | Railway pre-emption | Railway `a` (train on X1 track A) | I1 and I2 cut the current green, run amber + all-red in full, then a clearing green for the movement that comes out of the rail-side arm only (`EW` at I1). Central shows `RAIL HOLD`. | Pass |
| 4 | Two trains | `a` then `b` on the same crossing | Gates stay down while either track is occupied; they only rise when both are clear | HD |
| 5 | Pedestrians | Central `s` (sensor pattern) then `p` | WALK 6 s, flashing clearance 12 s, DON'T WALK. No vehicle movement from another phase runs during it. | Credit |
| 6 | Clearance is never cut short | Press `p`, then send a train while the pedestrian is still crossing | Pre-emption waits for the clearance to finish, then starts. This is the 30 s worst case in the report. | Credit |
| 7 | Monitoring | Watch the central dashboard | A status message on every lamp change, all six intersections and three crossings live | Credit |
| 8 | Pattern change | Central `0` then `f`, then `s` | All six accept and change at the end of the cycle | Distinction |
| 9 | Green wave | With the fixed pattern, compare I1 and I2 cycle starts | They converge on the 12 s offset, adjusted through shared memory with no messages | Distinction |
| 10 | Updateable pattern | Central `u` | New green times supplied by the control room, validated locally, then applied | HD |
| 11 | Command refused | Central `x` | Refused with "green below the safe minimum". The lights do not change. | HD |
| 12 | Override | Central `o`, then `c` | Phase A is held green, then the normal cycle resumes | HD |
| 13 | Pre-emption outranks override | `o`, then send a train | The override is refused or suspended; the train wins | HD |
| 14 | Boom gate fault | Railway `f` | Gate never reaches position → `FAULT`, train given a **red**, control room told, both controllers hold the tracks clear | HD |
| 15 | Central controller fails | `q` on VM1, or `slay central` | All six keep cycling on their last valid pattern, mark the link down within 3 s, and write status to `/fs`. Restart it: they resync. | HD |
| 16 | Node fails | On VM3, `slay railway` | Within the watchdog, both controllers on each crossing treat it as `FAULT` and hold. Restart it: they release. | HD |
| 17 | One intersection fails | On VM2, `slay intersection_i3` | Central marks I3 down after three missed heartbeats; I4 and everything else carry on | HD |
| 18 | Logs and test vectors | `cat /fs/rts/central.log` | Timestamped record of every state change on every node, including `mv=` — all eight vehicle lamps of that instant | HD |
| 19 | Cut by movement, not by phase | Send a train, then read the I1 lamp line | While the crossing is not clear, `we` and `se` (the two movements that end in the rail-side arm) are red in every phase, `EW` and `EN` still run, and the cycle still steps A → B → C → D at its full 90 s | HD |

For scenario 15, `q` on the central terminal is the clean way; `slay central`
from a second SSH session proves the same thing without a graceful shutdown.

Every scenario above is also checked automatically: `scripts/test/` drives the
three VMs over SSH, presses the same keys, and reads back the screens, the
logs and the corridor region. `python scripts/test/rts_tests.py` prints one
PASS or FAIL line per claim; `scripts/test/README.md` explains the groups.

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
| "Where is shared memory" | `rts_shm.c` → the seqlock in `rts_slot_write()` / `rts_slot_read()` |
| "What exactly does a train stop" | `rts_safety.c` → `rts_rail_block_mask()`, applied in `fsm_enter_at()` and re-checked in `lamps_commit()` |
| "How do you prove any of this" | `scripts/test/rts_tests.py`, one check per claim, run against the three VMs |
| "Why three threads" | header comment in `intersection_core.h` |
