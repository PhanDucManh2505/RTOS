#!/usr/bin/env python3
"""rts_validate.py - drive the RTS traffic system on the three QNX VMs and
check every function against README section 11 and the timing rules.

It talks to /tmp/manh/rtst.sh on vm1 over SSH: that harness starts the
eight processes with their keyboards on FIFOs and their screens captured
as plain text, so this script can press keys and read the screens.

    python rts_validate.py [group ...]      groups: g1 .. g8, default all
"""
import json, os, re, shlex, statistics as stats, subprocess, sys, time

VM1 = "root@192.168.56.110"
SPEED = float(os.environ.get("RTS_SPEED", "5"))
OUT = os.environ.get("RTS_OUT", "run_" + time.strftime("%Y%m%d_%H%M%S"))
RESULTS = []

# ----------------------------------------------------------------- plumbing
def ssh(cmd, timeout=120):
    """One command on vm1. A fresh SSH connection now and then takes far
    longer than the command itself - sshd on the target occasionally sits
    on a new connection - so a timeout is retried once rather than
    failing the whole group."""
    argv = ["ssh", "-o", "BatchMode=yes", "-o", "ConnectTimeout=8", VM1, cmd]
    for attempt in (1, 2):
        try:
            return subprocess.run(argv, capture_output=True, text=True,
                                  timeout=timeout).stdout
        except subprocess.TimeoutExpired:
            if attempt == 2:
                raise
            print(f"      (ssh treo, thử lại: {cmd[:60]})", flush=True)
    return ""

def H(args, timeout=120):
    return ssh("/tmp/manh/rtst.sh " + args, timeout)

def key(win, k):
    # Quoted, so keys the shell would read (# $ ! and so on) arrive as typed.
    H(f"key {win} {shlex.quote(k)}")

def real(design_s):
    return design_s / SPEED

def check(tid, name, ok, detail=""):
    RESULTS.append({"id": tid, "name": name, "ok": bool(ok), "detail": str(detail)})
    print(f"{'PASS' if ok else 'FAIL'}  {tid:5} {name}  |  {detail}", flush=True)
    return ok

def note(tid, name, detail):
    RESULTS.append({"id": tid, "name": name, "ok": None, "detail": str(detail)})
    print(f"NOTE  {tid:5} {name}  |  {detail}", flush=True)

def start(noauto=True):
    print(f"\n--- start, speed {SPEED}{' (no auto trains)' if noauto else ''}", flush=True)
    H(f"start {SPEED:g} {'noauto' if noauto else ''}", timeout=180)
    time.sleep(2.5)

def wait_for(fn, timeout, every=0.4):
    end = time.time() + timeout
    last = None
    while time.time() < end:
        last = fn()
        if last:
            return last
        time.sleep(every)
    return last

# ----------------------------------------------------------------- screens
def last_frame(txt, marker, n):
    idx = [m.start() for m in re.finditer(re.escape(marker), txt)]
    for i in reversed(idx):
        lines = txt[i:].split("\n")
        if len(lines) > n:
            return lines[:n]
    return None

def parse_central(txt):
    L = last_frame(txt, " CENTRAL CONTROL ROOM", 23)
    if not L:
        return None
    f = {"title": L[0], "rail": "DOWN" if "railway DOWN" in L[0] else "up",
         "I": {}, "X": {}, "msg": L[16].strip(), "target": "", "draft": None}
    tt = L[17].split()
    if len(tt) > 1 and tt[0] == "TARGET":
        f["target"] = tt[1]
    # The UPDATED row: the four green times u will send, and the phase
    # that - and + change, marked >like this<.
    dr = re.findall(r"\b([ABCD]) ([> ])\s*(\d+)", L[19]) if L[19].strip().startswith("UPDATED") else []
    if len(dr) == 4:
        f["draft"] = ([int(v) for _, _, v in dr], "".join(p for p, m, _ in dr if m == ">"))
    # The SENSOR row ends with the random cars switch.
    rc = re.search(r"random cars (ON|OFF)", L[21])
    f["random"] = rc.group(1) if rc else None
    # The RAILWAY row ends with whether the control room has the trains stopped.
    ln = re.search(r"trains (STOPPED|running)", L[22])
    f["line"] = ln.group(1) if ln else None
    for k in range(6):
        tok = L[3 + k].split()
        if not tok:
            continue
        sel = tok[0].startswith(">")
        if len(tok) > 1 and tok[1] == "----":
            f["I"][k + 1] = {"sel": sel, "waiting": True}
            continue
        # An UPDATED intersection shows the green times it runs, A/B/C/D.
        gr = re.fullmatch(r"(\d+)/(\d+)/(\d+)/(\d+)", tok[18])
        f["I"][k + 1] = {"sel": sel, "waiting": False, "link": tok[1], "ph": tok[2], "st": tok[3],
                         "lamps": tok[4:12], "ped": tok[12:16], "xs": tok[17],
                         "pat": "UPDATED" if gr else tok[18],
                         "greens": [int(v) for v in gr.groups()] if gr else None,
                         "hold": "RAIL" in tok[19:]}
    for k in range(3):
        tok = L[12 + k].split()
        if len(tok) < 2 or tok[1] == "no":
            f["X"][k + 1] = {"online": False}
            continue
        f["X"][k + 1] = {"online": True, "sel": tok[0].startswith(">"), "state": tok[1],
                         "gates": tok[2], "tracks": tok[3] + tok[4], "signal": tok[5],
                         "trains": int(tok[6])}
    return f

def central():
    return parse_central(H("frame c"))

def railway():
    L = last_frame(H("frame r"), " RAILWAY CONTROLLER", 22)
    if not L:
        return None
    tt = re.search(r"trains (\S+)", L[0])
    f = {"title": L[0], "timetable": tt.group(1) if tt else None, "next": L[7].strip(),
         "X": {}, "events": []}
    for k in range(3):
        tok = L[3 + k].split()
        f["X"][k + 1] = {"sel": tok[0].startswith(">"), "state": tok[1], "gates": tok[2],
                         "tracks": tok[3] + tok[4], "signal": tok[5], "fault": tok[6],
                         "trains": int(tok[7])}
    for k in range(6):
        s = L[10 + k].strip()
        if s and s != "nothing yet":
            f["events"].append(s)
    return f

VM2LINE = re.compile(r"^\[(I\d)\]\s+([\d.]+)s\s+ph (\w)\s+(\w+)")

def vm2_lines(n, count=60):
    out = []
    for l in H(f"vm2 {n} {count}").split("\n"):
        m = VM2LINE.match(l)
        out.append((l, m))
    return out

# ----------------------------------------------------------------- logs
LOGLINE = re.compile(r"^\[\s*([\d.]+)\] (.*)$")
LAMP = re.compile(r"^(I\d) ph=(\w) st=(\w+) NS=(\d) EW=(\d) ped=(\d{4}) hold=(\d) x=(\w+)"
                  r"(?: mv=(\d{8}))?")

# The eight vehicle movements in the order the controller logs them.
MV_ORDER = ["NS", "SN", "NW", "SE", "EW", "WE", "WS", "EN"]
# Every movement is named "from arm -> to arm", so the second letter is
# the arm it ends in. A movement that ends in the arm the tracks cross
# drives towards the gates and a train must hold it red.
RAIL_ARM = {1: "E", 2: "W", 3: "E", 4: "W", 5: "E", 6: "W"}
# The movements of each phase, same grouping as rts_safety.c.
PHASE_MV = {"A": ["NS", "SN"], "B": ["NW", "SE"],
            "C": ["EW", "WE"], "D": ["WS", "EN"]}


def blocked_mvs(i):
    """The movements a train forbids at intersection i."""
    return [m for m in MV_ORDER if m[1] == RAIL_ARM[i]]


def exit_mv(i):
    """The through movement that empties the road in front of the gates."""
    return "EW" if RAIL_ARM[i] == "E" else "WE"


def lit(ev, mv):
    """Is that movement showing anything but red in this lamp event?"""
    return bool(ev["mv"]) and ev["mv"].get(mv, 0) != 0

class Log:
    def __init__(self, lines):
        self.segs = []
        for l in lines:
            m = LOGLINE.match(l)
            if not m:
                continue
            t, txt = float(m.group(1)), m.group(2)
            if txt.startswith("=== ") and " started" in txt:
                self.segs.append([])
            if self.segs:
                self.segs[-1].append((t, txt))

    def seg(self, k=-1):
        return self.segs[k] if self.segs else []

    def find(self, pat, k=-1, after=-1.0):
        return [(t, s) for t, s in self.seg(k) if t > after and re.search(pat, s)]

    def lamps(self, k=-1):
        ev = []
        for t, s in self.seg(k):
            m = LAMP.match(s)
            if m:
                mv = {}
                if m.group(9):
                    mv = {name: int(c) for name, c in zip(MV_ORDER, m.group(9))}
                ev.append({"t": t, "ph": m.group(2), "st": m.group(3), "NS": int(m.group(4)),
                           "EW": int(m.group(5)), "ped": m.group(6), "hold": int(m.group(7)),
                           "x": m.group(8), "mv": mv})
        return ev

def logs(save_as=None):
    txt = H("logs", timeout=180)
    if save_as:
        os.makedirs(OUT, exist_ok=True)
        with open(os.path.join(OUT, save_as), "w", encoding="utf-8") as fp:
            fp.write(txt)
    out, cur = {}, None
    for line in txt.split("\n"):
        if line.startswith("#### "):
            p = line[5:]
            if "central" in p:
                name = "central"
            elif "railway" in p:
                name = "railway"
            else:
                name = re.search(r"intersection_(I\d)", p).group(1)
            out[name] = []
            cur = out[name]
            continue
        if cur is not None and line.strip():
            cur.append(line)
    return {k: Log(v) for k, v in out.items()}

def intervals(ev):
    """Each lamp state from its first commit to the next different state.
    Times stay in the process's own seconds; the last, still open, state
    gets t1 = None."""
    out, cur = [], None
    for e in ev:
        k = (e["ph"], e["st"])
        if cur is None or k != (cur["ph"], cur["st"]):
            if cur is not None:
                cur["t1"] = e["t"]
                out.append(cur)
            cur = {"t0": e["t"], "t1": None, "ph": e["ph"], "st": e["st"],
                   "hold": e["hold"], "x": e["x"], "ped": e["ped"],
                   "mv": e.get("mv", {})}
    if cur is not None:
        out.append(cur)
    return out

def dur(iv):
    return None if iv["t1"] is None else (iv["t1"] - iv["t0"]) * SPEED

LEGAL = {("STARTUP", "GREEN"), ("GREEN", "AMBER"), ("AMBER", "ALLRED"),
         ("ALLRED", "GREEN"), ("ALLRED", "RAILCLR"), ("RAILCLR", "AMBER")}

def safety_report(ivs):
    """Amber 4 s, all-red 2 s, legal order, same phase through a clearance."""
    bad = []
    for a, b in zip(ivs, ivs[1:]):
        if (a["st"], b["st"]) not in LEGAL:
            bad.append(f"{a['ph']}{a['st']}->{b['ph']}{b['st']}@{a['t1']:.2f}")
        elif a["st"] in ("GREEN", "AMBER") and b["st"] in ("AMBER", "ALLRED") and a["ph"] != b["ph"]:
            bad.append(f"phase changed inside clearance @{a['t1']:.2f}")
    amb = [dur(i) for i in ivs if i["st"] == "AMBER" and dur(i) is not None]
    red = [dur(i) for i in ivs if i["st"] == "ALLRED" and dur(i) is not None]
    short = [round(d, 2) for d in amb if d < 3.95] + [round(d, 2) for d in red if d < 1.95]
    return bad, amb, red, short

def rng(v):
    return f"{min(v):.2f}..{max(v):.2f} (n={len(v)})" if v else "n=0"

ALL = range(1, 7)

def parse_snap(txt):
    head, _, tail = txt.partition("#### VM2")
    c = parse_central(head)
    last = {}
    for l in tail.split("\n"):
        m = VM2LINE.match(l)
        if m:
            last.setdefault(m.group(1), []).append((m.group(3), m.group(4)))
    return c, last

def xing_lines(lg, x, k=-1):
    r = []
    for t, s in lg.seg(k):
        m = re.match(rf"X{x} -> (\w+) gates=(\d) busy=(\d\d)", s)
        if m:
            r.append((t, m.group(1), int(m.group(2)), m.group(3)))
    return r

def greens_by_phase(ivs, t_from, t_to):
    out = {p: [] for p in "ABCD"}
    for iv in ivs:
        if iv["st"] == "GREEN" and iv["hold"] == 0 and t_from <= iv["t0"] and iv["t1"] is not None and iv["t1"] <= t_to:
            out[iv["ph"]].append(round(dur(iv), 2))
    return out

def cycle_starts(ivs, t_from, t_to):
    return [iv["t0"] for iv in ivs if iv["ph"] == "A" and iv["st"] == "GREEN" and t_from <= iv["t0"] <= t_to]
