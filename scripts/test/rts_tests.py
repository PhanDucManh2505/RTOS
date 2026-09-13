#!/usr/bin/env python3
"""rts_tests.py - the test groups. Run:  python rts_tests.py [g1 g2 ...]"""
import sys
import rts_validate as V
from rts_validate import (H, key, real, check, note, start, wait_for, central, railway, logs,
                          intervals, dur, safety_report, rng, parse_snap, xing_lines,
                          greens_by_phase, cycle_starts, vm2_lines, ALL, SPEED,
                          MV_ORDER, PHASE_MV, RAIL_ARM, blocked_mvs, exit_mv, lit)
import time, re, json, os, statistics as stats


def near(v, want, tol):
    return v is not None and abs(v - want) <= tol


# ---------------------------------------------------------------- g1
def g1():
    """Start up, nodes, safe sequence, live monitoring."""
    start()
    c = wait_for(lambda: (lambda f: f if f and all(not f["I"].get(i, {"waiting": True})["waiting"]
                                                  for i in ALL) else None)(central()), 12)
    check("T01a", "Central thấy đủ 6 nút giao (LINK up)",
          c and all(c["I"][i].get("link") == "up" for i in ALL), c and {i: c["I"][i].get("link") for i in ALL})
    check("T01b", "Central có dữ liệu cả 3 giao cắt", c and all(c["X"][k]["online"] for k in (1, 2, 3)),
          c and {k: c["X"][k].get("state") for k in (1, 2, 3)})
    check("T01c", "Thanh tiêu đề central: railway up", c and c["rail"] == "up", c and c["title"].strip())
    ps = H("ps")
    try:
        v1 = ps.split("vm1:")[1].split("vm2:")[0]
        v2 = ps.split("vm2:")[1].split("vm3:")[0]
        v3 = ps.split("vm3:")[1]
        ok = "central" in v1 and all(f"intersection_i{n}" in v2 for n in ALL) and "railway" in v3
    except IndexError:
        ok = False
    check("T02", "Mỗi tiến trình chạy đúng máy (central@vm1, 6 nút giao@vm2, railway@vm3)", ok,
          " / ".join(x.strip() for x in ps.strip().split("\n")))
    # Off peak the lights only move when asked: a car for B, C and D at
    # all six, so every intersection runs amber and all-red a few times.
    key("c", "0"); key("c", "@#$")
    time.sleep(real(180))
    L = logs("g1_logs.txt")
    for i in ALL:
        lg = L[f"I{i}"]
        ev = lg.lamps()
        first = ev[0] if ev else {}
        rel = lg.find(r"crossing clear, normal cycle resumed")
        check("T01d", f"I{i} khởi động ở trạng thái an toàn, chỉ nhả đường ray khi railway báo CLEAR",
              first.get("hold") == 1 and first.get("x") == "FAULT" and rel,
              f"lệnh đèn đầu: hold={first.get('hold')} x={first.get('x')}; nhả lúc {rel[0][0] if rel else '-'} s")
        ivs = intervals(ev)
        bad, amb, red, short = safety_report(ivs)
        check("T03a", f"I{i} vàng đủ 4 s, đỏ toàn phần đủ 2 s", not short and amb and red,
              f"vàng {rng(amb)}, đỏ {rng(red)}, ngắn hơn chuẩn: {short}")
        check("T03b", f"I{i} đúng trình tự xanh→vàng→đỏ→xanh, không đổi pha giữa chừng", not bad, bad[:4])
        conflict = [e["t"] for e in ev if e["NS"] and e["EW"]]
        viol = lg.find("SAFETY VIOLATION")
        check("T03c", f"I{i} không bao giờ cho hai hướng xung đột cùng đi",
              not conflict and not viol, f"NS&EW cùng lúc: {conflict[:3]}, SAFETY VIOLATION: {len(viol)}")
        order = [iv["ph"] for iv in ivs if iv["st"] == "GREEN"]
        rep = [f"{a}{b}" for a, b in zip(order, order[1:]) if a == b]
        check("T03d", f"I{i} không lặp một pha xanh hai lần liền",
              not rep, f"thứ tự: {''.join(order[-20:])}")
    good = tot = 0
    for _ in range(6):
        c, last = parse_snap(H("snap"))
        for i in ALL:
            seen = last.get(f"I{i}", [])
            row = c["I"].get(i, {}) if c else {}
            if not seen or row.get("waiting", True):
                continue
            tot += 1
            good += (row["ph"], row["st"]) in seen[-2:]
        time.sleep(0.7)
    check("T08", "Central hiển thị đúng pha/trạng thái đang chạy của 6 nút giao (trễ tối đa 1 bước)",
          tot and good == tot, f"{good}/{tot} lần khớp")


# ---------------------------------------------------------------- g2
def g2():
    """Railway pre-emption, gate timing, trains running X1-X2-X3 and back, two meeting."""
    start()
    key("r", "1"); key("r", "a")
    t0 = time.time()                  # train 1 NS: X1 now, X2 after 60 s, X3 after 150 s
    time.sleep(real(34))
    r = wait_for(lambda: (lambda f: f if f and f["X"][1]["state"] == "CLOSED"
                          and f["X"][1]["gates"] == "DOWN" else None)(railway()), real(6) + 2)
    c = central()
    check("T04a", "Railway: X1 CLOSED, cổng DOWN, ray A có tàu",
          r and r["X"][1]["state"] == "CLOSED" and r["X"][1]["gates"] == "DOWN" and r["X"][1]["tracks"] == "A-",
          r and r["X"][1])
    check("T04b", "Central: RAIL HOLD ở I1, I2, không ở I3–I6",
          c and c["I"][1]["hold"] and c["I"][2]["hold"] and not any(c["I"][i]["hold"] for i in (3, 4, 5, 6)),
          c and {i: c["I"][i]["hold"] for i in ALL})
    check("T04c", "Central: dòng X1 = CLOSED, TRACKS A", c and c["X"][1]["state"] == "CLOSED"
          and c["X"][1]["tracks"] == "A-", c and c["X"][1])
    time.sleep(real(19) + 1.5)
    c = central()
    check("T04d", "Tàu qua: X1 CLEAR và I1/I2 bỏ RAIL HOLD",
          c and c["X"][1]["state"] == "CLEAR" and not c["I"][1]["hold"] and not c["I"][2]["hold"],
          c and (c["X"][1].get("state"), c["I"][1]["hold"], c["I"][2]["hold"]))
    # Let train 1 run the whole line and leave X3 (150 + 53 s). Then send
    # one each way so that they meet at X2: SN from X3 reaches X2 after
    # 90 s, NS from X1 after 60 s, so NS leaving 30 s later puts both on
    # X2 at once, one on each track.
    time.sleep(max(0.0, t0 + real(150 + 53 + 5) - time.time()))
    key("r", "b")
    t1 = time.time()                  # train 2 SN: X3 now, X2 after 90 s, X1 after 150 s
    time.sleep(max(0.0, t1 + real(30) - time.time()))
    key("r", "a")                     # train 3 NS: X1 at 30 s, X2 at 90 s, X3 at 180 s
    time.sleep(max(0.0, t1 + real(30 + 150 + 53) + 2 - time.time()))
    L = logs("g2_logs.txt")
    xs = xing_lines(L["railway"], 1)
    warn = [x for x in xs if x[1] == "WARNING"]
    for n in (1, 2):
        lg = L[f"I{n}"]
        w = [t for t, _ in lg.find(r"crossing 1 -> WARNING")]
        cl = [t for t, _ in lg.find(r"crossing 1 -> CLEAR")]
        if not w or not [t for t in cl if t > w[0]]:
            check("T04e", f"I{n} nhận được WARNING/CLEAR của X1", False, f"WARNING {w} CLEAR {cl}")
            continue
        tw = w[0]
        tc = [t for t in cl if t > tw][0]
        pre = [t for t, _ in lg.find(r"pre-emption requested") if tw <= t < tc]
        check("T04e", f"I{n} xin ưu tiên đường sắt đúng 1 lần cho 1 chuyến tàu", len(pre) == 1,
              f"{len(pre)} lần lúc {[round(t, 2) for t in pre]}")
        ivs = intervals(lg.lamps())
        rc = [iv for iv in ivs if iv["st"] == "RAILCLR" and iv["t0"] >= tw]
        first_rc = [iv for iv in rc if iv["t0"] < tc]
        check("T04f", f"I{n} xả đường ray đúng 1 lần, không bao giờ ngắn hơn 6 s (SENSOR)",
              len(first_rc) == 1 and 6.0 - 0.02 <= (dur(first_rc[0]) or 0) <= 6.0 + 1.2,
              [round(dur(iv) or -1, 2) for iv in first_rc])
        nxt = [t for t in w if t > tc]
        extra = [iv for iv in rc if tc <= iv["t0"] < (nxt[0] if nxt else 1e9)]
        check("T04g", f"I{n} không chạy thêm RAILCLR khi X1 đã CLEAR", not extra,
              f"RAILCLR thừa lúc {[round(iv['t0'], 2) for iv in extra]} s, X1 CLEAR lúc {tc:.2f} s")
        evs = lg.lamps()
        blk, ex = blocked_mvs(n), exit_mv(n)
        win = [e for e in evs if tw <= e["t"] <= tc]
        # A movement into the rail arm may never go green once the train
        # is announced. It may still show amber, but only to finish a
        # green it was already running when the train was announced, so
        # the amber must follow a lamp event where it was green.
        wrong = []
        for k, e in enumerate(win):
            prev = win[k - 1] if k else None
            for m in blk:
                v = e["mv"].get(m, 0) if e["mv"] else 0
                if v == 2 or (v == 1 and not (prev and prev["mv"].get(m) == 2)):
                    wrong.append(e)
                    break
        check("T04h", f"I{n} có tàu: luồng đi VÀO nhánh ray ({'+'.join(blk)}) không bao giờ được xanh",
              win and all(e["mv"] for e in win) and not wrong,
              f"{len(win)} lệnh đèn trong lúc có tàu, vi phạm: "
              + str([f"{e['ph']}{e['st']}@{e['t']:.2f}" for e in wrong[:3]]))
        cut = [e for e in win if e["st"] == "AMBER" and any(lit(e, m) for m in blk)]
        cutd = [dur(iv) for iv in intervals(evs)
                if iv["st"] == "AMBER" and any(iv["mv"].get(m) == 1 for m in blk)
                and tw <= iv["t0"] <= tc and dur(iv) is not None]
        check("T04r", f"I{n} luồng đang xanh lúc tàu đến vẫn được đủ 4 s vàng rồi mới đỏ",
              all(d >= 3.95 for d in cutd),
              f"{len(cut)} lệnh vàng có luồng vào ray, dài {[round(d, 2) for d in cutd]} s")
        wrong2 = [e for e in evs if (e["x"] in ("CLOSED", "FAULT") or e["hold"] == 1)
                  and any(lit(e, m) for m in blk)]
        check("T04l", f"I{n} KHÔNG BAO GIỜ mở luồng vào khu vực ray khi giao cắt chưa thông",
              not wrong2, [f"{e['ph']}{e['st']}@{e['t']:.2f} x={e['x']} hold={e['hold']}" for e in wrong2[:3]])
        clr = [e for e in win if e["st"] == "RAILCLR"]
        only = [e for e in clr if e["mv"] and e["mv"][ex] == 2
                and all(v == 0 for k, v in e["mv"].items() if k != ex)]
        check("T04m", f"I{n} xanh xả đường ray chỉ mở đúng chiều thoát ra ({ex})",
              clr and len(only) == len(clr),
              f"{len(clr)} lệnh RAILCLR, đúng {len(only)}: "
              + str([e["mv"] for e in clr[:2]]))
        hg = [e for e in win if e["st"] == "GREEN" and e["hold"] == 1 and e["mv"]]
        badset = [e for e in hg
                  if {m for m in MV_ORDER if e["mv"][m] == 2}
                  != {m for m in PHASE_MV[e["ph"]] if m not in blk}]
        check("T04n", f"I{n} lúc giữ vẫn chạy đủ các luồng KHÔNG đi vào ray",
              hg and not badset,
              "pha xanh khi giữ: " + "".join(dict.fromkeys(e["ph"] for e in hg))
              + ", sai: " + str([f"{e['ph']}@{e['t']:.2f} {e['mv']}" for e in badset[:2]]))
        bad, amb, red, short = safety_report([iv for iv in ivs if iv["t0"] >= tw - 1])
        check("T04i", f"I{n} cắt pha vì tàu nhưng vẫn đủ vàng 4 s + đỏ 2 s", not short and not bad,
              f"vàng {rng(amb)}, đỏ {rng(red)} {bad[:3]}")
    # The same train goes on to the next two pairs of intersections.
    got = {f"I{n}": len(L[f"I{n}"].find(r"pre-emption requested, crossing (WARNING|CLOSED)"))
           for n in (3, 4, 5, 6)}
    stall = {f"I{n}": len(L[f"I{n}"].find(r"crossing unreadable")) for n in (3, 4, 5, 6)}
    check("T04j", "Tàu chạy tiếp: I3/I4 nhận tàu khi tới X2, I5/I6 khi tới X3", all(got.values()), got)
    if any(stall.values()):
        note("T04j+", "Máy ảo đứng hình > 3 s thiết kế nên watchdog báo FAULT (không phải do tàu)",
             f"số lần 'crossing unreadable': {stall}")
    rl = L["railway"]
    tol = 2 * 0.1 * SPEED + 0.1       # the train thread looks every 100 ms of real time
    for tid, no, dirn, order, gaps in (("T20a", 1, "NS", (1, 2, 3), (60, 90)),
                                       ("T20b", 2, "SN", (3, 2, 1), (90, 60))):
        seen = [(t, int(re.search(r"at X(\d)", s).group(1)))
                for t, s in rl.find(rf"train {no} {dirn} detected at X\d")]
        order_seen = tuple(x for _, x in seen)
        d = [round((b[0] - a[0]) * SPEED, 2) for a, b in zip(seen, seen[1:])]
        check(tid, f"Tàu {no} {dirn}: " + " → ".join(f"X{x}" for x in order)
              + f", cách nhau {gaps[0]} s rồi {gaps[1]} s",
              order_seen == order and len(d) == 2 and all(near(v, w, tol) for v, w in zip(d, gaps)),
              f"thứ tự {order_seen}, khoảng cách {d} s")
    closed = [x for x in xs if x[1] == "CLOSED" and x[2] == 2]
    clear = [x for x in xs if x[1] == "CLEAR"]
    if warn and closed and clear:
        d1 = (closed[0][0] - warn[0][0]) * SPEED
        d2 = (clear[0][0] - warn[0][0]) * SPEED
        tick = 0.1 * SPEED      # the crossing threads look every 100 ms of real time
        check("T04k", "Railway: báo trước ≥ 30 s + hạ cổng 4 s; tàu chiếm 15 s + nâng cổng 4 s",
              34 - 0.05 <= d1 <= 34 + 2 * tick + 0.1 and 53 - 0.05 <= d2 <= 53 + 4 * tick + 0.1,
              f"WARNING→CLOSED {d1:.2f} s (chuẩn 34, +≤{2 * tick:.1f} do chu kỳ quét), →CLEAR {d2:.2f} s (chuẩn 53, +≤{4 * tick:.1f})")
    # Trains 2 and 3 meet at X2, one on each track.
    xs2 = xing_lines(L["railway"], 2)
    warn2 = [x for x in xs2 if x[1] == "WARNING"]
    seq = [x for x in xs2 if len(warn2) > 1 and x[0] >= warn2[1][0]]
    busy = [x[3] for x in seq]
    up_early = [x for x in seq if x[3] != "00" and x[2] != 2]
    check("T05", "Hai tàu ngược chiều gặp nhau ở X2: cổng chỉ nâng khi CẢ HAI ray trống",
          "03" in busy and not up_early and seq and seq[-1][1] == "CLEAR", f"busy theo thời gian: {busy}")
    rr = railway()
    check("T05b", "Railway: X1, X2, X3 mỗi giao cắt đếm đủ 3 chuyến tàu",
          rr and all(rr["X"][k]["trains"] == 3 for k in (1, 2, 3)),
          rr and {k: rr["X"][k]["trains"] for k in (1, 2, 3)})

    # A long hold: the crossing is stuck in FAULT, so I1 keeps the mask
    # on for several cycles. FIXED, so the cycle must stay exactly 90 s.
    key("c", "1"); key("c", "f")
    time.sleep(1.2)
    key("r", "1"); key("r", "f")
    time.sleep(real(2 * 90 + 20) + 2)
    L = logs("g2b_logs.txt")
    lg = L["I1"]
    blk, ex = blocked_mvs(1), exit_mv(1)
    tf = lg.find(r"crossing 1 -> FAULT")
    evs = [e for e in lg.lamps() if tf and e["t"] >= tf[-1][0] and e["mv"]]
    wrong = [e for e in evs if any(lit(e, m) for m in blk)]
    greens = [e for e in evs if e["st"] == "GREEN"]
    phases = "".join(dict.fromkeys(e["ph"] for e in greens))
    check("T04o", "Giữ lâu (X1 FAULT): I1 vẫn chạy đủ A→B→C→D, chỉ khoá luồng vào ray",
          evs and not wrong and all(p in phases for p in "ABCD"),
          f"pha đã chạy: {phases}, vi phạm: "
          + str([f"{e['ph']}{e['st']}@{e['t']:.2f}" for e in wrong[:3]]))
    ivs = intervals(lg.lamps())
    cs = cycle_starts(ivs, tf[-1][0] if tf else 0, 1e9)
    lens = [round((b - a) * SPEED, 2) for a, b in zip(cs, cs[1:])]
    check("T04p", "Giữ lâu: chu kỳ vẫn đúng 90 s",
          lens and all(near(v, 90, 0.35) for v in lens), lens)
    exg = [e for e in evs if e["st"] == "GREEN" and e["ph"] == "C"]
    check("T04q", f"Giữ lâu: đường qua ray vẫn chạy chiều thoát ({ex}), chiều vào ray đỏ",
          exg and all(e["mv"][ex] == 2 for e in exg),
          str([e["mv"] for e in exg[:2]]))
    key("r", "c")
    time.sleep(real(6) + 1.0)


# ---------------------------------------------------------------- g3
def phase_after(ivs, t, n):
    """The first n greens that start after t, as [(phase, interval)]."""
    return [(iv["ph"], iv) for iv in ivs if iv["st"] == "GREEN" and iv["t0"] > t][:n]


def g3():
    """Off peak requests: any phase straight away, the green holds with nothing
    asking, pedestrians first and 18 s, cars in arrival order; a train still wins."""
    start()
    key("c", "r")
    time.sleep(1.2)
    c = central()
    check("T06r", "Central r: tắt xe ngẫu nhiên, hàng SENSOR hiện OFF", c and c["random"] == "OFF",
          c and c["random"])
    # Let I1 serve the random cars already waiting, then put it on B.
    key("c", "1")
    time.sleep(real(40))
    key("c", "@")
    time.sleep(real(12))
    # From B, D is not a neighbour: it must still come straight after B.
    key("c", "$")
    time.sleep(real(40))
    # Pedestrian E, a car for B, pedestrian N, in that order.
    key("c", "P@p")
    time.sleep(real(75))
    L = logs("g3a_logs.txt")
    lg = L["I1"]
    ivs = intervals(lg.lamps())
    td = lg.find(r"car waiting for phase D \(operator\)")
    tp = lg.find(r"pedestrian button 2 pressed")
    if not td or not tp:
        check("T06", "I1 nhận được yêu cầu xe D và nút đi bộ E", False, f"xe D {td[:1]}, nút E {tp[:1]}")
        return
    first = phase_after(ivs, td[-1][0], 1)
    check("T06e", "Có xe chờ pha D khi đang B: đổi thẳng sang D (không phải pha kề)",
          first and first[0][0] == "D", [p for p, _ in phase_after(ivs, td[-1][0], 3)])
    dg = first[0][1] if first and first[0][0] == "D" else None
    held = dg is not None and (dg["t1"] is None or dg["t1"] >= tp[-1][0] - 0.01)
    check("T06f", "Không còn yêu cầu nào: D đứng yên vô thời hạn, không có chu kỳ 8 s",
          held and (tp[-1][0] - dg["t0"]) * SPEED >= 30,
          dg and f"D xanh từ {dg['t0']:.2f} s tới {dg['t1']}, nút E lúc {tp[-1][0]:.2f} s")
    seq = phase_after(ivs, tp[-1][0], 3)
    check("T06a", "Bộ hành trước xe: A (đi bộ E) → C (đi bộ N) → B (xe bấm trước người N)",
          [p for p, _ in seq] == ["A", "C", "B"], [p for p, _ in seq])
    for ph, arms, want in (("A", "E W", "0011"), ("C", "N S", "1100")):
        g = [iv for p, iv in seq if p == ph]
        if not g:
            continue
        g = g[0]
        check("T06b", f"Pha {ph} xanh: đi bộ {arms} WALK cùng pha", g["ped"] == want, f"ped={g['ped']} (N S E W)")
        d = dur(g)
        check("T06d", f"Pha {ph} có người đi bộ bấm: giữ ít nhất 18 s rồi mới nhường",
              d is not None and 17.95 <= d <= 20.5, f"xanh {d:.2f} s" if d is not None else "chưa kết thúc")
        amb = [iv for iv in ivs if iv["st"] == "AMBER" and iv["ph"] == ph and iv["t0"] >= g["t0"]][:1]
        check("T06c", f"Pha {ph} hết xanh: đi bộ {arms} nhấp nháy suốt 4 s vàng",
              amb and amb[0]["ped"] == want.replace("1", "2") and near(dur(amb[0]), 4, 0.12),
              amb and f"ped={amb[0]['ped']}, vàng {dur(amb[0])}")
    # A train outranks a pedestrian: p asks for C, and the train is sent
    # while N and S are still inside their 18 s.
    key("c", "p")
    time.sleep(real(9))
    key("r", "a")
    time.sleep(real(12) + 2)
    L = logs("g3b_logs.txt")
    lg = L["I1"]
    ivs = intervals(lg.lamps())
    pr = lg.find(r"pedestrian button 0 pressed")
    warn = [t for t, _ in lg.find(r"crossing 1 -> WARNING") if pr and t > pr[-1][0]]
    cg = [iv for p, iv in phase_after(ivs, pr[-1][0], 1) if p == "C"] if pr else []
    if not warn or not cg:
        check("T07", "Tàu tới lúc người đi bộ N đang đi", False, f"nút {pr[-1:]} WARNING {warn[:1]} xanh C {bool(cg)}")
        return
    tW, g = warn[0], cg[0]
    amb = [iv for iv in ivs if iv["st"] == "AMBER" and iv["t0"] >= g["t0"]][:1]
    note("T07", "Tàu tới lúc người đi bộ N đang đi", f"WARNING sau {(tW - g['t0']) * SPEED:.2f} s xanh C")
    check("T07a", "Tàu thắng người đi bộ: xanh C bị cắt ngay khi có WARNING, không đợi hết 18 s",
          amb and (amb[0]["t0"] - tW) * SPEED <= 1.0 and (amb[0]["t0"] - g["t0"]) * SPEED < 17.5,
          amb and f"vàng sau WARNING {(amb[0]['t0'] - tW) * SPEED:.2f} s, xanh C {(amb[0]['t0'] - g['t0']) * SPEED:.2f} s")
    check("T07b", "Xanh bị cắt vẫn có đủ 4 s vàng, đi bộ N S nhấp nháy",
          amb and near(dur(amb[0]), 4, 0.12) and amb[0]["ped"] == "2200",
          amb and f"vàng {dur(amb[0])}, ped={amb[0]['ped']}")


# ---------------------------------------------------------------- g4
def g4():
    """Targeting, refused command, updateable pattern, FIXED after UPDATED."""
    start()
    key("c", "3"); key("c", "f")
    time.sleep(1.8)
    c = central()
    check("T09a", "Chọn I3 rồi f: chỉ I3 đổi sang FIXED",
          c and c["target"] == "I3" and c["I"][3]["pat"] == "FIXED" and all(c["I"][i]["pat"] == "SENSOR" for i in (1, 2, 4, 5, 6))
          and "I3 accepted the command" in c["msg"], c and ({i: c["I"][i]["pat"] for i in ALL}, c["msg"]))
    key("c", "s")
    time.sleep(1.5)
    # The UPDATED row starts at the programmed 20/13/20/13, phase A.
    key("c", "1"); key("c", "+" * 10 + "]" + "-" * 3 + "]" + "-" * 6 + "]" + "-" * 3)
    time.sleep(0.8)
    c = central()
    check("T12d", "Chỉnh tay dòng UPDATED: 20/13/20/13 thành 30/10/14/10, đang chọn pha D",
          c and c["draft"] == ([30, 10, 14, 10], "D"), c and c["draft"])
    key("c", "u")
    time.sleep(1.8)
    c = central()
    check("T12a", "Mẫu UPDATED 30/10/14/10 được I1 nhận, cột PATTERN hiện đúng số",
          c and c["I"][1]["pat"] == "UPDATED" and c["I"][1]["greens"] == [30, 10, 14, 10],
          c and (c["msg"], c["I"][1].get("greens")))
    key("c", "---"); key("c", "u")
    time.sleep(0.8)
    c = central()
    check("T13a", "Pha D chỉnh xuống 7 s: I1 huỷ lệnh vì an toàn, vẫn chạy 30/10/14/10",
          c and c["msg"] == ">> I1 REFUSED the command: green below the safe minimum"
          and c["I"][1]["greens"] == [30, 10, 14, 10],
          c and (c["msg"], c["I"][1].get("greens")))
    key("c", "0"); key("c", "u")
    time.sleep(0.8)
    c = central()
    check("T13b", "Gửi cả 6 lệnh có pha 7 s: cả 6 huỷ, không ai đổi mẫu",
          c and c["msg"] == ">> all six: 0 accepted, 6 REFUSED (green below the safe minimum), 0 did not answer"
          and c["I"][1]["greens"] == [30, 10, 14, 10] and all(c["I"][i]["pat"] == "SENSOR" for i in (2, 3, 4, 5, 6)),
          c and (c["msg"], {i: c["I"][i]["pat"] for i in ALL}))
    key("c", "1")
    time.sleep(real(88 * 2.4))
    key("c", "f")
    time.sleep(real(90 * 2.6))
    key("c", "s")
    time.sleep(1)
    L = logs("g4_logs.txt")
    lg = L["I1"]
    rej = len(lg.find("REJECTED")), [len(L[f"I{i}"].find("REJECTED")) for i in (2, 3, 4, 5, 6)]
    check("T13c", "Log ghi lại lệnh bị từ chối", rej[0] == 2 and rej[1] == [1] * 5, rej)
    tu = lg.find(r"accepted pattern UPDATED")[0][0]
    tf = [t for t, _ in lg.find(r"accepted pattern FIXED") if t > tu][0]
    ts = [t for t, _ in lg.find(r"accepted pattern SENSOR") if t > tf][0]
    ivs = intervals(lg.lamps())
    cs = cycle_starts(ivs, tu, tf)
    g = greens_by_phase(ivs, cs[0] if cs else tu, tf)
    check("T12b", "UPDATED: pha A/B/C/D xanh đúng 30/10/14/10 s",
          all(g[p] and all(near(v, w, 0.12) for v in g[p]) for p, w in (("A", 30), ("B", 10), ("C", 14), ("D", 10))), g)
    cs = cycle_starts(ivs, tf, ts)
    g = greens_by_phase(ivs, cs[0] if cs else tf, ts)
    check("T12c", "Bấm f sau u: FIXED phải về đúng 20/13/20/13 s",
          all(g[p] and all(near(v, w, 0.12) for v in g[p]) for p, w in (("A", 20), ("B", 13), ("C", 20), ("D", 13))), g)


# ---------------------------------------------------------------- g5
def g5():
    """All six FIXED: exact timing, each intersection on its own."""
    start()
    key("c", "0"); key("c", "f")
    time.sleep(1.8)
    c = central()
    check("T09b", "0 rồi f: cả 6 nhận FIXED",
          c and all(c["I"][i]["pat"] == "FIXED" for i in ALL) and "all six intersections accepted the command" in c["msg"],
          c and ({i: c["I"][i]["pat"] for i in ALL}, c["msg"]))
    cycles = int(os.environ.get("RTS_FIXED_CYCLES", "8"))
    time.sleep(real(90 * cycles) + 1)
    L = logs("g5_logs.txt")
    acc, ivs_all = {}, {}
    for i in ALL:
        lg = L[f"I{i}"]
        acc[i] = lg.find(r"accepted pattern FIXED")[0][0]
        ivs_all[i] = intervals(lg.lamps())
    for i in ALL:
        ivs = ivs_all[i]
        cs = cycle_starts(ivs, acc[i], 1e9)
        lens = [round((b - a) * SPEED, 2) for a, b in zip(cs, cs[1:])]
        allg = greens_by_phase(ivs, cs[0] if cs else acc[i], 1e9)
        bad, amb, red, short = safety_report([iv for iv in ivs if iv["t0"] >= acc[i]])
        def steady(vals, want):
            off = [abs(v - want) for v in vals]
            return vals and sum(1 for d in off if d > 0.12) <= 1 and all(d <= 1.0 for d in off)
        check("T10a", f"I{i} FIXED: B/C/D xanh đúng 13/20/13 s (bỏ qua tối đa 1 lần máy ảo giật)",
              all(steady(allg[p], w) for p, w in (("B", 13), ("C", 20), ("D", 13))),
              {p: allg[p] for p in "BCD"})
        check("T10b", f"I{i} FIXED: pha A luôn đúng 20 s", steady(allg["A"], 20), allg["A"])
        # every state is timed from the moment its lamps changed, so one
        # cycle collects the timer latency of all twelve of its states
        check("T10c", f"I{i} FIXED: chu kỳ đúng 90 s (± 0.35 s, bỏ qua tối đa 1 lần máy ảo giật)",
              lens and sum(1 for v in lens if abs(v - 90) > 0.35) <= 1 and all(abs(v - 90) <= 1.0 for v in lens), lens)
        check("T10d", f"I{i} FIXED: vàng 4 s, đỏ 2 s", not short, f"vàng {rng(amb)}, đỏ {rng(red)}")
    key("c", "s")
    time.sleep(1.8)
    c = central()
    check("T09c", "Sau đó bấm s: cả 6 về SENSOR", c and all(c["I"][i]["pat"] == "SENSOR" for i in ALL),
          c and {i: c["I"][i]["pat"] for i in ALL})


# ---------------------------------------------------------------- g6
def g6():
    """Override: hold, expiry, cancel; pre-emption outranks it."""
    start()
    key("c", "1")
    t0 = time.time()
    key("c", "o")
    c = wait_for(lambda: (lambda f: f if f and f["I"][1].get("pat") == "OVERRIDE" else None)(central()), real(40))
    delay = time.time() - t0
    check("T14a", "Central hiện OVERRIDE ngay sau khi I1 nhận lệnh (≤ 1.5 s)", c and delay <= 1.5,
          f"hiện sau {delay:.1f} s" if c else "không hiện")
    key("c", "#")                     # a car for C, served once the override is over
    time.sleep(max(0.0, real(52) - delay) + 1)
    L = logs("g6a_logs.txt")
    lg = L["I1"]
    t_ov = lg.find(r"override: hold phase A")[0][0]
    t_ex = lg.find(r"override expired")
    ivs = intervals(lg.lamps())
    firstA = [iv for iv in ivs if iv["t0"] >= t_ov and iv["ph"] == "A" and iv["st"] == "GREEN"]
    end = t_ex[0][0] if t_ex else 1e9
    during = [iv for iv in ivs if firstA and firstA[0]["t0"] <= iv["t0"] < end]
    check("T14b", "Override giữ pha A XANH liên tục tới khi hết hạn (không vàng/đỏ xen giữa)",
          firstA and all(iv["ph"] == "A" and iv["st"] == "GREEN" for iv in during),
          " ".join(f"{iv['ph']}{iv['st'][:2]}" for iv in during[:12]))
    check("T14c", "Override tự hết hạn sau 40 s", t_ex and near((t_ex[0][0] - t_ov) * SPEED, 40, 1.2),
          f"{((t_ex[0][0] - t_ov) * SPEED):.2f} s" if t_ex else "không hết hạn")
    after = [iv for iv in ivs if iv["t0"] > end and iv["st"] == "GREEN"]
    check("T14d", "Hết override: các pha B/C/D chạy lại", any(iv["ph"] != "A" for iv in after),
          "".join(iv["ph"] for iv in after[:8]))
    key("c", "o")
    time.sleep(real(10))
    key("c", "#"); key("c", "c")      # after the cancel the car for C gets its green
    time.sleep(real(40))
    L = logs("g6b_logs.txt")
    lg = L["I1"]
    tc = lg.find(r"override cancelled")
    ivs = intervals(lg.lamps())
    after = [iv for iv in ivs if tc and iv["t0"] > tc[-1][0] and iv["st"] == "GREEN"]
    check("T14e", "Bấm c: huỷ override, chu kỳ bình thường chạy lại", tc and any(iv["ph"] != "A" for iv in after),
          "".join(iv["ph"] for iv in after[:8]))
    key("c", "o")
    time.sleep(0.8)
    key("r", "1"); key("r", "a")
    time.sleep(real(34 + 19) + 2)
    L = logs("g6c_logs.txt")
    lg = L["I1"]
    t_ov = lg.find(r"override: hold phase A")[-1][0]
    pre = [t for t, _ in lg.find(r"pre-emption requested") if t > t_ov]
    ivs = intervals(lg.lamps())
    rc = [iv for iv in ivs if iv["t0"] > t_ov and iv["st"] == "RAILCLR"]
    hold = [iv for iv in ivs if iv["t0"] > t_ov and iv["hold"] == 1]
    check("T15", "Tàu thắng override: vẫn xả đường ray và giữ RAIL HOLD", pre and rc and hold,
          f"xin ưu tiên {len(pre)}, RAILCLR {len(rc)}, giữ {len(hold)}")
    key("c", "c")


# ---------------------------------------------------------------- g7
def g7():
    """Gate faults and their acknowledgement, stopping the trains, where a train
    can enter, the timetable."""
    start()
    key("r", "1"); key("r", "f")
    time.sleep(real(20) + 1.2)
    r, c = railway(), central()
    check("T16a", "Railway f: X1 FAULT, đèn tàu RED, cột FAULT = YES",
          r and r["X"][1]["state"] == "FAULT" and r["X"][1]["signal"] == "RED" and r["X"][1]["fault"] == "YES", r and r["X"][1])
    check("T16b", "Central: X1 FAULT, I1/I2 RAIL HOLD, I3–I6 không",
          c and c["X"][1]["state"] == "FAULT" and c["I"][1]["hold"] and c["I"][2]["hold"]
          and not any(c["I"][i]["hold"] for i in (3, 4, 5, 6)), c and (c["X"][1], {i: c["I"][i]["hold"] for i in ALL}))
    ev = " | ".join(r["events"]) if r else ""
    check("T16c", "Railway ghi sự kiện lỗi và báo phòng điều khiển", "GATE FAULT" in ev and "control room told" in ev, ev[-160:])
    check("T16e", "Central tự xác nhận lỗi: railway ghi 'fault acknowledged by the control room'",
          "X1 fault acknowledged by the control room" in ev, ev[-160:])
    key("r", "c")
    time.sleep(real(6) + 1.5)
    r, c = railway(), central()
    check("T16d", "Railway c: X1 về CLEAR, đèn tàu GREEN, I1/I2 bỏ giữ",
          r and r["X"][1]["state"] == "CLEAR" and r["X"][1]["signal"] == "GREEN" and c and not c["I"][1]["hold"]
          and not c["I"][2]["hold"], (r and r["X"][1], c and (c["I"][1]["hold"], c["I"][2]["hold"])))
    # The control room cannot run a train, but it can stop all of them.
    key("r", "a")
    time.sleep(real(5))
    key("c", "e")
    time.sleep(1.2)
    c, r = central(), railway()
    check("T17a", "Central e: mọi giao cắt báo đèn tàu RED, central hiện trains STOPPED, railway hiện LINE STOPPED",
          c and c["line"] == "STOPPED" and all(c["X"][k].get("signal") == "RED" for k in (1, 2, 3))
          and r and "LINE STOPPED" in r["next"],
          (c and (c["line"], {k: c["X"][k].get("signal") for k in (1, 2, 3)}), r and r["next"]))
    key("r", "b")
    time.sleep(1.0)
    r = railway()
    check("T17b", "Đang dừng: railway từ chối tàu mới", r and any("refused, line stopped" in e for e in r["events"]),
          r and r["events"][-3:])
    time.sleep(real(80))
    key("c", "E")
    time.sleep(real(60) + 2)
    lg = logs("g7_stop_logs.txt")["railway"]
    ent = lg.find(r"train \d+ NS enters at X1")
    x2 = lg.find(r"train \d+ NS detected at X2")
    stp = lg.find(r"control room stopped every train")
    rel = lg.find(r"control room released every train")
    if ent and x2 and stp and rel:
        lost = (rel[-1][0] - stp[-1][0]) * SPEED
        gap = (x2[-1][0] - ent[-1][0]) * SPEED
        check("T17c", "Central E: tàu chạy tiếp, tới X2 sau đúng 60 s cộng thời gian bị dừng",
              near(gap, 60 + lost, 1.0), f"X1 → X2 {gap:.2f} s, bị dừng {lost:.2f} s")
    else:
        check("T17c", "Central E: tàu chạy tiếp tới X2", False,
              f"vào {ent[-1:]} X2 {x2[-1:]} dừng {stp[-1:]} chạy lại {rel[-1:]}")
    # Let that train leave the whole line before the next checks.
    time.sleep(real(140))
    # There is no way to put a train on X2: it always enters at an end.
    key("r", "2"); key("r", "a")
    time.sleep(real(34) + 1.2)
    c, r = central(), railway()
    check("T22a", "Railway 2 rồi a: tàu vẫn vào từ X1 (không sinh ở X2), chỉ I1/I2 giữ đường ray",
          r and r["X"][2]["sel"] and r["X"][1]["state"] == "CLOSED" and r["X"][2]["state"] == "CLEAR"
          and c and c["I"][1]["hold"] and c["I"][2]["hold"] and not any(c["I"][i]["hold"] for i in (3, 4, 5, 6)),
          (r and (r["X"][1]["state"], r["X"][2]["state"]), c and {i: c["I"][i]["hold"] for i in ALL}))

    def next_in(line):
        m = re.search(r"one every (\d+) min, next (NS|SN) in (\d+) s", line or "")
        return (int(m.group(1)), int(m.group(3))) if m else None
    seen = {}
    for k in ("r", "o", "n"):
        key("r", k)
        time.sleep(0.8)
        rr = railway()
        seen[k] = rr and (rr["timetable"], rr["next"])
    nr = seen["r"] and next_in(seen["r"][1])
    no = seen["o"] and next_in(seen["o"][1])
    check("T22b", "Railway r / o / n: lịch RUSH (2 phút) / OFF-PEAK (4 phút) / NONE",
          seen["r"] and seen["r"][0] == "RUSH" and nr and nr[0] == 2 and 90 <= nr[1] <= 120
          and seen["o"] and seen["o"][0] == "OFF-PEAK" and no and no[0] == 4 and 210 <= no[1] <= 240
          and seen["n"] and seen["n"][0] == "NONE" and "no timetable" in seen["n"][1],
          seen)

    # Rush hour: two timetabled trains 120 s apart, the second the other way.
    key("r", "r")
    time.sleep(real(2 * 120 + 5) + 1)
    key("r", "n")
    lg = logs("g7_logs.txt")["railway"]
    tt = [(t, re.search(r"train \d+ (NS|SN) enters", s).group(1))
          for t, s in lg.find(r"enters at X\d on track [AB] \(timetable\)")]
    gaps = [round((b[0] - a[0]) * SPEED, 2) for a, b in zip(tt, tt[1:])]
    check("T22c", "Giờ cao điểm: tàu theo lịch cách nhau 120 s, luân phiên NS rồi SN",
          len(tt) == 2 and [d for _, d in tt] == ["NS", "SN"]
          and all(near(v, 120, 2 * 0.1 * SPEED + 0.1) for v in gaps),
          f"{[(round(t, 1), d) for t, d in tt]}, cách nhau {gaps} s")


# ---------------------------------------------------------------- g8
def g8():
    """Node and process failures, logs."""
    start()
    key("c", "0"); key("c", "f")
    time.sleep(1.5)
    key("c", "q")
    time.sleep(real(5) + 2)
    ps = H("ps")
    check("T18a", "Central q: tiến trình central dừng", "central" not in ps.split("vm2:")[0], ps.split("\n")[0])
    L = logs("g8a_logs.txt")
    off = {i: len(L[f"I{i}"].find(r"central controller offline")) for i in ALL}
    check("T18b", "6 nút giao phát hiện mất central, tự chạy tiếp", all(off.values()), off)
    time.sleep(real(60))
    L2 = logs()
    grow = {i: len(L2[f"I{i}"].lamps()) - len(L[f"I{i}"].lamps()) for i in ALL}
    check("T18c", "Không có central, đèn vẫn đổi bình thường", all(v >= 4 for v in grow.values()),
          f"số lần đổi đèn trong 60 s thiết kế: {grow}")
    H("restart central")
    time.sleep(3)
    c = central()
    check("T18d", "Chạy lại central: đủ 6 nút giao, vẫn giữ mẫu FIXED cuối cùng",
          c and all(not c["I"][i]["waiting"] and c["I"][i]["pat"] == "FIXED" for i in ALL),
          c and {i: c["I"][i].get("pat") for i in ALL})
    L = logs()
    back = {i: len(L[f"I{i}"].find(r"central controller back online")) for i in ALL}
    check("T18e", "Nút giao báo lại toàn bộ trạng thái khi central quay lại", all(back.values()), back)
    H("kill central")
    time.sleep(real(5) + 2)
    L = logs()
    off2 = {i: len(L[f"I{i}"].find(r"central controller offline")) for i in ALL}
    check("T18f", "slay central (tắt đột ngột): nút giao vẫn phát hiện", all(v >= 2 for v in off2.values()), off2)
    H("restart central")
    time.sleep(3)
    H("kill railway vm3")
    time.sleep(real(4 + 4 + 2 + 21 + 4 + 2 + 3) + 2)
    c = central()
    check("T19a", "Railway chết: thanh tiêu đề central đỏ 'railway DOWN'", c and c["rail"] == "DOWN", c and c["title"].strip())
    check("T19b", "Railway chết: cả 6 nút giao coi giao cắt là FAULT và giữ đường ray",
          c and all(c["I"][i]["xs"] == "FAULT" and c["I"][i]["hold"] for i in ALL),
          c and {i: (c["I"][i]["xs"], c["I"][i]["hold"]) for i in ALL})
    H("restart railway")
    time.sleep(4)
    c = central()
    check("T19c", "Chạy lại railway: CLEAR, cả 6 nút giao bỏ giữ",
          c and c["rail"] == "up" and all(c["I"][i]["xs"] == "CLEAR" and not c["I"][i]["hold"] for i in ALL),
          c and {i: (c["I"][i]["xs"], c["I"][i]["hold"]) for i in ALL})
    H("kill intersection_i3 vm2")
    time.sleep(real(4) + 1.5)
    c = central()
    check("T20a", "slay I3: central đánh dấu I3 DOWN, I4 vẫn up",
          c and c["I"][3]["link"] == "DOWN" and c["I"][4]["link"] == "up", c and (c["I"][3]["link"], c["I"][4]["link"], c["msg"]))
    L = logs()
    n4 = len(L["I4"].lamps())
    time.sleep(real(15))
    L = logs()
    m4 = len(L["I4"].lamps())
    check("T20b", "I4 và các nút giao khác vẫn chạy khi I3 chết", m4 > n4,
          f"{n4} → {m4}" + (" (không đọc được log qua Qnet)" if m4 == 0 else ""))
    H("restart i3")
    time.sleep(4)
    c = central()
    check("T20c", "Chạy lại I3: central thấy I3 up", c and c["I"][3]["link"] == "up" and not c["I"][3]["waiting"],
          c and c["I"][3])
    L = logs("g8_logs.txt")
    cl = [s for _, s in L["central"].seg(0)]
    check("T21", "Log central có mốc thời gian cho các thay đổi trạng thái",
          any("reported for the first time" in s for s in cl) and any("declared offline" in s for s in [x for _, x in L["central"].seg(-1)] + cl),
          f"{len(cl)} dòng ở lần chạy đầu")


GROUPS = {"g1": g1, "g2": g2, "g3": g3, "g4": g4, "g5": g5, "g6": g6, "g7": g7, "g8": g8}

if __name__ == "__main__":
    names = sys.argv[1:] or list(GROUPS)
    os.makedirs(V.OUT, exist_ok=True)
    t0 = time.time()
    for n in names:
        try:
            GROUPS[n]()
        except Exception as e:
            import traceback
            traceback.print_exc()
            check(n.upper(), f"nhóm {n} bị lỗi khi chạy test", False, repr(e))
    H("stop")
    with open(os.path.join(V.OUT, "results.json"), "w", encoding="utf-8") as fp:
        json.dump(V.RESULTS, fp, ensure_ascii=False, indent=1)
    p = sum(1 for r in V.RESULTS if r["ok"] is True)
    f = sum(1 for r in V.RESULTS if r["ok"] is False)
    print(f"\n==== {p} PASS, {f} FAIL, {time.time() - t0:.0f} s, kết quả: {V.OUT}/results.json")
