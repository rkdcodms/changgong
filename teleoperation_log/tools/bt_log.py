# -*- coding: utf-8 -*-
"""블루투스 끊김 로그 기록·분석 도구 (BTLOG 펌웨어용)

  (이 파일이 있는 tools 폴더에서 실행합니다)
  기록:  python bt_log.py COM5            (리더 보드 USB 하나만 꽂아도 로봇 쪽 기록까지 같이 받는다)
         python bt_log.py COM5 COM7       (리더와 로봇 보드를 둘 다 USB로 꽂았을 때)
         Ctrl+C 로 멈추면 logs/ 폴더에 파일이 남고, 끊김 원인 요약이 나온다.
  분석:  python bt_log.py --analyze logs/bt_20261113_150102.log
         (아두이노 IDE 시리얼 모니터에서 복사해 붙여 넣은 .txt 도 된다)
  포트:  python bt_log.py --list
  올리기: python bt_log.py COM5 --upload --note "주행 시험, LED 깜빡임"
         (기록이 끝나면 로그와 요약을 GitHub 저장소의 bt-logs 브랜치에 올린다)
         python bt_log.py --upload-file logs/bt_20261113_150102.log   (이미 있는 로그 올리기)

필요: 파이썬 3.8 이상, 기록할 때만 pyserial (python -m pip install pyserial),
      올릴 때만 git 과 이 저장소를 clone 한 폴더 + GitHub 쓰기 권한
"""
import argparse
import datetime as dt
import os
import queue
import re
import subprocess
import sys
import tempfile
import threading
import time
from collections import Counter

START_RE = re.compile(r"#([RL]) (\d{1,10}) ([A-Z_]+)")
KV_RE = re.compile(r'(\w+)=("([^"]*)"|\S+)')
PC_TIME_RE = re.compile(r"^(\d{4}-\d{2}-\d{2} )?(\d{2}):(\d{2}):(\d{2})\.(\d{3})")

NUM = r"\d+"
V1 = r"\d+\.\d"
CLS = r"NO_DATA|BAD_DATA|PARTIAL|SLOW_LOOP"
ANY = r".*"
WORD = r"[A-Z0-9_]+"
# 펌웨어가 찍는 줄의 정확한 키와 값 모양. 블루투스로 넘어오며 깨진 줄은 여기서 걸러진다.
SCHEMA = {
    ("R", "S"): dict(good=NUM, bad=NUM, why=r"c\d+/l\d+/p\d+", bytes=NUM, gapmax=NUM, since_good=r"\d+|none",
                     loopmax_us=NUM, slow=NUM, rxpeak=NUM, vin=r"\d+\.\d-\d+\.\d|na", hold=NUM, link=r"[01]",
                     fault=r"[01]", qdrop=NUM, rst=WORD),
    ("R", "GAP"): dict(dur=NUM, cls=CLS, bytes=NUM, bad=NUM, sil=NUM, loopmax_us=NUM, rxpeak=NUM, vin=V1),
    ("R", "GAP_OPEN"): dict(dur=NUM, cls=CLS, bytes=NUM, bad=NUM, sil=NUM, loopmax_us=NUM, rxpeak=NUM, vin=V1),
    ("R", "HOLD"): dict(since_good=NUM, bytes=NUM, bad=NUM, sil=NUM, loopmax_us=NUM, rxpeak=NUM, vin=V1),
    ("R", "BOOT"): dict(rst=WORD, raw=r"0x[0-9A-F]{2}", build=r"\S+"),
    ("R", "READY"): dict(setup_ms=NUM, grip=r"[01]", wheels=r"[01]", rst=WORD),
    ("R", "LATCHED"): dict(reason=ANY, id=NUM),
    ("L", "S"): dict(sent=NUM, rej=NUM, busy=NUM, sendgap=NUM, since_sent=r"\d+|none", loopmax_us=NUM, slow=NUM,
                     robot_lines=NUM, robot_bad=NUM, robot_bytes=NUM, robot_age=r"\d+|none",
                     state=r"LATCHED|HOMING|STREAMING|NOT_READY", rst=WORD, why=ANY),
    ("L", "SENDGAP"): dict(dur=NUM, why=ANY),
    ("L", "ROBOT_SILENT"): dict(since_ms=NUM, bytes_meanwhile=NUM),
    ("L", "ROBOT_BACK"): dict(silent_ms=NUM, bytes_meanwhile=NUM),
    ("L", "BOOT"): dict(rst=WORD, raw=r"0x[0-9A-F]{2}", build=r"\S+"),
    ("L", "LATCHED"): dict(reason=ANY, id=NUM),
}
RST_TEXT = {
    "BROWNOUT_3V3": "3.3V 전원이 순간적으로 떨어짐(브라운아웃)",
    "BROWNOUT_1V2": "보드 내부 1.2V 전원이 떨어짐",
    "POWER_ON": "전원이 완전히 끊겼다가 다시 들어옴(사람이 껐다 켠 것이 아니면 배터리·커넥터·전원선 문제)",
    "RESET_PIN": "리셋 버튼",
    "SOFTWARE": "업로드나 소프트웨어 리셋",
    "WATCHDOG": "워치독",
}


def parse_kv(rest):
    out = {}
    for m in KV_RE.finditer(rest):
        out[m.group(1)] = m.group(3) if m.group(3) is not None else m.group(2)
    return out


def to_int(v, default=0):
    try:
        return int(v)
    except (TypeError, ValueError):
        return default


def pc_seconds(line):
    """'2026-11-13 15:01:02.345 ...' 또는 아두이노 IDE '15:01:02.345 -> ...' 에서 초 단위 시각."""
    m = PC_TIME_RE.match(line)
    if not m:
        return None
    return int(m.group(2)) * 3600 + int(m.group(3)) * 60 + int(m.group(4)) + int(m.group(5)) / 1000.0


def valid(side, typ, kv):
    schema = SCHEMA.get((side, typ))
    if schema is None:
        return False
    for key, pat in schema.items():
        if key == "qdrop" and key not in kv:
            continue
        if key not in kv or not re.fullmatch(pat, kv[key]):
            return False
    return True


def vin_ok(v):
    return 3.0 <= v <= 16.0


def read_log_file(path):
    raw = open(path, "rb").read()
    if raw.startswith(b"\xff\xfe") or raw.startswith(b"\xfe\xff") or raw[:200].count(b"\x00") > 20:
        text = raw.decode("utf-16", errors="replace")
    else:
        text = raw.decode("utf-8-sig", errors="replace")
    return text.splitlines()


# ----------------------------------------------------------------------------- 분석
class Event:
    __slots__ = ("pc", "side", "ms", "typ", "kv", "raw", "outer_ms", "idx")

    def __init__(self, pc, side, ms, typ, kv, raw, outer_ms, idx):
        self.pc, self.side, self.ms, self.typ, self.kv, self.raw, self.outer_ms, self.idx = pc, side, ms, typ, kv, raw, outer_ms, idx


def parse_events(lines):
    events, corrupt_fwd, corrupt_direct, usb_drops = [], 0, 0, []
    last_seen = {}
    for idx, raw in enumerate(lines):
        raw = raw.rstrip("\r\n")
        pc = pc_seconds(raw)
        if "## USB DISCONNECTED" in raw:
            usb_drops.append((pc, idx))
            continue
        starts = list(START_RE.finditer(raw))
        if not starts:
            continue
        outer_ms = None
        first = starts[0]
        forwarded = first.group(1) == "L" and first.group(3) == "FROM_ROBOT"
        if forwarded:
            outer_ms = int(first.group(2))
            inner = [m for m in starts[1:] if m.group(1) == "R"]
            if not inner:
                corrupt_fwd += 1
                continue
            m = inner[-1]                       # 줄이 붙어 들어오면 마지막(완전한) 것만
            if len(inner) > 1:
                corrupt_fwd += 1
        else:
            m = starts[-1]
            if len(starts) > 1:
                corrupt_direct += 1
        side, ms, typ = m.group(1), int(m.group(2)), m.group(3)
        rest = raw[m.end():]
        kv = parse_kv(rest)
        if not valid(side, typ, kv):
            if forwarded:
                corrupt_fwd += 1
            else:
                corrupt_direct += 1
            continue
        key = (side, ms, typ, rest.strip())
        prev = last_seen.get(key)
        if prev is not None and (pc is None or prev is None or abs(pc - prev) < 3.0):
            if not (pc is None and typ in ("BOOT", "READY")):
                continue                        # 로봇 USB와 리더 전달로 같은 줄이 두 번 들어온 것
        last_seen[key] = pc
        events.append(Event(pc, side, ms, typ, kv, raw, outer_ms, idx))
    return events, corrupt_fwd, corrupt_direct, usb_drops


def find_restarts(evs, boot_types):
    """보드 ms가 2초 넘게 뒤로 가고(다음 줄도 확인) 또는 BOOT/READY 가 나오면 재시작. [(event, rst)]"""
    jumps = []
    prev_ms = None
    for i, e in enumerate(evs):
        nxt = evs[i + 1] if i + 1 < len(evs) else None
        if prev_ms is not None and e.ms + 2000 < prev_ms:
            confirmed = e.typ in boot_types or (nxt is not None and nxt.ms + 2000 < prev_ms and nxt.ms >= e.ms)
            if confirmed:
                jumps.append(e)
                prev_ms = e.ms
            continue                            # 확인 안 된 한 줄은 숫자가 깨진 것: 무시
        if prev_ms is not None and e.ms > prev_ms + 120000:
            if nxt is not None and not (e.ms - 2000 <= nxt.ms <= e.ms + 120000):
                continue                        # 숫자가 부풀려진 깨진 줄
        prev_ms = e.ms if prev_ms is None else max(prev_ms, e.ms)
    # BOOT/READY 묶음으로도 센다 (재시작 직후 ms가 2초 이상 안 줄어드는 경우).
    # READY 와 BOOT 는 한 번 켜질 때 붙어서 나온다.
    groups = []
    last_was_boot = False
    for e in evs:
        if e.typ in boot_types:
            if last_was_boot:
                groups[-1].append(e)
            else:
                groups.append([e])
            last_was_boot = True
        else:
            last_was_boot = False
    if evs and evs[0].typ in boot_types and groups:
        groups = groups[1:]                     # 기록을 켜자마자 보드가 켜진 것은 재시작이 아님
    merged = list(jumps)
    for g in groups:
        head = g[0]
        if not any(abs(j.ms - head.ms) < 15000 and abs(j.idx - head.idx) < 60 for j in jumps):
            merged.append(head)
    merged.sort(key=lambda e: e.idx)
    return merged


def rst_after(evs, start_idx):
    for e in evs:
        if e.idx >= start_idx and "rst" in e.kv:
            return e.kv["rst"]
    return "?"


def analyze(lines):
    events, corrupt_fwd, corrupt_direct, usb_drops = parse_events(lines)
    R = [e for e in events if e.side == "R"]
    L = [e for e in events if e.side == "L"]
    rep = []
    p = rep.append
    p("=" * 72)
    p("블루투스 로그 요약  (로봇 줄 %d개, 리더 줄 %d개)" % (len(R), len(L)))
    p("=" * 72)
    if not R and not L:
        p("'#R' / '#L' 기록 줄이 없습니다. BTLOG 펌웨어(_BTLOG 빌드)를 올렸는지, 로그 파일이 맞는지 확인하세요.")
        return "\n".join(rep)

    # ---- 재시작
    r_restarts = find_restarts(R, ("BOOT", "READY"))
    l_restarts = find_restarts(L, ("BOOT",))
    r_restart_rst = [rst_after(R, e.idx) for e in r_restarts]
    l_restart_rst = [rst_after(L, e.idx) for e in l_restarts]
    l_states = [e.kv.get("state") for e in L if e.typ == "S"]
    homing_again = sum(1 for a, b in zip(l_states, l_states[1:]) if a == "STREAMING" and b == "HOMING")
    l_restart_count = max(len(l_restarts), homing_again)
    current_r_rst = rst_after(R, 0) if R else "?"

    # ---- 로봇 S 창
    S = [e for e in R if e.typ == "S"]
    tot_good = sum(to_int(e.kv["good"]) for e in S)
    tot_bad = sum(to_int(e.kv["bad"]) for e in S)
    why = Counter()
    for e in S:
        c, l_, p_ = (int(x) for x in re.match(r"c(\d+)/l(\d+)/p(\d+)", e.kv["why"]).groups())
        why["깨진 글자"] += c
        why["너무 긴 줄"] += l_
        why["형식 오류"] += p_
    rxpeak = max([to_int(e.kv["rxpeak"]) for e in S] or [0])
    loopmax = max([to_int(e.kv["loopmax_us"]) for e in S] or [0])
    vmins, vmaxs = [], []
    for e in S:
        m = re.match(r"(\d+)\.(\d)-(\d+)\.(\d)", e.kv["vin"])
        if m:
            lo = int(m.group(1)) + int(m.group(2)) / 10.0
            hi = int(m.group(3)) + int(m.group(4)) / 10.0
            if vin_ok(lo) and vin_ok(hi):
                vmins.append(lo)
                vmaxs.append(hi)
    readies = sum(1 for e in R if e.typ == "READY")
    startup_rejects = readies + (1 if R and R[0].typ not in ("BOOT", "READY") else 0)
    r_latched = Counter(e.kv["reason"] for e in R if e.typ == "LATCHED")
    holds = [e for e in R if e.typ == "HOLD"]

    # ---- 끊김(닫힌 것 + 안 닫힌 것)
    gaps = []          # dict(dur, cls, bad, bytes, ev, recovered, ended_by)
    open_out = None
    restart_idx = {e.idx for e in r_restarts}
    for e in R:
        if e.idx in restart_idx and open_out is not None:
            open_out["ended_by"] = "로봇 재부팅/전원 재투입"
            gaps.append(open_out)
            open_out = None
        if e.typ == "GAP_OPEN":
            d = to_int(e.kv["dur"])
            if open_out is None or d < open_out["dur"]:
                if open_out is not None:
                    open_out["ended_by"] = "기록이 끊김"
                    gaps.append(open_out)
                open_out = dict(dur=d, cls=e.kv["cls"], bad=to_int(e.kv["bad"]), bytes=to_int(e.kv["bytes"]),
                                ev=e, recovered=False, ended_by=None)
            else:
                open_out.update(dur=d, cls=e.kv["cls"], bad=to_int(e.kv["bad"]), bytes=to_int(e.kv["bytes"]), ev=e)
        elif e.typ == "GAP":
            gaps.append(dict(dur=to_int(e.kv["dur"]), cls=e.kv["cls"], bad=to_int(e.kv["bad"]),
                             bytes=to_int(e.kv["bytes"]), ev=e, recovered=True, ended_by=None))
            open_out = None
    if open_out is not None:
        open_out["ended_by"] = "기록 끝까지 회복 안 됨"
        gaps.append(open_out)

    # 리더만 기록한 경우: 로봇 줄이 안 돌아온 구간(ROBOT_SILENT)을 끊김으로 본다
    sil_eps = []
    cur = None
    for e in events:
        if e.side == "L" and e.typ == "ROBOT_SILENT":
            cur = dict(start=e, bytes0=to_int(e.kv["bytes_meanwhile"]))
        elif e.side == "L" and e.typ == "ROBOT_BACK" and cur is not None:
            cur.update(end=e, silent_ms=to_int(e.kv["silent_ms"]), bytes=to_int(e.kv["bytes_meanwhile"]))
            sil_eps.append(cur)
            cur = None
    if cur is not None:
        cur.update(end=None, silent_ms=None, bytes=cur["bytes0"])
        sil_eps.append(cur)
    for ep in sil_eps:
        after = [e for e in R if e.idx > (ep["end"].idx if ep["end"] else ep["start"].idx)]
        nxt = after[0] if after else None
        if ep["end"] is None:
            ep["how"] = "기록 끝까지 로봇 기록이 안 돌아옴"
        elif nxt is not None and (nxt.idx in restart_idx or nxt.typ in ("READY", "BOOT")):
            ep["how"] = "로봇이 재부팅된 뒤에야 돌아옴"
        elif nxt is not None and nxt.typ == "GAP":
            ep["how"] = "로봇이 GAP 줄을 보냄(저절로 회복)"
        else:
            ep["how"] = "다시 들어옴"
        # 같은 끊김이 로봇 GAP으로도 잡혔으면 중복으로 세지 않는다
        ep["covered"] = nxt is not None and nxt.typ == "GAP"

    # ---- 리더
    LS = [e for e in L if e.typ == "S"]
    l_sent = sum(to_int(e.kv["sent"]) for e in LS)
    l_rej = sum(to_int(e.kv["rej"]) for e in LS)
    l_busy = sum(to_int(e.kv["busy"]) for e in LS)
    robot_bad_lines = sum(to_int(e.kv["robot_bad"]) for e in LS)
    l_state_count = Counter(e.kv["state"] for e in LS)
    sendgaps = [e for e in L if e.typ == "SENDGAP"]
    sg_why = Counter(e.kv["why"] for e in sendgaps)
    l_latched = Counter(e.kv["reason"] for e in L if e.typ == "LATCHED")
    for e in LS:
        if e.kv["state"] == "LATCHED" and not l_latched:
            l_latched[e.kv["why"]] += 1

    # ---- 끊김마다 원인 맞추기
    def leader_ms_near(g):
        return g["ev"].outer_ms

    for g in gaps:
        g["why"] = None
        if g["cls"] not in ("NO_DATA", "PARTIAL"):
            continue
        for s in sendgaps:
            close = False
            if g["ev"].pc is not None and s.pc is not None:
                close = abs(s.pc - g["ev"].pc) <= 2.5
            elif leader_ms_near(g) is not None:
                close = abs(s.ms - leader_ms_near(g)) <= 1500
            if close and to_int(s.kv["dur"]) >= g["dur"] * 0.5:
                g["why"] = "리더가 안 보냄(%s)" % s.kv["why"]
                break
        if g["why"] is None and l_restart_count:
            for lr in l_restarts:
                if g["ev"].pc is not None and lr.pc is not None and 0 <= g["ev"].pc - lr.pc <= 10:
                    g["why"] = "리더 재부팅"
                    break
            if g["why"] is None and usb_drops:
                for (upc, _) in usb_drops:
                    if g["ev"].pc is not None and upc is not None and 0 <= g["ev"].pc - upc <= 10:
                        g["why"] = "리더 재부팅(USB 끊김 기록)"
                        break
    big = [g for g in gaps if g["dur"] >= 300 or not g["recovered"]]
    radio_gaps = [g for g in gaps if g["cls"] in ("NO_DATA", "PARTIAL") and g["why"] is None]
    radio_big = [g for g in radio_gaps if g["dur"] >= 300 or not g["recovered"]]
    leader_explained = [g for g in gaps if g["why"]]
    unrecovered = [g for g in gaps if not g["recovered"]]
    silent_uncovered = [ep for ep in sil_eps if not ep["covered"]]
    corrupt_gaps = [g for g in gaps if g["cls"] == "BAD_DATA" and g["bad"] >= 2]
    slow_gaps = [g for g in gaps if g["cls"] == "SLOW_LOOP"]
    extra_bad = max(0, tot_bad - startup_rejects)
    bad_rate = extra_bad / max(1, tot_good + extra_bad)

    # ---- 보고
    p("")
    p("[로봇(팔로워) 쪽]")
    if S:
        p("  정상 줄 %d, 버린 줄 %d (켜질 때 버리는 %d줄 제외 %.2f%%)  버린 이유: %s" % (
            tot_good, tot_bad, startup_rejects, 100.0 * bad_rate,
            ", ".join("%s %d" % kv for kv in why.items() if kv[1]) or "없음"))
    else:
        p("  로봇 S줄 없음 (로봇이 BTLOG 빌드가 아니거나, 무선이 계속 끊겨 리더로 안 왔거나, 로봇 USB를 기록 안 함)")
    cls_count = Counter(g["cls"] for g in gaps)
    p("  끊김(정상 줄이 150ms 이상 안 옴) %d번  종류: %s" % (len(gaps), ", ".join("%s %d" % kv for kv in cls_count.most_common()) or "없음"))
    for g in sorted(gaps, key=lambda g: -g["dur"])[:6]:
        tail = "" if g["recovered"] else "  ← %s" % g["ended_by"]
        p("    - %6d ms  %-9s bytes=%s bad=%s  %s%s%s" % (
            g["dur"], g["cls"], g["bytes"], g["bad"], g["ev"].raw[:23] if g["ev"].pc is not None else "",
            ("  [%s]" % g["why"]) if g["why"] else "", tail))
    p("  팔 정지(HOLD, 0.5초 이상 끊김) %d번" % len(holds))
    if r_restarts:
        p("  로봇 보드 재시작 %d번: %s" % (len(r_restarts), ", ".join(r_restart_rst)))
    else:
        p("  로봇 보드 재시작 없음 (지금 켜진 이유: %s)" % current_r_rst)
    if S:
        p("  수신 버퍼 최대 %d/256 바이트, loop 최대 %.1f ms (원래 코드도 loop마다 10ms 이상 걸림)" % (rxpeak, loopmax / 1000.0))
    if vmins:
        p("  모터 전원(XL430 입력 전압) 최저 %.1f V, 최고 %.1f V" % (min(vmins), max(vmaxs)))
    if r_latched:
        p("  로봇 고장 래치: %s" % ", ".join("%s(%d초)" % kv for kv in r_latched.items()))

    p("")
    p("[리더 쪽]")
    if LS:
        p("  보낸 줄 %d, 모터 읽기 실패로 못 보낸 줄 %d, 송신 버퍼가 차서 건너뛴 줄 %d" % (l_sent, l_rej, l_busy))
        p("  상태: %s" % ", ".join("%s %d초" % kv for kv in l_state_count.most_common()))
        p("  로봇에서 돌아온 기록 줄 중 깨진 줄 %d" % robot_bad_lines)
    else:
        p("  리더 S줄 없음 (리더가 BTLOG 빌드가 아니거나 리더 USB를 기록 안 함)")
    p("  리더가 스스로 안 보낸 구간(SENDGAP) %d번  이유: %s" % (len(sendgaps), ", ".join("%s %d" % kv for kv in sg_why.most_common()) or "없음"))
    p("  로봇 기록이 안 돌아온 구간(ROBOT_SILENT) %d번" % len(sil_eps))
    for ep in sorted(sil_eps, key=lambda ep: -(ep["silent_ms"] or 10 ** 9))[:4]:
        p("    - %s  그동안 받은 바이트 %d  → %s" % (
            ("%.1f초" % (ep["silent_ms"] / 1000.0)) if ep["silent_ms"] else "끝까지", ep["bytes"], ep["how"]))
    if l_restart_count:
        p("  리더 보드 재시작 %d번: %s" % (l_restart_count, ", ".join(l_restart_rst) or "?"))
    if usb_drops:
        p("  기록 중 USB 연결이 끊긴 횟수 %d (보드 리셋·케이블)" % len(usb_drops))
    if l_latched:
        p("  리더 고장 래치: %s" % ", ".join(l_latched))
    if corrupt_fwd or corrupt_direct:
        p("  모양이 깨져서 버린 기록 줄: 블루투스로 온 것 %d, USB로 온 것 %d" % (corrupt_fwd, corrupt_direct))

    # ---- 판단 (근거 수로 정렬)
    hints = []          # (score, text)
    if r_restarts:
        kinds = Counter(r_restart_rst)
        txt = ", ".join("%s %d번(%s)" % (k, n, RST_TEXT.get(k, "원인 미상")) for k, n in kinds.items())
        # 끊긴 채로 있다가 재시작으로 끝난 경우는 사람이 복구하려고 껐다 켠 것일 가능성이 크다
        recovery = sum(1 for g in gaps if g["ended_by"] == "로봇 재부팅/전원 재투입") + \
            sum(1 for ep in sil_eps if ep["how"] == "로봇이 재부팅된 뒤에야 돌아옴")
        recovery = min(recovery, len(r_restarts))
        if any(k.startswith("BROWNOUT") for k in kinds):
            score = 100
        elif recovery >= len(r_restarts):
            score = 15
        else:
            score = 40
        extra = (" 그중 %d번은 이미 끊긴 뒤에 일어났으니 복구하려고 껐다 켠 것일 수 있습니다." % recovery) if recovery else ""
        hints.append((score, "로봇 보드가 기록 중에 재시작했습니다: %s.%s 재시작하는 동안(셋업 수 초)에는 팔이 안 따라오고, "
                             "블루투스도 다시 붙어서 끊긴 것처럼 보입니다. 사람이 껐다 켠 것이 아니면 배터리 전압·전원 커넥터·전원선을 보세요."
                      % (txt, extra)))
    if r_latched:
        hints.append((90, "로봇이 고장 래치 상태였습니다(%s). 이때는 블루투스가 살아 있어도 팔이 안 따라오고, 전원을 껐다 켜야 풀립니다. "
                          "블루투스 문제가 아닙니다." % ", ".join(r_latched)))
    if l_latched:
        hints.append((90, "리더가 래치되어 보내기를 멈췄습니다(%s). 블루투스 문제가 아니라 리더 쪽 모터·설정 문제입니다." % ", ".join(l_latched)))
    if leader_explained:
        whys = Counter(g["why"] for g in leader_explained)
        hints.append((10 * len(leader_explained), "끊김 %d번은 같은 시각에 리더가 보내지 않았거나 재부팅 중이었습니다(%s). "
                                                  "이건 무선이 아니라 리더 쪽 문제입니다(모터 읽기 실패면 리더 다이나믹셀 배선·ID, 재부팅이면 리더 전원)."
                      % (len(leader_explained), ", ".join("%s %d" % kv for kv in whys.items()))))
    if radio_big or silent_uncovered:
        n = len(radio_big) + len(silent_uncovered)
        unrec = sum(1 for g in radio_big if not g["recovered"]) + sum(1 for ep in silent_uncovered if ep["how"] != "다시 들어옴")
        hints.append((10 * n + 30 * unrec, "리더는 보내고 있는데 로봇 쪽에 아무것도 안 들어온 무선 끊김이 %d번입니다%s. "
                                           "BT-210 파란 LED가 깜빡이면(재연결 중) 모듈 전원(3.3V인지)·거리·가림·간섭을, 계속 켜져 있으면 배선(Tx/Rx)을 보세요. "
                                           "5V에 연결해 썼던 모듈이면 3.3V로 고친 뒤 다시 기록해 비교하고, 그래도 끊기면 모듈 한 쌍을 바꿔 보세요."
                      % (n, (" (그중 %d번은 저절로 회복되지 않음)" % unrec) if unrec else "")))
    corrupt_score = 0
    if corrupt_gaps:
        corrupt_score += 10 * len(corrupt_gaps)
    if extra_bad >= 5 and bad_rate > 0.01:
        corrupt_score += 20
    if robot_bad_lines >= 3:
        corrupt_score += 10
    if corrupt_fwd >= 3:
        corrupt_score += 5
    if corrupt_score:
        hints.append((corrupt_score, "무선은 붙어 있는데 글자가 깨져 들어옵니다(로봇이 버린 줄 %d, 깨진 끊김 %d번, 리더가 받은 깨진 로봇 줄 %d). "
                                     "전기 쪽 문제일 가능성이 큽니다: BT-210 전원을 5V에 연결했다면 3.3V로 옮기고, 커넥터 방향·납땜·보드레이트(57600)를 확인하세요."
                      % (extra_bad, len(corrupt_gaps), robot_bad_lines)))
    if slow_gaps or rxpeak >= 250:
        hints.append((10 * len(slow_gaps) + 10, "로봇 보드가 바빠서 블루투스를 제때 못 읽은 끊김이 있습니다(SLOW_LOOP %d번, 수신 버퍼 최대 %d/256). "
                                                "다이나믹셀 응답 시간초과가 겹치는지 보세요." % (len(slow_gaps), rxpeak)))
    if vmins and min(vmins) < 10.0:
        hints.append((30, "모터 전원이 %.1f V까지 떨어졌습니다. 배터리 충전 상태와 전원선을 확인하세요." % min(vmins)))
    if not hints:
        if big:
            hints.append((1, "끊김은 있었지만 원인을 가를 근거가 부족합니다. 로봇 USB도 같이 기록하거나 끊김이 일어날 때 BT-210 LED를 메모해 주세요."))
        else:
            hints.append((1, "이 기록에서는 끊김 원인이 보이지 않았습니다. 끊김이 실제로 일어나는 동안(주행·방지턱·팔 큰 동작) 다시 기록하세요."))
    p("")
    p("[판단 (근거가 많은 것부터)]")
    for i, (_, h) in enumerate(sorted(hints, key=lambda t: -t[0]), 1):
        p("  %d. %s" % (i, h))
    p("")
    p("※ 5V로 쓴 것이 원인인지는 로그만으로 증명되지 않습니다. 멀티미터로 BT-210 VCC–GND(3.3V)를 재고, 3.3V로 고친 뒤 같은 시험을 다시 기록해 비교하세요.")
    return "\n".join(rep)


# ----------------------------------------------------------------------------- 기록
def list_ports():
    try:
        from serial.tools import list_ports as lp
    except ImportError:
        print("pyserial 이 없습니다:  python -m pip install pyserial")
        return
    ports = list(lp.comports())
    if not ports:
        print("찾은 포트가 없습니다. USB 케이블을 꽂고 다시 실행하세요.")
    for prt in ports:
        print("%-8s %s" % (prt.device, prt.description))


def now_stamp():
    return dt.datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]


def record(ports, baud, outdir):
    try:
        import serial
    except ImportError:
        print("pyserial 이 없습니다. 먼저 설치하세요:  python -m pip install pyserial")
        sys.exit(1)
    os.makedirs(outdir, exist_ok=True)
    path = os.path.join(outdir, dt.datetime.now().strftime("bt_%Y%m%d_%H%M%S.log"))
    f = open(path, "w", encoding="utf-8", buffering=1)
    lock = threading.Lock()
    stop = threading.Event()
    captured = []
    screen = queue.Queue()
    highlight = ("GAP", "HOLD", "SENDGAP", "ROBOT_SILENT", "ROBOT_BACK", "BOOT", "LATCHED", "READY", "FAULT")

    def save(line, show):
        with lock:
            f.write(line + "\n")
            captured.append(line)
        if show:
            screen.put(line)

    def reader(port):
        while not stop.is_set():
            try:
                s = serial.Serial(port, baud, timeout=0.05)
            except Exception as exc:  # 포트가 아직 없거나 보드가 재부팅 중
                screen.put("[%s] 여는 중... (%s)" % (port, exc))
                stop.wait(1.0)
                continue
            save("%s [%s] ## USB CONNECTED" % (now_stamp(), port), True)
            buf = b""
            try:
                while not stop.is_set():
                    n = getattr(s, "in_waiting", 0) or 1
                    chunk = s.read(n)
                    if not chunk:
                        continue
                    stamp = now_stamp()            # 읽은 순간의 시각 (줄마다 거의 정확)
                    buf += chunk
                    while b"\n" in buf:
                        raw, buf = buf.split(b"\n", 1)
                        text = raw.decode("utf-8", "replace").rstrip("\r")
                        show = any((" %s" % h) in text for h in highlight) or not text.startswith("#")
                        save("%s [%s] %s" % (stamp, port, text), show)
            except Exception as exc:  # USB가 빠짐 / 보드 리셋
                save("%s [%s] ## USB DISCONNECTED: %s" % (now_stamp(), port, exc), True)
            finally:
                try:
                    s.close()
                except Exception:
                    pass
                stop.wait(0.5)

    threads = [threading.Thread(target=reader, args=(prt,), daemon=True) for prt in ports]
    for t in threads:
        t.start()
    print("기록 중: %s   (멈추려면 Ctrl+C)" % path)
    print("화면에는 사건 줄(GAP, HOLD, SENDGAP, ROBOT_SILENT, BOOT, LATCHED ...)과 일반 메시지만 보입니다. 파일에는 전부 남습니다.")
    try:
        while True:
            try:
                print(screen.get(timeout=0.2))
            except queue.Empty:
                pass
    except KeyboardInterrupt:
        pass
    stop.set()
    for t in threads:
        t.join(timeout=2.0)
    with lock:
        f.close()
        lines = list(captured)
    report = analyze(lines)
    print(report)
    summary = summary_path(path)
    with open(summary, "w", encoding="utf-8") as g:
        g.write(report + "\n")
    print("\n로그: %s\n요약: %s" % (path, summary))
    return path, summary


def summary_path(log_path):
    return os.path.splitext(log_path)[0] + "_요약.txt"


# ----------------------------------------------------------------------------- 올리기 (GitHub 브랜치)
# 로그는 코드 브랜치가 아니라 로그 전용 브랜치(bt-logs)의 logs/<날짜>/ 에 커밋한다.
# git plumbing 과 임시 index 를 쓰므로 지금 체크아웃된 브랜치·작업 폴더·index 는 전혀 건드리지 않는다.
LOG_BRANCH = "bt-logs"
FALLBACK_NAME = "bt_log uploader"           # git 사용자 이름/이메일이 설정 안 된 PC에서 쓰는 이름
FALLBACK_EMAIL = "bt-log@localhost"
BRANCH_README = """# 블루투스 끊김 로그 (bt_log.py 업로드)

이 브랜치에는 코드가 없고 `teleoperation_log/tools/bt_log.py --upload` 로 올린 기록만 있습니다.

- `logs/<날짜>/bt_<날짜>_<시각>.log` : 전체 기록 (`#L ...` 리더, `#R ...` 로봇)
- `logs/<날짜>/bt_<날짜>_<시각>_요약.txt` : 끊김 원인 요약
- 커밋 메시지에 올린 사람이 적은 메모(시험 상황, LED 상태 등)가 있습니다.

읽는 법은 코드 브랜치의 `teleoperation_log/README.md` 4장을 보세요.
"""


class UploadError(Exception):
    pass


def _git(repo, args, env=None, data=None, check=True):
    p = subprocess.run(["git", "-C", repo] + args, input=data, stdout=subprocess.PIPE,
                       stderr=subprocess.PIPE, env=env)
    out = p.stdout.decode("utf-8", "replace").strip()
    if check and p.returncode != 0:
        raise UploadError("git %s 실패: %s" % (args[0], p.stderr.decode("utf-8", "replace").strip()))
    return out if check else (p.returncode, out, p.stderr.decode("utf-8", "replace").strip())


def find_repo(start):
    try:
        p = subprocess.run(["git", "-C", start, "rev-parse", "--show-toplevel"],
                           stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    except FileNotFoundError:
        raise UploadError("git 이 설치되어 있지 않습니다 (https://git-scm.com 에서 설치).")
    if p.returncode != 0:
        raise UploadError("이 폴더는 git 저장소가 아닙니다. GitHub 저장소를 clone 한 폴더의 "
                          "teleoperation_log/tools 에서 실행하세요.")
    return p.stdout.decode("utf-8", "replace").strip()


def github_web_url(repo, remote):
    rc, url, _ = _git(repo, ["remote", "get-url", remote], check=False)
    m = re.match(r"^(?:https://(?:[^@/]+@)?github\.com/|git@github\.com:)([^/]+)/(.+?)(?:\.git)?/?$", url or "")
    return "https://github.com/%s/%s" % (m.group(1), m.group(2)) if m else None


def _day_of(path):
    m = re.search(r"bt_(\d{4})(\d{2})(\d{2})_", os.path.basename(path))
    return "%s-%s-%s" % m.groups() if m else dt.date.today().isoformat()


def upload_logs(files, branch=LOG_BRANCH, remote="origin", note="", repo=None, retries=3, before_push=None):
    """files 를 <remote>/<branch> 의 logs/<날짜>/ 에 커밋해서 올린다. (올린 경로 목록, 웹 주소) 를 돌려준다."""
    files = [os.path.abspath(f) for f in files if f and os.path.isfile(f)]
    if not files:
        raise UploadError("올릴 파일이 없습니다.")
    repo = repo or find_repo(os.path.dirname(os.path.abspath(__file__)))
    env = dict(os.environ)
    env.setdefault("GIT_TERMINAL_PROMPT", "0")      # 터미널에서 비밀번호를 묻다가 멈추지 않게 (로그인 창은 뜸)
    if not _git(repo, ["config", "user.name"], check=False)[1]:
        env["GIT_AUTHOR_NAME"] = env["GIT_COMMITTER_NAME"] = FALLBACK_NAME
    if not _git(repo, ["config", "user.email"], check=False)[1]:
        env["GIT_AUTHOR_EMAIL"] = env["GIT_COMMITTER_EMAIL"] = FALLBACK_EMAIL
    tracking = "refs/remotes/%s/%s" % (remote, branch)
    last_err = ""
    for attempt in range(retries):
        rc, _, err = _git(repo, ["fetch", "--quiet", remote, "+refs/heads/%s:%s" % (branch, tracking)], env=env, check=False)
        if rc == 0:
            parent = _git(repo, ["rev-parse", "--verify", tracking + "^{commit}"])
        elif "couldn't find remote ref" in err or "could not find remote ref" in err:
            parent = None                       # 브랜치가 아직 없음: 처음 올리는 것
        else:
            raise UploadError("GitHub에서 '%s' 브랜치를 받아오지 못했습니다(인터넷·주소 확인): %s" % (branch, err))
        with tempfile.TemporaryDirectory() as td:
            ienv = dict(env)
            ienv["GIT_INDEX_FILE"] = os.path.join(td, "index")
            if parent:
                _git(repo, ["read-tree", parent], env=ienv)
            else:
                _git(repo, ["read-tree", "--empty"], env=ienv)
                blob = _git(repo, ["hash-object", "-w", "--stdin"], data=BRANCH_README.encode("utf-8"))
                _git(repo, ["update-index", "--add", "--cacheinfo", "100644,%s,README.md" % blob], env=ienv)
            existing = set(x for x in _git(repo, ["ls-files", "-z"], env=ienv).split("\0") if x)
            placed = []
            for f in files:
                target = "logs/%s/%s" % (_day_of(f), os.path.basename(f))
                stem, ext = os.path.splitext(target)
                n = 2
                while target in existing:       # 같은 이름이 이미 있으면 _2, _3 ...
                    target = "%s_%d%s" % (stem, n, ext)
                    n += 1
                blob = _git(repo, ["hash-object", "-w", "--no-filters", f])
                _git(repo, ["update-index", "--add", "--cacheinfo", "100644,%s,%s" % (blob, target)], env=ienv)
                existing.add(target)
                placed.append(target)
            tree = _git(repo, ["write-tree"], env=ienv)
        msg = "logs: " + ", ".join(os.path.basename(t) for t in placed)
        if note:
            msg += "\n\n" + note.strip()
        commit = _git(repo, ["commit-tree", tree, "-m", msg] + (["-p", parent] if parent else []), env=env)
        if before_push:
            before_push(attempt)
        rc, _, err = _git(repo, ["push", "--quiet", remote, "%s:refs/heads/%s" % (commit, branch)], env=env, check=False)
        if rc == 0:
            _git(repo, ["update-ref", tracking, commit], check=False)
            web = github_web_url(repo, remote)
            return placed, (web + "/tree/" + branch + "/" + os.path.dirname(placed[0])) if web else None
        last_err = err
        if any(k in err for k in ("non-fast-forward", "fetch first", "rejected")):
            continue                            # 그사이 다른 사람이 올렸다: 새로 받아서 다시
        if any(k in err.lower() for k in ("authentication", "permission", "403", "could not read username", "terminal prompts disabled")):
            raise UploadError("GitHub 로그인 또는 쓰기 권한이 필요합니다. 이 저장소에 쓰기 권한이 있는 계정으로 "
                              "'git push' 를 한 번 해서 로그인한 뒤 다시 올리세요.\n  (git 메시지: %s)" % err)
        raise UploadError("올리기(push) 실패: %s" % err)
    raise UploadError("다른 사람이 동시에 올리고 있어서 %d번 시도했지만 실패했습니다: %s" % (retries, last_err))


def do_upload(files, a):
    try:
        placed, url = upload_logs(files, branch=a.branch, remote=a.remote, note=a.note)
    except UploadError as exc:
        print("\n[올리기 실패] %s" % exc)
        print("로그 파일은 그대로 남아 있습니다. 나중에 다시: python bt_log.py --upload-file <로그 파일>")
        return False
    print("\n[올리기 완료] '%s' 브랜치에 올렸습니다:" % a.branch)
    for t in placed:
        print("  " + t)
    if url:
        print("  " + url)
    return True


def main():
    try:
        sys.stdout.reconfigure(encoding="utf-8")
    except Exception:
        pass
    ap = argparse.ArgumentParser(description="BTLOG 블루투스 끊김 로그 기록·분석")
    ap.add_argument("ports", nargs="*", help="COM 포트 (예: COM5). 리더 보드만 꽂아도 됨")
    ap.add_argument("--baud", type=int, default=115200, help="USB라서 1200만 아니면 아무 값이나 됨 (기본 115200)")
    ap.add_argument("--out", default=os.path.join(os.path.dirname(os.path.abspath(__file__)), "logs"))
    ap.add_argument("--analyze", metavar="FILE", help="저장된 로그(.log/.txt) 요약만 하기")
    ap.add_argument("--list", action="store_true", help="COM 포트 목록")
    ap.add_argument("--upload", action="store_true",
                    help="기록(또는 --analyze)이 끝나면 로그와 요약을 GitHub의 로그 브랜치에 올림")
    ap.add_argument("--upload-file", nargs="+", metavar="FILE", help="이미 있는 로그 파일을 올리기 (요약은 자동으로 같이)")
    ap.add_argument("--note", default="", help="올릴 때 커밋 메시지에 붙일 메모 (시험 상황, LED 상태 등)")
    ap.add_argument("--branch", default=LOG_BRANCH, help="로그를 올릴 브랜치 (기본 bt-logs)")
    ap.add_argument("--remote", default="origin", help="git remote 이름 (기본 origin)")
    a = ap.parse_args()
    if a.baud == 1200:
        ap.error("1200은 쓰면 안 됩니다: 이 보드는 1200으로 열었다 닫으면 업로드 모드로 리셋됩니다.")
    if a.list:
        list_ports()
    elif a.upload_file:
        files = []
        for f in a.upload_file:
            if not os.path.isfile(f):
                print("파일이 없습니다: %s" % f)
                continue
            files.append(f)
            if not f.endswith("_요약.txt"):
                summ = summary_path(f)
                if not os.path.isfile(summ):     # 요약이 없으면 지금 만들어서 같이 올린다
                    with open(summ, "w", encoding="utf-8") as g:
                        g.write(analyze(read_log_file(f)) + "\n")
                if summ not in files:
                    files.append(summ)
        if files and not do_upload(files, a):
            sys.exit(1)
    elif a.analyze:
        report = analyze(read_log_file(a.analyze))
        print(report)
        if a.upload:
            summ = summary_path(a.analyze)
            with open(summ, "w", encoding="utf-8") as g:
                g.write(report + "\n")
            if not do_upload([a.analyze, summ], a):
                sys.exit(1)
    elif a.ports:
        path, summ = record(a.ports, a.baud, a.out)
        if a.upload and not do_upload([path, summ], a):
            sys.exit(1)
    else:
        ap.print_help()


if __name__ == "__main__":
    main()
