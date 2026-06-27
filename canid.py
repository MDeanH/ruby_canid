#!/usr/bin/env python3
"""
Ruby CAN identifier — offline decode + identify for ANY CAN bus.

Databases live in ~/canid/db:
  * opendbc/*.dbc      — comma.ai car DBCs (cantools), indexed by frame id
  * any extra *.dbc    — dropped in alongside (e.g. a J1939 dbc) are auto-indexed
  * embedded           — J1939 common PGN/SPN table + OBD-II mode-01 PIDs (below)

Memory-safe for the 1 GB Pi: an index (frame_id -> candidate dbcs) is built once and
cached to db/dbc_index.json; individual DBCs are loaded on demand with a small LRU,
so we never hold all 116 DBCs in RAM.

CLI:
  canid.py index                       (re)build the DBC index
  canid.py dbcs                        list databases + coverage
  canid.py decode <IDhex> <DATAhex> [--ext]
  canid.py live   [--channel can0] [--seconds N] [--bitrate B]
  canid.py analyze <candump.log>       heuristic identify of an unknown bus
"""
import os, sys, json, glob, time, argparse, statistics
from collections import defaultdict, OrderedDict

DB = os.path.expanduser("~/canid/db")
INDEX = os.path.join(DB, "dbc_index.json")

# ----------------------------------------------------------------- DBC index
def build_index():
    import cantools
    idx = defaultdict(list)
    files = sorted(glob.glob(os.path.join(DB, "**", "*.dbc"), recursive=True))
    ok = 0
    for f in files:
        try:
            db = cantools.database.load_file(f, strict=False)
        except Exception:
            continue
        ok += 1
        rel = os.path.relpath(f, DB)
        for m in db.messages:
            idx["%X" % m.frame_id].append([rel, m.name])
        del db
    os.makedirs(DB, exist_ok=True)
    json.dump(idx, open(INDEX, "w"))
    return len(idx), ok, len(files)

def load_index():
    try:
        return json.load(open(INDEX))
    except Exception:
        return {}

_DBCACHE = OrderedDict()
def _get_db(rel):
    import cantools
    if rel in _DBCACHE:
        _DBCACHE.move_to_end(rel); return _DBCACHE[rel]
    db = cantools.database.load_file(os.path.join(DB, rel), strict=False)
    _DBCACHE[rel] = db
    while len(_DBCACHE) > 8:
        _DBCACHE.popitem(last=False)
    return db

# ----------------------------------------------------------------- J1939
# pgn: (name, [(spn_name, start_byte, len_bytes, scale, offset, unit)])  little-endian SPNs
J1939_PGN = {
    61444: ("EEC1 electronic engine controller 1",
            [("EngineSpeed", 3, 2, 0.125, 0, "rpm"),
             ("ActualEngTorque", 2, 1, 1, -125, "%"),
             ("DriverDemandTorque", 1, 1, 1, -125, "%")]),
    61443: ("EEC2 electronic engine controller 2",
            [("AccelPedalPos", 1, 1, 0.4, 0, "%"),
             ("EnginePercentLoad", 2, 1, 1, 0, "%")]),
    65265: ("CCVS cruise control/vehicle speed",
            [("WheelBasedSpeed", 1, 2, 1/256.0, 0, "km/h")]),
    65262: ("ET1 engine temperature 1",
            [("CoolantTemp", 0, 1, 1, -40, "C"),
             ("FuelTemp", 1, 1, 1, -40, "C"),
             ("OilTemp", 2, 2, 0.03125, -273, "C")]),
    65266: ("LFE fuel economy",
            [("FuelRate", 0, 2, 0.05, 0, "L/h"),
             ("InstFuelEconomy", 2, 2, 1/512.0, 0, "km/L")]),
    65271: ("VEP1 vehicle electrical power",
            [("BatteryVoltage", 4, 2, 0.05, 0, "V"),
             ("AlternatorCurrent", 2, 1, 1, 0, "A")]),
    65276: ("DD1 dash display",
            [("FuelLevel", 1, 1, 0.4, 0, "%")]),
    65263: ("EFL/P1 engine fluid level/pressure 1",
            [("OilPressure", 3, 1, 4, 0, "kPa"),
             ("CoolantPressure", 6, 1, 2, 0, "kPa")]),
    65253: ("HOURS engine hours",
            [("TotalEngineHours", 0, 4, 0.05, 0, "h")]),
    65264: ("PTO power takeoff", []),
    65132: ("TCO1 tachograph", []),
    60928: ("Address Claim", []),
    65260: ("VI vehicle identification (VIN)", []),
    64932: ("Aftertreatment", []),
    65215: ("EBC2 wheel speed (ABS)",
            [("FrontAxleSpeed", 0, 2, 1/256.0, 0, "km/h")]),
}

def parse_j1939(can_id):
    pri = (can_id >> 26) & 0x7
    dp = (can_id >> 24) & 0x1
    pf = (can_id >> 16) & 0xFF
    ps = (can_id >> 8) & 0xFF
    sa = can_id & 0xFF
    if pf < 240:                       # PDU1: ps = destination address
        pgn = (dp << 16) | (pf << 8)
        da = ps
    else:                             # PDU2: ps = group extension
        pgn = (dp << 16) | (pf << 8) | ps
        da = None
    return pri, pgn, sa, da

def decode_j1939(can_id, data):
    pri, pgn, sa, da = parse_j1939(can_id)
    name, spns = J1939_PGN.get(pgn, (None, []))
    sigs = {}
    for (sn, sb, ln, sc, off, unit) in spns:
        if sb + ln <= len(data):
            raw = int.from_bytes(data[sb:sb + ln], "little")
            if raw not in (0xFF, 0xFFFF, 0xFFFFFFFF):    # J1939 "not available"
                sigs[sn] = "%.2f %s" % (raw * sc + off, unit)
    return {"proto": "J1939", "pgn": pgn, "name": name or ("PGN %d (unknown)" % pgn),
            "sa": sa, "da": da, "pri": pri, "signals": sigs}

# ----------------------------------------------------------------- OBD-II mode 01
def _u8(a, *_): return a
OBD_PID = {
    0x04: ("EngineLoad", "%", lambda a, *_: a * 100 / 255),
    0x05: ("CoolantTemp", "C", lambda a, *_: a - 40),
    0x0A: ("FuelPressure", "kPa", lambda a, *_: a * 3),
    0x0B: ("IntakeMAP", "kPa", lambda a, *_: a),
    0x0C: ("RPM", "rpm", lambda a, b, *_: (a * 256 + b) / 4),
    0x0D: ("Speed", "km/h", lambda a, *_: a),
    0x0E: ("TimingAdvance", "deg", lambda a, *_: a / 2 - 64),
    0x0F: ("IntakeTemp", "C", lambda a, *_: a - 40),
    0x10: ("MAF", "g/s", lambda a, b, *_: (a * 256 + b) / 100),
    0x11: ("Throttle", "%", lambda a, *_: a * 100 / 255),
    0x1F: ("RunTime", "s", lambda a, b, *_: a * 256 + b),
    0x21: ("DistanceMIL", "km", lambda a, b, *_: a * 256 + b),
    0x2F: ("FuelLevel", "%", lambda a, *_: a * 100 / 255),
    0x33: ("BaroPressure", "kPa", lambda a, *_: a),
    0x42: ("ModuleVoltage", "V", lambda a, b, *_: (a * 256 + b) / 1000),
    0x46: ("AmbientTemp", "C", lambda a, *_: a - 40),
    0x5C: ("OilTemp", "C", lambda a, *_: a - 40),
    0x5E: ("FuelRate", "L/h", lambda a, b, *_: (a * 256 + b) / 20),
}

def decode_obd(can_id, data):
    if not (can_id == 0x7DF or 0x7E0 <= can_id <= 0x7EF):
        return None
    if len(data) >= 3 and data[1] == 0x41:                # mode-01 response
        pid = data[2]; args = list(data[3:]) + [0, 0, 0, 0]
        if pid in OBD_PID:
            name, unit, fn = OBD_PID[pid]
            try:
                return {"proto": "OBD-II", "name": name, "value": "%.1f %s" % (fn(*args), unit)}
            except Exception:
                pass
        return {"proto": "OBD-II", "name": "mode01 PID 0x%02X" % pid, "raw": data.hex()}
    if len(data) >= 3 and data[1] == 0x01:                # request
        return {"proto": "OBD-II", "name": "request PID 0x%02X" % data[2]}
    return None

# ----------------------------------------------------------------- VESC (robot BLDC controllers)
def _be_s(d, o, n):
    return int.from_bytes(d[o:o + n], "big", signed=True) if len(d) >= o + n else None

# VESC 29-bit ID: vesc_id = id & 0xFF ; command = (id >> 8) & 0xFF. STATUS msgs are big-endian signed.
VESC_CMD = {0: "SET_DUTY", 1: "SET_CURRENT", 2: "SET_CURRENT_BRAKE", 3: "SET_RPM", 4: "SET_POS",
            5: "SET_CURRENT_REL", 6: "SET_CURRENT_BRAKE_REL", 7: "SET_HANDBRAKE", 8: "SET_HANDBRAKE_REL"}
VESC_CMD_SCALE = {0: 1e5, 1: 1e3, 2: 1e3, 3: 1.0, 4: 1e6}
VESC_STATUS = {
    9:  ("STATUS_1", [("ERPM", 0, 4, 1.0, "rpm"), ("Current", 4, 2, 0.1, "A"), ("Duty", 6, 2, 0.001, "")]),
    14: ("STATUS_2", [("AmpHours", 0, 4, 1e-4, "Ah"), ("AhCharged", 4, 4, 1e-4, "Ah")]),
    15: ("STATUS_3", [("WattHours", 0, 4, 1e-4, "Wh"), ("WhCharged", 4, 4, 1e-4, "Wh")]),
    16: ("STATUS_4", [("TempFET", 0, 2, 0.1, "C"), ("TempMot", 2, 2, 0.1, "C"), ("CurrentIn", 4, 2, 0.1, "A"), ("PIDpos", 6, 2, 0.02, "")]),
    27: ("STATUS_5", [("Tacho", 0, 4, 1 / 6.0, ""), ("Vin", 4, 2, 0.1, "V")]),
    28: ("STATUS_6", [("ADC1", 0, 2, 1e-3, ""), ("ADC2", 2, 2, 1e-3, ""), ("ADC3", 4, 2, 1e-3, ""), ("PPM", 6, 2, 1e-3, "")]),
}

def decode_vesc(can_id, data):
    cmd = (can_id >> 8) & 0xFF
    vid = can_id & 0xFF
    if cmd in VESC_STATUS:
        name, sigs = VESC_STATUS[cmd]
        out = {}
        for (sn, o, n, sc, u) in sigs:
            v = _be_s(data, o, n)
            if v is not None:
                out[sn] = ("%.2f %s" % (v * sc, u)).strip()
        return {"proto": "VESC", "name": "%s (vesc%d)" % (name, vid), "signals": out}
    if cmd in VESC_CMD:
        v = _be_s(data, 0, 4)
        val = "%.3f" % (v / VESC_CMD_SCALE[cmd]) if (v is not None and cmd in VESC_CMD_SCALE) else "?"
        return {"proto": "VESC", "name": "%s vesc%d" % (VESC_CMD[cmd], vid), "value": val}
    return None

# ----------------------------------------------------------------- CANopen (predefined connection set)
_CO_RANGES = [(0x081, 0x0FF, "EMCY", 0x80), (0x180, 0x1FF, "TPDO1", 0x180), (0x200, 0x27F, "RPDO1", 0x200),
              (0x280, 0x2FF, "TPDO2", 0x280), (0x300, 0x37F, "RPDO2", 0x300), (0x380, 0x3FF, "TPDO3", 0x380),
              (0x400, 0x47F, "RPDO3", 0x400), (0x480, 0x4FF, "TPDO4", 0x480), (0x500, 0x57F, "RPDO4", 0x500),
              (0x580, 0x5FF, "SDOtx", 0x580), (0x600, 0x67F, "SDOrx", 0x600), (0x700, 0x77F, "Heartbeat", 0x700)]

def decode_canopen(can_id, data):
    """Tentative CANopen interpretation by COB-ID; used only as an 11-bit fallback (no DBC match)."""
    if can_id == 0x000:
        return {"proto": "CANopen?", "name": "NMT"}
    if can_id == 0x080:
        return {"proto": "CANopen?", "name": "SYNC"}
    if can_id == 0x100:
        return {"proto": "CANopen?", "name": "TIME"}
    for lo, hi, name, base in _CO_RANGES:
        if lo <= can_id <= hi:
            return {"proto": "CANopen?", "name": "%s node%d" % (name, can_id - base)}
    return None

# ----------------------------------------------------------------- DJI RoboMaster (C610/C620/GM6020)
def decode_robomaster(can_id, data):
    if 0x201 <= can_id <= 0x20B and len(data) >= 7:          # ESC feedback (big-endian)
        ang = int.from_bytes(data[0:2], "big")               # 0..8191 -> 0..360 deg
        return {"proto": "RoboMaster", "name": "ESC fb id%d" % (can_id - 0x200),
                "signals": {"Angle": "%.1fdeg" % (ang * 360.0 / 8192.0),
                            "RPM": "%d" % _be_s(data, 2, 2),
                            "Torq": "%d" % _be_s(data, 4, 2), "Temp": "%dC" % data[6]}}
    if can_id in (0x200, 0x1FF, 0x2FF):                      # current/voltage command, 4x int16 BE
        grp = {0x200: "ESC1-4", 0x1FF: "ESC5-8/gimbal", 0x2FF: "gimbal"}[can_id]
        sig = {("m%d" % (i + 1)): "%d" % _be_s(data, i * 2, 2)
               for i in range(4) if _be_s(data, i * 2, 2) is not None}
        return {"proto": "RoboMaster", "name": "cmd %s" % grp, "signals": sig}
    return None

# ----------------------------------------------------------------- Xiaomi CyberGear micromotor
# 29-bit id: type=(id>>24)&0x1F, motor=(id>>8)&0xFF, host=id&0xFF. Type-2 feedback data = big-endian.
def decode_cybergear(can_id, data):
    if ((can_id >> 24) & 0x1F) == 2 and len(data) >= 8:           # motor feedback frame
        mid = (can_id >> 8) & 0xFF
        faults = (can_id >> 16) & 0x3F
        mode = (can_id >> 22) & 0x3
        u = lambda o: (data[o] << 8) | data[o + 1]
        sig = {"Angle": "%.2f rad" % (u(0) / 65535.0 * 25.13274 - 12.56637),
               "Vel": "%.2f rad/s" % (u(2) / 65535.0 * 60.0 - 30.0),
               "Torque": "%.2f Nm" % (u(4) / 65535.0 * 24.0 - 12.0),
               "Temp": "%.1f C" % (u(6) / 10.0)}
        if faults:
            sig["FAULT"] = "0x%02X" % faults
        m = ("reset", "cal", "RUN")[mode] if mode < 3 else "?"
        return {"proto": "CyberGear", "name": "motor%d fb [%s]" % (mid, m), "signals": sig}
    return None

# ----------------------------------------------------------------- ServeBot S1 (the user's own robot, RE'd)
# Reverse-engineered telemetry map (see ruby-motor-can-re). 29-bit, little-endian.
def _sv(data, o, n, typ):
    if len(data) < o + n:
        return None
    return int.from_bytes(data[o:o + n], "little", signed=(typ != "u"))
SERVEBOT = {  # id: (name, [(sig, off, len, scale, unit, type i/u)])
    0x0CB00320: ("speed", [("L", 0, 2, 1, "", "i"), ("R", 2, 2, 1, "", "i")]),
    0x0CB00334: ("odometer", [("L", 0, 4, 1, "", "i"), ("R", 4, 4, 1, "", "i")]),
    0x0CB0033E: ("current", [("I", 0, 2, 1, "", "i")]),
    0x0CB0032F: ("accel", [("X", 0, 2, 1 / 4125.0, "g", "i"), ("Y", 2, 2, 1 / 4125.0, "g", "i"), ("Z", 4, 2, 1 / 4125.0, "g", "i")]),
    0x0CB00329: ("tilt", [("roll", 0, 2, 1 / 10000.0, "rad", "i"), ("pitch", 2, 2, 1 / 10000.0, "rad", "i")]),
    0x0CB0032C: ("gyro", [("X", 0, 2, 1, "", "i"), ("Y", 2, 2, 1, "", "i"), ("Z", 4, 2, 1, "", "i")]),
    0x0CB0033A: ("voltage", [("V", 6, 2, 1e-3, "V", "u")]),
    0x0CB0031E: ("armed", [("flag", 0, 2, 1, "", "u")]),
    0x0CB00316: ("setpoint", [("lin", 0, 2, 1, "", "i"), ("turn", 2, 2, 1, "", "i")]),
    0x0DB20550: ("DRIVE_fwd", [("cmd", 0, 2, 1, "", "i")]),
    0x0DB20551: ("DRIVE_turn", [("cmd", 0, 2, 1, "", "i")]),
    0x0CB00343: ("fault", [("f", 0, 2, 1, "", "i")]),
}
def decode_servebot(can_id, data):
    spec = SERVEBOT.get(can_id)
    if not spec:
        return None
    name, sigs = spec
    out = {}
    for (sn, o, n, sc, u, typ) in sigs:
        v = _sv(data, o, n, typ)
        if v is not None:
            out[sn] = ("%.3f %s" % (v * sc, u)).strip() if sc != 1 else str(v)
    return {"proto": "ServeBot", "name": name, "signals": out}

# ----------------------------------------------------------------- decode (all paths)
PROFILES = ("auto", "car", "servebot", "robomaster", "cybergear", "canopen", "vesc", "j1939", "obd")
_PROFILE_PREF = {"car": "DBC", "servebot": "ServeBot", "robomaster": "RoboMaster", "cybergear": "CyberGear",
                 "canopen": "CANopen?", "vesc": "VESC", "j1939": "J1939", "obd": "OBD-II"}  # exact emitted proto strings

def _confidence(it):
    """Heuristic 0-100 that this interpretation is the REAL protocol for the frame, so the
    'auto' profile can rank concrete decodes above generic/speculative fallbacks. A specific
    --profile still wins via the float-to-front key; this only orders the rest."""
    p = it.get("proto", "")
    nm = (it.get("name", "") or "").lower()
    if p == "ServeBot":
        return 98                       # exact 29-bit id match against the RE'd map
    if p == "CyberGear":
        return 90                       # 29-bit type-2 feedback (id-shape match) > a bare DBC frame-id hit
    if p == "RoboMaster":
        return 90                       # id-range AND len>=7 check > a bare DBC frame-id hit (opendbc covers ~87% of 11-bit ids)
    if p == "DBC":
        return 88                       # opendbc frame-id matched and signals decoded
    if p == "OBD-II":
        return 58 if nm.startswith(("mode01", "request")) else 84
    if p == "J1939":
        return 12 if "unknown" in nm else 70    # known PGN solid; unknown = honest "dunno"
    if p == "CANopen?":
        return 40                       # tentative COB-ID range (11-bit, no-DBC fallback)
    if p == "DBC?":
        return 15                       # frame-id matched but the DBC decode raised
    if p == "VESC":
        return 10                       # (id>>8)&0xFF matches almost any 29-bit id; trust only via --profile vesc
    return 25

def decode(can_id, data, extended=False, index=None, profile=None):
    data = bytes(data)
    out = []
    dbc_hit = False
    if index is not None:
        for rel, msg in index.get("%X" % can_id, [])[:4]:
            try:
                db = _get_db(rel)
                dec = db.decode_message(can_id, data)
                sig = {k: (round(v, 3) if isinstance(v, float) else v) for k, v in dec.items()}
                out.append({"proto": "DBC", "db": os.path.basename(rel), "name": msg, "signals": sig})
                dbc_hit = True
            except Exception:
                out.append({"proto": "DBC?", "db": os.path.basename(rel), "name": msg})
    if extended or can_id > 0x7FF:
        sb = decode_servebot(can_id, data)
        if sb:
            out.append(sb)
        v = decode_vesc(can_id, data)
        if v:
            out.append(v)
        cg = decode_cybergear(can_id, data)
        if cg:
            out.append(cg)
        out.append(decode_j1939(can_id, data))
    else:                                       # 11-bit standard
        rm = decode_robomaster(can_id, data)
        if rm:
            out.append(rm)
        if not dbc_hit:
            co = decode_canopen(can_id, data)
            if co:
                out.append(co)
    o = decode_obd(can_id, data)
    if o:
        out.append(o)
    pref = _PROFILE_PREF.get(profile)               # specific profile floats its protocol to the front (exact
    out.sort(key=lambda it: (0 if (pref and it.get("proto", "") == pref) else 1, -_confidence(it)))
    return out                                      # match: a failed "DBC?" never floats above a real decode)

# ----------------------------------------------------------------- heuristic identify
def classify_byte(vals, n):
    uniq = len(set(vals))
    if uniq == 1:
        return "const(0x%02X)" % vals[0]
    diffs = [(vals[i + 1] - vals[i]) & 0xFF for i in range(len(vals) - 1)]
    if diffs and sum(1 for x in diffs if x == 1) / len(diffs) > 0.7:
        return "counter"
    if diffs and sum(1 for x in diffs if x in (1, 2, 3, 4)) / len(diffs) > 0.8:
        return "counter~"
    if uniq >= min(240, max(8, n * 0.7)):
        return "crc/random"
    if uniq <= 16:
        return "enum/flags(%d)" % uniq
    return "value"

def analyze(frames):
    by_id = defaultdict(list)
    for ts, cid, ext, d in frames:
        by_id[(cid, ext)].append((ts, d))
    report = []
    for (cid, ext), lst in sorted(by_id.items()):
        tss = [t for t, _ in lst]; datas = [d for _, d in lst]
        n = len(lst)
        dts = [tss[i + 1] - tss[i] for i in range(len(tss) - 1)]
        period = (statistics.median(dts) * 1000) if dts else 0.0
        dlc = max((len(d) for d in datas), default=0)
        bs = []
        for bi in range(dlc):
            vals = [d[bi] for d in datas if len(d) > bi]
            if vals:
                bs.append("b%d=%s" % (bi, classify_byte(vals, n)))
        report.append({"id": ("%X" % cid) + ("x" if ext else ""), "count": n,
                       "period_ms": round(period, 1), "dlc": dlc, "bytes": bs})
    report.sort(key=lambda r: -r["count"])
    return report

# ----------------------------------------------------------------- candump parsing
import re as _re
_LINE = _re.compile(r'\(([\d.]+)\)\s+\S+\s+([0-9A-Fa-f]+)#([0-9A-Fa-f]*)')
def read_candump(path):
    out = []
    for ln in open(path):
        m = _LINE.match(ln.strip())
        if not m:
            continue
        cid = int(m.group(2), 16); ext = len(m.group(2)) > 3
        try:
            data = bytes.fromhex(m.group(3))
        except ValueError:
            continue
        out.append((float(m.group(1)), cid & 0x1FFFFFFF, ext, data))
    return out

# ----------------------------------------------------------------- pretty print
def fmt(can_id, ext, data, interps):
    head = "%-8s [%d] %s" % (("%X" % can_id) + ("x" if ext else ""), len(data), data.hex())
    if not interps:
        return head + "   ?? unknown"
    parts = []
    for it in interps:
        if it["proto"].startswith("DBC"):
            s = " ".join("%s=%s" % (k, v) for k, v in list(it.get("signals", {}).items())[:6])
            parts.append("%s:%s %s" % (it["db"], it["name"], s))
        elif it["proto"] == "J1939":
            s = " ".join("%s=%s" % (k, v) for k, v in it.get("signals", {}).items())
            parts.append("J1939 PGN%d %s SA%d %s" % (it["pgn"], it["name"], it["sa"], s))
        elif it["proto"] == "OBD-II":
            parts.append("OBD %s %s" % (it.get("name", ""), it.get("value", "")))
        elif it["proto"] == "VESC":
            s = " ".join("%s=%s" % (k, v) for k, v in it.get("signals", {}).items())
            parts.append(("VESC %s %s %s" % (it.get("name", ""), s, it.get("value", ""))).strip())
        elif it["proto"] == "CANopen?":
            parts.append("CANopen? %s" % it.get("name", ""))
        elif it["proto"] == "RoboMaster":
            s = " ".join("%s=%s" % (k, v) for k, v in it.get("signals", {}).items())
            parts.append("RoboMaster %s %s" % (it.get("name", ""), s))
        elif it["proto"] == "CyberGear":
            s = " ".join("%s=%s" % (k, v) for k, v in it.get("signals", {}).items())
            parts.append("CyberGear %s %s" % (it.get("name", ""), s))
        elif it["proto"] == "ServeBot":
            s = " ".join("%s=%s" % (k, v) for k, v in it.get("signals", {}).items())
            parts.append("ServeBot %s %s" % (it.get("name", ""), s))
    return head + "   " + " | ".join(parts)

# ----------------------------------------------------------------- CLI
def main():
    ap = argparse.ArgumentParser(prog="canid")
    sub = ap.add_subparsers(dest="cmd")
    sub.add_parser("index")
    sub.add_parser("dbcs")
    d = sub.add_parser("decode"); d.add_argument("id"); d.add_argument("data"); d.add_argument("--ext", action="store_true"); d.add_argument("--profile", default=None, choices=PROFILES)
    lv = sub.add_parser("live"); lv.add_argument("--channel", default="can0"); lv.add_argument("--seconds", type=float, default=0); lv.add_argument("--bitrate", type=int, default=0); lv.add_argument("--profile", default=None, choices=PROFILES)
    an = sub.add_parser("analyze"); an.add_argument("log")
    a = ap.parse_args()

    if a.cmd == "index":
        ids, ok, total = build_index()
        print("indexed %d unique frame-ids from %d/%d DBCs -> %s" % (ids, ok, total, INDEX)); return
    if a.cmd == "dbcs":
        idx = load_index()
        files = sorted(glob.glob(os.path.join(DB, "**", "*.dbc"), recursive=True))
        print("DBC files: %d   indexed frame-ids: %d" % (len(files), len(idx)))
        print("J1939 PGNs: %d  OBD-II PIDs: %d  VESC msgs: %d  + CANopen connection-set + any *.dbc (e.g. ODrive)"
              % (len(J1939_PGN), len(OBD_PID), len(VESC_STATUS) + len(VESC_CMD)))
        for f in files[:6]:
            print("  ", os.path.relpath(f, DB))
        if len(files) > 6:
            print("   ... and %d more" % (len(files) - 6)); return
        return
    if a.cmd == "decode":
        idx = load_index()
        cid = int(a.id, 16); ext = a.ext or len(a.id) > 3
        print(fmt(cid, ext, bytes.fromhex(a.data), decode(cid, bytes.fromhex(a.data), ext, idx, a.profile))); return
    if a.cmd == "analyze":
        rep = analyze(read_candump(a.log))
        print("%-9s %6s %8s %3s  bytes" % ("id", "count", "period", "dlc"))
        for r in rep:
            print("%-9s %6d %7.1fms %3d  %s" % (r["id"], r["count"], r["period_ms"], r["dlc"], "  ".join(r["bytes"])))
        return
    if a.cmd == "live":
        import can
        idx = load_index()
        kw = {"channel": a.channel, "interface": "socketcan"}
        bus = can.interface.Bus(**kw)
        t0 = time.time()
        try:
            while True:
                m = bus.recv(timeout=1.0)
                if m is None:
                    if a.seconds and time.time() - t0 > a.seconds:
                        break
                    continue
                cid = m.arbitration_id & (0x1FFFFFFF if m.is_extended_id else 0x7FF)
                print(fmt(cid, m.is_extended_id, bytes(m.data), decode(cid, bytes(m.data), m.is_extended_id, idx, a.profile)))
                if a.seconds and time.time() - t0 > a.seconds:
                    break
        except KeyboardInterrupt:
            pass
        finally:
            bus.shutdown()
        return
    ap.print_help()

if __name__ == "__main__":
    main()
