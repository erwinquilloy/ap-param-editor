"""
Off-radio test of core.lua (the engine) against copies of ArduPilot's side:
  sport   fake_mavlite.FakeAP (AP_Frsky_MAVlite_*: S.Port uplink/downlink, F.Port duplicates)
  crsf    the same handler, MAVlite in one CRSF frame (Stratos / PR #35160 framing)
  mlrs    fake_mavap.FakeAP (pymavlink ArduPilot behind the mLRS cbridge, lossy, busy stream)
Runs every link on Lua 5.2 (EdgeTX) and 5.4 (Ethos).

  python tests/test_core.py
"""
import os, struct, sys
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import lupa
from fake_mavlite import FakeAP as LiteAP
from fake_mavap import FakeAP as MavAP

SRC = os.path.join(HERE, '..', 'src', 'common')
fails = 0


def check(cond, what):
    global fails
    if not cond:
        fails += 1
        print('  FAIL', what)


def f32(x):
    return struct.unpack('<f', struct.pack('<f', x))[0]


def lua_table(L, seq):
    return L.table_from(list(seq))


class Rig:
    def __init__(self, ver, kind, custom=None):
        self.L = (lupa.lua54 if ver == 54 else lupa.lua52).LuaRuntime(unpack_returned_tuples=True)
        L = self.L
        self.kind, self.t, self.ver = kind, 0.0, ver
        self.busy_once = False
        params = {'Q_ENABLE': 1.0, 'Q_ASSIST_SPEED': 0.2, 'AIRSPEED_CRUISE': 18.0, 'TRIM_THROTTLE': 45.0,
                  'ARMING_SKIPCHK': 0.0, 'RLL_RATE_P': 0.08, 'PTCH_RATE_FF': 0.345, 'THR_MAX': 100.0,
                  'AIRSPEED_MAX': 30.0, 'TECS_CLMB_MAX': 5.0, 'FBWB_CLIMB_RATE': 2.0, 'AIRSPEED_MIN': 9.0,
                  'STAB_PITCH_DOWN': 2.0, 'TECS_SINK_MIN': 2.0, 'TECS_PITCH_MIN': 0.0, 'TECS_SINK_MAX': 5.0,
                  'KFF_THR2PTCH': 0.0, 'TECS_PITCH_MAX': 15.0, 'SERVO1_TRIM': 1500.0, 'WPNAV_SPEED': 1000.0}
        if kind == 'mlrs':
            self.ap = MavAP()
            self.ap.p = dict(params)
            self.ap.needs_hello = True
            self.ap.loss = 0.1
        else:
            self.ap = LiteAP(params, dup=(kind == 'sport'))
            self.ap.armed = False
        self.custom = custom
        G = L.globals()
        rig = self

        def push(f):
            if rig.busy_once:
                rig.busy_once = False
                return False
            if f['sp']:
                check(rig.kind == 'sport', 'S.Port frame on ' + rig.kind)
                rig.ap.uplink(int(f['appId']), int(f['value']) & 0xFFFFFFFF)
                return True
            cmd = int(f['cmd'])
            data = [int(f['data'][i + 1]) for i in range(len(f['data']))]
            if cmd == 0xAA:
                rig.hb = getattr(rig, 'hb', 0) + 1
                if rig.kind == 'mlrs':
                    rig.ap.from_radio(cmd, data)
                return True
            check(rig.kind == 'crsf', 'CRSF MAVlite frame on ' + rig.kind)
            assert data[:3] == [0xC8, 0xEA, 0xF4] and data[3] == len(data) - 5, data
            rig.lite_crsf(data[4], bytes(data[5:]))
            return True

        def read_file(path):
            return rig.custom

        self.io = L.eval('''function(push, clock, readFile, src)
          return { clock = clock, push = push, readFile = readFile, customPath = "custom.txt",
                   loadList = function() return dofile(src .. "/list.lua") end }
        end''')(push, lambda: rig.t, read_file, SRC.replace('\\', '/'))
        self.C = L.eval('dofile')(os.path.join(SRC, 'core.lua').replace('\\', '/'))(self.io)

    # Stratos: same handler, answer as one CRSF frame F4 len msgid payload
    def lite_crsf(self, msgid, pl):
        ap = self.ap
        saved = ap.send_msg
        out = []
        ap.send_msg = lambda mid, payload: out.append((mid, payload))
        ntext = len(ap.texts)
        ap.handle(msgid, pl)
        ap.send_msg = saved
        self.crsf_down = getattr(self, 'crsf_down', [])
        for mid, payload in out:
            self.crsf_down.append((0x80, [0xF4, len(payload), mid] + list(payload)))
        for t in ap.texts[ntext:]:
            self.crsf_down.append((0x80, [0xF1, 4] + list(t.encode()) + [0]))

    def frames_in(self):
        C, L = self.C, self.L
        if self.kind == 'sport':
            # passthrough: armed bit in 0x5001, status texts as 0x5000 chunks (each sent twice)
            if int(self.t * 10) % 5 == 0:
                C.sport(0x10, 0x5001, 256 if self.ap.armed else 0)
            for txt in self.ap.texts:
                b = txt.encode() + b'\0'
                while len(b) % 4: b += b'\0'
                for i in range(0, len(b), 4):
                    v = (b[i] << 24) | (b[i + 1] << 16) | (b[i + 2] << 8) | b[i + 3]
                    if self.ver == 54 and v >= 2 ** 31: v -= 2 ** 32
                    C.sport(0x10, 0x5000, v); C.sport(0x10, 0x5000, v)
            self.ap.texts = []
            n = 0
            while self.ap.down and n < 4:
                a, v = self.ap.down.pop(0)
                if self.ver == 54 and v >= 2 ** 31: v -= 2 ** 32
                C.sport(0x32, a, v)
                n += 1
        elif self.kind == 'crsf':
            if int(self.t * 10) % 5 == 0:
                C.crsf(0x80, lua_table(L, [0xF0, 0x01, 0x50, 0, 1 if self.ap.armed else 0, 0, 0]))
            for cmd, d in getattr(self, 'crsf_down', [])[:2]:
                C.crsf(cmd, lua_table(L, d))
            self.crsf_down = getattr(self, 'crsf_down', [])[2:]
        else:
            # a real mLRS Tx also sends CRSF telemetry next to the bridge: native frames and
            # ArduPilot passthrough (for Yaapu). Auto must still take the bridge (radio, 2026-11-19:
            # "CRSF", then "No answer").
            if int(self.t * 10) % 5 == 0:
                C.crsf(0x80, lua_table(L, [0xF0, 0x01, 0x50, 0, 1 if self.ap.armed else 0, 0, 0]))
                C.crsf(0x08, lua_table(L, [0, 160, 0, 10, 0, 0, 0, 80]))
            self.ap.tick(self.t)
            for fr in self.ap.frames():
                C.crsf(0xAA, lua_table(L, fr))

    def step(self, n=1):
        for _ in range(n):
            self.t += 0.05
            self.frames_in()
            self.C.update()

    def until(self, cond, secs):
        end = self.t + secs
        while self.t < end:
            self.step()
            if cond(): return True
        return False

    def rows(self):
        r = self.C.rows()
        return [r[i + 1] for i in range(len(r))] if r else []

    def tabs(self):
        t = self.C.tabs()
        return [t[i + 1] for i in range(len(t))]

    def val(self, name):
        return self.C.value(name)


def run(ver, kind):
    print('lua%d %s' % (ver, kind))
    custom = "# my extras\nSERVO1_TRIM, Aileron trim, 800, 2200, 1\nwpnav_speed\nNOT_THERE\nbad name here\n"
    R = Rig(ver, kind, custom)
    C = R.C
    ok = R.until(lambda: C.blocked is None and 'VTOL' in R.tabs(), 20)
    check(ok, 'ready with VTOL tabs (Q_ENABLE 1): blocked=%s tabs=%s' % (C.blocked and list(C.blocked.values()), R.tabs()))
    check(C.link == {'sport': 'SPORT', 'crsf': 'CRSF', 'mlrs': 'MAVLINK'}[kind], 'link ' + str(C.link))
    check('CUSTOM' in R.tabs(), 'CUSTOM tab')
    check(C.customInfo == 'custom.txt line 5?', 'custom bad line: %s' % C.customInfo)
    if kind == 'mlrs': check(getattr(R, 'hb', 0) > 0, 'heartbeat woke the bridge')

    C.open('TECS')
    rows = R.rows()
    ok = R.until(lambda: all(R.val(r[1]) is not None for r in rows), 60)
    check(ok, 'TECS tab read: %s' % C.debug())
    check(abs(R.val('AIRSPEED_CRUISE') - 18.0) < 1e-6, 'AIRSPEED_CRUISE 18')
    check(R.val('TRIM_THROTTLE') == 45.0, 'TRIM_THROTTLE 45')
    check(C.text(rows[0]) == '45 %', 'text %r' % C.text(rows[0]))

    # save: cruise 18 -> 19.5, verified by the read-back
    i = [r[1] for r in rows].index('AIRSPEED_CRUISE') + 1
    check(C.beginEdit(i), 'beginEdit')
    R.step(10)
    C.nudge(1); C.nudge(1); C.nudge(1)
    check(C.editing()[1] == 19.5, 'draft %s' % C.editing()[1])
    R.busy_once = True   # the radio refuses the first push: must be retried
    check(C.save() == 'saving', 'save started')
    ok = R.until(lambda: C.phase == 'Saved and verified', 20)
    check(ok, 'saved and verified: %r' % C.phase)
    check(abs(R.ap.p['AIRSPEED_CRUISE'] - 19.5) < 1e-6, 'FC has 19.5')
    check(C.phaseKind == 'ok', 'phase kind ok')
    sets = R.ap.sets
    check(C.save() == 'closed', 'unchanged save just closes')
    R.step(40)
    check(R.ap.sets == sets, 'no second PARAM_SET')

    # clamp + -1 rule
    C.open('RETURN')
    R.until(lambda: R.val('RTL_ALTITUDE') is not None, 30)
    check(R.val('RTL_ALTITUDE') is False, 'RTL_ALTITUDE n/a (FC lacks it): %r' % R.val('RTL_ALTITUDE'))
    check(C.text(R.rows()[0]) == 'n/a', 'n/a text')

    # armed: view only, PID with Air edits
    R.ap.armed = True
    if kind == 'mlrs':
        orig = R.ap.mv.heartbeat_encode
        R.ap.mv.heartbeat_encode = lambda t, a, bm, cm, ss: orig(t, a, 128, cm, ss)
    R.until(lambda: C.armed, 5)
    check(C.armed, 'armed seen')
    C.open('TECS')
    R.until(lambda: R.val('TRIM_THROTTLE') is not None, 20)
    C.beginEdit(1)
    check(C.locked(C.editing()[0]), 'TECS locked while armed')
    C.nudge(1)
    check(C.editing()[1] == 45, 'nudge refused while locked')
    check(C.save() is None and C.phase == 'Disarm to save', 'save refused armed')
    C.endEdit()
    C.open('PID')
    R.until(lambda: R.val('RLL_RATE_P') is not None, 20)
    C.beginEdit(1)
    check(C.locked(C.editing()[0]), 'PID locked without Air edits')
    C.airEdits = True
    check(not C.locked(C.editing()[0]), 'PID open with Air edits')
    C.nudge(1)
    C.save()
    ok = R.until(lambda: C.phase == 'Saved and verified', 20)
    check(ok and abs(R.ap.p['RLL_RATE_P'] - f32(0.085)) < 1e-6, 'PID saved armed: %s %r' % (R.ap.p['RLL_RATE_P'], C.phase))
    C.airEdits = False
    R.ap.armed = False
    if kind == 'mlrs': R.ap.mv.heartbeat_encode = orig
    R.until(lambda: not C.armed, 5)

    # bits editor
    C.open('ARMING')
    R.until(lambda: R.val('ARMING_SKIPCHK') is not None, 20)
    C.beginEdit(1)
    R.step(10)
    C.toggleBit(2)   # Compass = bit 2
    check(C.editing()[1] == 4, 'bit toggled %s' % C.editing()[1])
    C.save()
    ok = R.until(lambda: C.phase == 'Saved and verified', 20)
    check(ok and R.ap.p['ARMING_SKIPCHK'] == 4, 'SKIPCHK saved 4')

    # custom tab: auto step, ranges from the file
    C.open('CUSTOM')
    names = [r[1] for r in R.rows()]
    check(names == ['SERVO1_TRIM', 'WPNAV_SPEED', 'NOT_THERE'], 'custom rows %s' % names)
    R.until(lambda: all(R.val(n) is not None for n in names), 40)
    check(R.val('SERVO1_TRIM') == 1500 and R.val('NOT_THERE') is False, 'custom values')
    C.beginEdit(1)
    R.step(10)
    for _ in range(3): C.nudge(1)
    check(C.editing()[1] == 1503, 'custom step 1: %s' % C.editing()[1])
    C.endEdit()
    C.beginEdit(2)
    R.step(10)
    C.nudge(-1)
    check(C.editing()[1] == 999, 'auto step 1 for integer value: %s' % C.editing()[1])
    C.endEdit()


def no_answer(ver):
    # passthrough heard, but the FC never answers MAVlite (old firmware over CRSF): "No answer" shown
    print('lua%d crsf no answer' % ver)
    R = Rig(ver, 'crsf')
    R.lite_crsf = lambda mid, pl: None
    ok = R.until(lambda: R.C.blocked is not None and R.C.blocked[1] == 'No answer', 30)
    check(ok, 'No answer screen: %s' % (R.C.blocked and list(R.C.blocked.values())))


def heartbeat_bytes():
    from pymavlink.dialects.v20 import ardupilotmega as mav
    m = mav.MAVLink(None, srcSystem=255, srcComponent=25)
    m.seq = 0
    pkt = m.heartbeat_encode(6, 8, 0, 0, 0).pack(m)
    L = lupa.lua54.LuaRuntime()
    io = L.eval('{ clock = function() return 0 end, push = function() end, loadList = function() end }')
    C = L.eval('dofile')(os.path.join(SRC, 'core.lua').replace('\\', '/'))(io)
    hb = [C._HB[i + 1] for i in range(len(C._HB))]
    check(hb == [0, len(pkt)] + list(pkt), 'HB frame == pymavlink heartbeat %s / %s' % (hb, list(pkt)))


def forced_source(ver):
    # telemetry source chosen by the user: only that link is used, and the choice is kept
    print('lua%d forced source' % ver)
    R = Rig(ver, 'crsf')
    saved = {}
    R.L.globals().pysave = lambda p, v: saved.__setitem__(p, v)
    R.io.writeFile = R.L.eval('function(p, v) pysave(p, v) end')
    R.io.settingsPath = 'settings.txt'
    R.C.setSource('SPORT')
    R.step(80)
    check(R.C.link == 'SPORT', 'forced link SPORT: %s' % R.C.link)
    b = R.C.blocked
    check(b is not None and b[1] == 'No S.Port / F.Port telemetry', 'blocked says no S.Port: %s' % (b and list(b.values())))
    check(saved.get('settings.txt') == 'source=SPORT\n', 'saved %r' % saved)
    R.C.nextSource()
    check(R.C.source == 'CRSF', 'next source CRSF')
    ok = R.until(lambda: R.C.blocked is None and R.C.link == 'CRSF', 20)
    check(ok, 'CRSF forced works')
    # read back at start
    R2 = Rig(ver, 'sport')
    R2.L.globals().src_txt = 'source=MAVLINK\n'
    io = R2.L.eval('''function(push, clock, src) return { clock = clock, push = push, settingsPath = "s",
       readFile = function(p) if p == "s" then return src_txt end end,
       loadList = function() return dofile(src .. "/list.lua") end } end''')(lambda f: True, lambda: 0, SRC.replace('\\', '/'))
    C2 = R2.L.eval('dofile')(os.path.join(SRC, 'core.lua').replace('\\', '/'))(io)
    check(C2.source == 'MAVLINK', 'source read from settings: %s' % C2.source)


if __name__ == '__main__':
    heartbeat_bytes()
    for ver in (52, 54):
        for kind in ('sport', 'crsf', 'mlrs'):
            run(ver, kind)
        no_answer(ver)
        forced_source(ver)
    print('FAILURES: %d' % fails if fails else 'ALL PASS')
    sys.exit(1 if fails else 0)
