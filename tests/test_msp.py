"""
Off-radio test of the INAV / Betaflight path: pick.lua (engine choice) + mspcore.lua, against
OmniDeck's fake flight controllers (fake_inav_settings.FakeInav, fake_bf.FakeBF: the replies of
INAV 9.1 fc_msp.c and Betaflight 4.5.5 / 2025.12.5 msp.c, chunked as msp_shared.c sends them).
Links: CRSF (MSP frames 0x7A / 0x7B), S.Port (primId 0x30 / 0x32), mLRS bridge (raw MSPv2 in 0x81 / 0x82).
Runs on Lua 5.2 (EdgeTX) and 5.4 (Ethos).

  python tests/test_msp.py
"""
import os, struct, sys
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import lupa
from fake_inav_settings import FakeInav
from fake_bf import FakeBF, PID, ADV, RC

SRC = os.path.join(HERE, '..', 'src', 'common').replace('\\', '/')
fails = 0


def check(cond, what):
    global fails
    if not cond:
        fails += 1
        print('  FAIL', what)


class Rig:
    def __init__(self, ver, link, fc, ap=False):
        self.L = (lupa.lua54 if ver == 54 else lupa.lua52).LuaRuntime(unpack_returned_tuples=True)
        L = self.L
        self.link, self.fc, self.t, self.ap = link, fc, 0.0, ap
        self.files = {}
        rig = self

        def push(f):
            if f['sp']:
                check(rig.link == 'sport', 'S.Port frame on ' + rig.link)
                if rig.fc: rig.fc.sport_push(0x0D, 0x30, int(f['appId']), int(f['value']))
                return True
            cmd = int(f['cmd'])
            data = [int(f['data'][i + 1]) for i in range(len(f['data']))]
            if rig.fc and rig.link != 'sport': rig.fc.crsf_push(cmd, data)
            return True

        def read_file(path): return rig.files.get(str(path))
        def write_file(path, s): rig.files[str(path)] = str(s)

        self.io = L.eval('''function(push, clock, readFile, writeFile, src)
          local function mod(n) return dofile(src .. "/" .. n) end
          return { clock = clock, push = push, readFile = readFile, writeFile = writeFile, version = "test",
                   settingsPath = "settings.txt", customPath = "custom.txt", inavCustomPath = "custom_inav.txt",
                   loadCore = function() return mod("core.lua") end, loadMsp = function() return mod("mspcore.lua") end,
                   loadList = function() return mod("list.lua") end, loadInav = function() return mod("ilist.lua") end,
                   loadBf = function() return mod("bflist.lua") end }
        end''')(push, lambda: rig.t, read_file, write_file, SRC)
        self.C = L.eval('dofile')(SRC + '/pick.lua')(self.io)

    def tick(self, n=1):
        C, L = self.C, self.L
        for _ in range(n):
            self.t += 0.05
            if self.link == 'sport':
                if int(self.t * 20) % 4 == 0:
                    C.sport(0x10, 0x5001 if self.ap else 0x0110, 0)   # FC telemetry (passthrough if ArduPilot)
            else:
                if int(self.t * 20) % 4 == 0:
                    C.crsf(0x1E, L.table_from([1, 2, 3, 4, 5, 6]))          # attitude (native)
                if self.ap and int(self.t * 10) % 5 == 0:
                    C.crsf(0x80, L.table_from([0xF0, 0x01, 0x50, 0, 0, 0, 0]))
            fc = self.fc
            n = 0
            while fc and fc.down and n < 6:
                item = fc.down.pop(0)
                n += 1
                if item[0] == 'sport':
                    C.sport(0x32, item[1], item[2])
                elif item[0] == 'crsf':
                    C.crsf(0x7B, L.table_from(item[1]))
                else:
                    cmd, d = item[1]
                    C.crsf(cmd, L.table_from(d))
            C.update()

    def until(self, cond, limit=2000):
        for _ in range(limit):
            if cond(): return True
            self.tick()
        return cond()

    def tabs(self):
        t = self.C.tabs()
        return [t[i + 1] for i in range(len(t))]

    def rows(self):
        r = self.C.rows()
        return [r[i + 1] for i in range(len(r))]

    def read_tab(self, g):
        self.C.open(g)
        self.C.view(1, 20)
        ok = self.until(lambda: self.C.progress()[0] == self.C.progress()[1], 3000)
        return ok

    def text(self, name):
        for r in self.rows():
            if r[1] == name: return self.C.text(r)
        return None

    def edit(self, name, steps):
        rows = self.rows()
        i = [r[1] for r in rows].index(name) + 1
        self.C.select(i)
        ok = self.C.beginEdit(i)
        check(ok, 'edit opens ' + name)
        self.until(lambda: not self.C.saving(), 200)
        for _ in range(abs(steps)): self.C.nudge(1 if steps > 0 else -1)
        r = self.C.save()
        check(r == 'saving', 'save of %s goes out (%s)' % (name, r))
        self.until(lambda: not self.C.saving(), 2000)


def inav_link(ver, link):
    print('inav %s lua%d' % (link, ver))
    fc = FakeInav(bridge=(link == 'mlrs'))
    R = Rig(ver, 'sport' if link == 'sport' else 'crsf', fc)
    C = R.C
    if link == 'mlrs': C.setSource('MAVLINK')
    ok = R.until(lambda: C.kind() == 'MSP' and C.blocked is None)
    check(ok, 'MSP engine ready (%s) %s' % (C.kind(), C.blocked and [C.blocked[i + 1] for i in range(len(C.blocked))]))
    check(C.fcText() == 'INAV 9.1.0', 'FC row: %s' % C.fcText())
    check(R.tabs()[:2] == ['FLIGHT', 'LAUNCH'] and 'FW PID' in R.tabs(), 'INAV tabs: %s' % R.tabs())
    check(R.read_tab('RTH'), 'RTH tab read')
    check(R.text('nav_rth_altitude') == '50 m', 'RTH altitude 5000 cm -> %s' % R.text('nav_rth_altitude'))
    check(R.text('nav_rth_alt_mode') == 'AT_LEAST', 'lookup by INAV name: %s' % R.text('nav_rth_alt_mode'))
    # a change: set, read back, stored (disarmed)
    R.edit('nav_rth_altitude', 2)
    check(fc.set['nav_rth_altitude']['v'] == 5200, 'INAV got 5200 cm (%s)' % fc.set['nav_rth_altitude']['v'])
    R.until(lambda: fc.stores >= 1 and 'stored' in C.phase, 400)
    check(fc.stores >= 1, 'EEPROM write sent when disarmed')
    check(C.phase == 'Saved, verified and stored', 'phase: %s' % C.phase)
    # a lookup row: by option
    R.edit('nav_rth_alt_mode', -1)
    check(fc.set['nav_rth_alt_mode']['v'] == 3, 'alt mode -> MAX (%s)' % fc.set['nav_rth_alt_mode']['v'])
    # armed: view only unless Air edits; with it the value is set at once, stored after the disarm
    fc.armed = True
    R.until(lambda: C.armed, 200)
    check(C.armed, 'armed state from MSP2_INAV_STATUS')
    rows = R.rows()
    check(C.locked(rows[0]), 'armed: locked')
    C.airEdits = True
    check(not C.locked(rows[0]), 'Air edits unlocks')
    stores = fc.stores
    R.edit('nav_rth_altitude', 1)
    check(fc.set['nav_rth_altitude']['v'] == 5300, 'armed set (%s)' % fc.set['nav_rth_altitude']['v'])
    R.tick(200)
    check(fc.stores == stores, 'not stored while armed')
    fc.armed = False
    R.until(lambda: fc.stores > stores, 400)
    check(fc.stores > stores, 'stored after the disarm')
    # PIDs: only the active platform's (fake: AIRPLANE)
    check(R.read_tab('FW PID'), 'FW PID read')
    r = [x for x in R.rows() if x[1] == 'fw_p_roll'][0]
    check(not C.locked(r), 'plane PIDs editable on AIRPLANE')
    R.read_tab('MC PID')
    r = [x for x in R.rows() if x[1] == 'mc_p_roll'][0]
    check(C.locked(r), 'hover PIDs locked on AIRPLANE')
    if link == 'mlrs': check(0x81 in fc.bridge_types, 'requests in mLRS mb frames')


def inav_old(ver):
    print('inav 9.0 lua%d' % ver)
    fc = FakeInav(ver=(9, 0, 0))
    R = Rig(ver, 'crsf', fc)
    R.until(lambda: R.C.kind() == 'MSP' and R.C.blocked is not None and 'INAV 9.0' in R.C.blocked[1])
    b = R.C.blocked
    check(b is not None and 'update to INAV 9.1' in ' '.join(b[i + 1] for i in range(len(b))), 'INAV 9.0 refused with a reason')


def betaflight(ver, api):
    print('betaflight api 1.%d lua%d' % (api, ver))
    fc = FakeBF(api=api)
    R = Rig(ver, 'crsf', fc)
    C = R.C
    ok = R.until(lambda: C.kind() == 'MSP' and C.blocked is None)
    check(ok, 'Betaflight ready')
    check(C.fcText().startswith('Betaflight ' + {46: '4.5', 47: '2025.12', 48: '2026.6'}[api]), 'FC row: %s' % C.fcText())
    check(R.tabs() == ['PID', 'FF D', 'TUNE', 'RATES', 'FILTERS'], 'BF tabs %s' % R.tabs())
    check(R.read_tab('PID'), 'PID read')
    before = fc.value(PID, 0)
    check(R.text('p_roll') == str(before), 'roll P %s == %s' % (R.text('p_roll'), before))
    R.edit('p_roll', 3)
    check(fc.value(PID, 0) == before + 3, 'roll P written (%s)' % fc.value(PID, 0))
    R.until(lambda: fc.stores >= 1, 400)
    check(fc.stores >= 1, 'BF EEPROM write')
    check(R.read_tab('RATES'), 'RATES read')
    names = [x[1] for x in R.rows()]
    check('roll_rc_rate' in names, 'rate rows')
    check(R.read_tab('FF D'), 'FF D read')
    want = 'd_min_roll' if api == 46 else 'd_max_roll'
    check(want in [x[1] for x in R.rows()], 'version names (%s)' % want)
    check(R.read_tab('FILTERS'), 'FILTERS read (2026.6: longer block)')
    check('dyn_notch_count' in [x[1] for x in R.rows()], 'filter rows')
    # the rows added for 0.2.x: TUNE tab, options shown by name, one field written
    check(R.read_tab('TUNE'), 'TUNE read')
    check(R.text('tpa_mode') == 'PD', 'TPA mode by name: %s' % R.text('tpa_mode'))
    check(R.text('tpa_breakpoint') == '1350 us', 'TPA breakpoint %s' % R.text('tpa_breakpoint'))
    adv0 = list(fc.block(ADV))
    R.edit('anti_gravity_gain', 1)
    check(fc.value(ADV, 21, 2) == 85, 'anti gravity written (%s)' % fc.value(ADV, 21, 2))
    a = fc.block(ADV)
    check([i for i in range(len(a)) if a[i] != adv0[i]] == [21], 'only anti gravity changed')
    check(R.read_tab('RATES'), 'RATES read again')
    check(R.text('throttle_limit_type') == 'OFF', 'throttle limit by name: %s' % R.text('throttle_limit_type'))


def ardupilot_stays(ver):
    print('ardupilot stays lua%d' % ver)
    R = Rig(ver, 'crsf', None, ap=True)
    R.tick(400)   # 20 s of passthrough, nothing answering parameter requests
    check(R.C.kind() == 'AP', 'ArduPilot passthrough keeps the ArduPilot engine (%s)' % R.C.kind())
    check(R.C.fcText() == 'ArduPilot', 'FC row %s' % R.C.fcText())


def nothing_answers(ver):
    print('no MSP answer lua%d' % ver)
    R = Rig(ver, 'crsf', None)
    went = R.until(lambda: R.C.kind() == 'MSP', 400)
    check(went, 'MSP asked when no ArduPilot')
    back = R.until(lambda: R.C.kind() == 'AP', 2000)
    check(back, 'back to waiting for ArduPilot when nothing answers MSP')
    R.tick(200)
    check(R.C.kind() == 'AP', 'MSP not asked again at once')


for v in (52, 54):
    for lk in ('crsf', 'sport', 'mlrs'):
        inav_link(v, lk)
    inav_old(v)
    betaflight(v, 46)
    betaflight(v, 47)
    betaflight(v, 48)
    ardupilot_stays(v)
    nothing_answers(v)

print('ALL PASS' if fails == 0 else 'FAILURES: %d' % fails)
sys.exit(1 if fails else 0)
