"""
UI tests: ui_color.lua at several radio screen sizes and ui_bw.lua at 128x64 / 212x64, driven by
keys and taps against the fake flight controllers of test_core.py; every screen saved as a PNG in
tests/out/ (fonts approximate). Then the two platform entry scripts (EdgeTX APParam.lua, Ethos
main.lua) run once each with stubbed radio APIs, from the built dist/ folders.

  python tools/build.py && python tests/test_ui.py
"""
import os, sys
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from PIL import Image, ImageDraw, ImageFont
import test_core
from test_core import Rig, check, SRC

OUT = os.path.join(HERE, 'out')
os.makedirs(OUT, exist_ok=True)
DIST = os.path.join(HERE, '..', 'dist')


def font(px):
    for f in ('arial.ttf', 'DejaVuSans.ttf'):
        try: return ImageFont.truetype(f, px)
        except OSError: pass
    return ImageFont.load_default()


class Canvas:
    """g adapter for ui_color.lua, drawing into a PIL image."""
    def __init__(self, L, w, h, big=False):
        self.w, self.h = w, h
        self.px = {'s': 13, 'm': 18, 'l': 30} if not big else {'s': 16, 'm': 22, 'l': 36}
        self.fonts = {k: font(v) for k, v in self.px.items()}
        self.img = Image.new('RGB', (w, h))
        self.d = ImageDraw.Draw(self.img)
        c = self
        g = L.table()
        g.W, g.H = w, h
        g.rgb = lambda r, gr, b: (int(r) << 16) | (int(gr) << 8) | int(b)
        def rgb(v): v = int(v); return ((v >> 16) & 255, (v >> 8) & 255, v & 255)
        def rect(x, y, ww, hh, col):
            if ww > 0 and hh > 0: c.d.rectangle([x, y, x + ww - 1, y + hh - 1], fill=rgb(col))
        def frame(x, y, ww, hh, col): c.d.rectangle([x, y, x + ww - 1, y + hh - 1], outline=rgb(col))
        def text(x, y, s, f, col, align=None):
            s = str(s); fo = c.fonts[f]
            tw = c.d.textlength(s, font=fo)
            if align == 'r': x -= tw
            elif align == 'c': x -= tw / 2
            c.d.text((x, y), s, font=fo, fill=rgb(col))
        g.rect, g.frame, g.text = rect, frame, text
        g.textW = lambda s, f: int(c.d.textlength(str(s), font=c.fonts[f]))
        g.fontH = lambda f: c.px[f] + 4
        self.g = g

    def save(self, name):
        self.img.save(os.path.join(OUT, name + '.png'))


def colour(ver, kind, w, h):
    tag = 'c%dx%d_%s_lua%d' % (w, h, kind, ver)
    print(tag)
    R = Rig(ver, kind, "SERVO1_TRIM, Aileron trim, 800, 2200, 1\n")
    L = R.L
    U = L.eval('function(f, C) return f(C, "AP Param Editor") end')(
        L.eval('dofile')(os.path.join(SRC, 'ui_color.lua').replace('\\', '/')), R.C)
    cv = Canvas(L, w, h, big=w >= 800)
    def paint(name=None):
        U.paint(cv.g)
        if name: cv.save(tag + '_' + name)
    paint('0_waiting')
    R.until(lambda: R.C.blocked is None and 'VTOL' in R.tabs(), 20)
    paint('1_tabs')
    # open TECS by keys: tab index of TECS
    for _ in range(R.tabs().index('TECS')): U.key('next')
    U.key('enter')
    check(U.view() == 'rows', 'rows view')
    R.until(lambda: all(R.val(r[1]) is not None for r in R.rows()), 60)
    paint('2_rows')
    # tap on the second row: selects; tap again: opens the editor
    hH = max(cv.g.fontH('m') + 10, h // 10)
    rh = max(cv.g.fontH('m') + 10, (h - 8 - hH) // 9)
    y = hH + 4 + rh + rh // 2
    U.tap(20, y); paint()
    U.tap(20, y)
    check(U.view() == 'edit', 'tap opens editor: %s' % U.view())
    R.step(20)
    paint('3_edit')
    # + button: second row from the bottom, third of four
    bH = max(cv.g.fontH('m') + 14, h // 8)
    by1 = h - bH - 4 - bH - 8
    bw4 = (w - 8 * 5) // 4
    before = R.C.editing()[1]
    U.tap(8 * 3 + bw4 * 2 + bw4 // 2, by1 + bH // 2)
    check(R.C.editing()[1] == before + 0.5, '+ button: %s -> %s' % (before, R.C.editing()[1]))
    paint('4_edited')
    U.tap(w - 20, h - bH // 2 - 4)   # Save
    R.until(lambda: R.C.phase == 'Saved and verified', 20)
    paint('5_saved')
    check(R.C.phase == 'Saved and verified', 'saved by tap')
    U.key('back'); U.key('back')
    check(U.view() == 'tabs', 'back to tabs')
    # ARMING: bits editor
    for _ in range(10): U.key('prev')
    for _ in range(R.tabs().index('ARMING')): U.key('next')
    U.key('enter')
    R.until(lambda: R.val('ARMING_SKIPCHK') is not None, 20)
    U.key('enter')
    R.step(10)
    U.key('next'); U.key('enter')      # Compass
    paint('6_bits')
    check(R.C.editing()[1] == 4, 'bits by key')
    U.key('back'); U.key('back')
    # telemetry source item: last but one
    for _ in range(20): U.key('next')
    U.key('prev'); U.key('enter')
    check(R.C.source == 'SPORT', 'source from tab list: %s' % R.C.source)
    if kind == 'sport': U.key('enter')   # a source this rig doesn't have
    R.step(80)
    paint('7_blocked_source')
    check(R.C.blocked is not None, 'blocked with wrong source')
    was = R.C.source
    check(U.key('enter') is True and R.C.source != was, 'ENTER cycles source on blocked screen')
    check(U.key('back') is False, 'RTN on blocked screen leaves')
    R.C.setSource('AUTO')
    R.until(lambda: R.C.blocked is None, 20)
    for _ in range(20): U.key('prev')
    check(U.key('back') is False, 'RTN on tab list leaves')
    # slide scrolls the tab list
    U.slide(-rh * 3)
    paint('8_slid')


class Mono:
    """lcd stub for ui_bw.lua: 1-bit image."""
    def __init__(self, L, w):
        self.img = Image.new('1', (w, 64), 1); self.d = ImageDraw.Draw(self.img); self.f = ImageFont.load_default()
        G = L.globals(); c = self
        G.LCD_W, G.LCD_H = w, 64
        G.FORCE, G.INVERS, G.RIGHT, G.SMLSIZE, G.MIDSIZE = 1, 2, 4, 8, 16
        lcd = L.table()
        lcd.clear = lambda *a: c.d.rectangle([0, 0, w, 64], fill=1)
        def fr(x, y, ww, hh, f=0): c.d.rectangle([x, y, x + ww - 1, y + hh - 1], fill=0)
        def tx(x, y, s, f=0):
            f = int(f or 0); s = str(s)
            tw = c.d.textlength(s, font=c.f)
            if f & 4: x -= tw
            c.d.text((x, y - 2), s, font=c.f, fill=1 if f & 2 else 0)
        lcd.drawFilledRectangle, lcd.drawText = fr, tx
        G.lcd = lcd

    def save(self, name): self.img.convert('L').resize((self.img.width * 3, 192), Image.NEAREST).save(os.path.join(OUT, name + '.png'))


def mono(ver, w):
    tag = 'bw%d_lua%d' % (w, ver)
    print(tag)
    R = Rig(ver, 'sport')
    L = R.L
    m = Mono(L, w)
    U = L.eval('function(f, C) return f(C, "AP Param Editor") end')(
        L.eval('dofile')(os.path.join(SRC, '..', 'edgetx', 'ui_bw.lua').replace('\\', '/')), R.C)
    U.paint(); m.save(tag + '_0_waiting')
    R.until(lambda: R.C.blocked is None and 'VTOL' in R.tabs(), 20)
    U.paint(); m.save(tag + '_1_tabs')
    for _ in range(R.tabs().index('PID')): U.key('next')
    U.key('enter')
    R.until(lambda: all(R.val(r[1]) is not None for r in R.rows()), 60)
    U.paint(); m.save(tag + '_2_rows')
    U.key('enter'); R.step(10); U.key('next')
    U.paint(); m.save(tag + '_3_edit')
    U.key('enter')
    ok = R.until(lambda: R.C.phase == 'Saved and verified', 20)
    check(ok, 'bw save')
    U.paint(); m.save(tag + '_4_saved')
    U.key('back'); U.key('back')
    for _ in range(20): U.key('next')
    U.key('prev')
    U.paint(); m.save(tag + '_5_source_item')
    check(U.key('back') is False, 'bw RTN leaves')


def edgetx_entry(ver):
    """APParam.lua from dist/ with EdgeTX stubs: S.Port answers from the fake FC."""
    print('edgetx entry lua%d' % ver)
    R = Rig(ver, 'sport')
    L = R.L
    G = L.globals()
    root = os.path.join(DIST, 'EdgeTX').replace('\\', '/')
    G.ROOTDIR = root
    L.execute('''
      LCD_W, LCD_H = 480, 272
      SMLSIZE, MIDSIZE, RIGHT, CENTER, INVERS, FORCE = 0x100, 0x200, 0x400, 0x800, 2, 1
      EVT_VIRTUAL_NEXT, EVT_VIRTUAL_PREV, EVT_VIRTUAL_ENTER, EVT_VIRTUAL_EXIT = 1, 2, 3, 4
      EVT_TOUCH_TAP, EVT_TOUCH_SLIDE = 5, 6
      local t = 0
      getTime = function() t = t + 5 return t end
      loadScript = function(p, m) return loadfile(ROOTDIR .. p) end
      lcd = { RGB = function(r, g, b) return r * 65536 + g * 256 + b end, clear = function() end,
              drawFilledRectangle = function() end, drawRectangle = function() end, drawText = function() end,
              sizeText = function(s, f) return #s * 9, 20 end }
      io = io or {}
    ''')
    G.sportTelemetryPush = lambda phys, prim, app, val: (R.ap.uplink(int(app), int(val) & 0xFFFFFFFF), True)[1]
    queue = []
    def pop():
        if not queue: return None
        return queue.pop(0)
    G.sportTelemetryPop = L.eval('function(pop) return function() local f = pop() if f == nil then return nil end '
                                 'return f[1], f[2], f[3], f[4] end end')(lambda: (L.table_from(queue.pop(0)) if queue else None))
    tool = L.eval('dofile')(root + '/SCRIPTS/TOOLS/APParam.lua')
    tool.init()
    events = {i: v for i, v in enumerate([0] * 400)}
    events[200] = 3  # ENTER on the first tab: FLIGHT
    got = None
    for i in range(500):
        R.t += 0.05
        if int(R.t * 10) % 5 == 0: queue.append([0x1B, 0x10, 0x5001, 0])
        while R.ap.down:
            a, v = R.ap.down.pop(0); queue.append([0x1B, 0x32, a, v])
        r = tool.run(events.get(i, 0), None)
        check(r == 0, 'run returns 0')
    tool.run(4, None); tool.run(4, None)
    check(tool.run(4, None) == 1, 'EXIT on tab list leaves the tool')


def edgetx_shared_queue(entry_path=None, label='built'):
    """EdgeTX 2.12.5's ONE Lua telemetry byte queue (api_general.cpp): sportTelemetryPop takes 8 raw
    bytes, crossfireTelemetryPop a length-prefixed frame. ELRS model (getModule Type 5), Auto."""
    print('edgetx shared queue (%s)' % label)
    R = Rig(52, 'crsf')
    L = R.L
    G = L.globals()
    root = os.path.join(DIST, 'EdgeTX').replace(os.sep, '/')
    G.ROOTDIR = root
    L.execute('''
      LCD_W, LCD_H = 480, 272
      SMLSIZE, MIDSIZE, RIGHT, CENTER, INVERS, FORCE = 0x100, 0x200, 0x400, 0x800, 2, 1
      EVT_VIRTUAL_NEXT, EVT_VIRTUAL_PREV, EVT_VIRTUAL_ENTER, EVT_VIRTUAL_EXIT = 1, 2, 3, 4
      local t = 0
      getTime = function() t = t + 5 return t end
      loadScript = function(p, m) return loadfile(ROOTDIR .. p) end
      lcd = { RGB = function(r, g, b) return r * 65536 + g * 256 + b end, clear = function() end,
              drawFilledRectangle = function() end, drawRectangle = function() end, drawText = function() end,
              sizeText = function(s, f) return #s * 9, 20 end }
      model = { getModule = function(i) if i == 1 then return { Type = 5 } end return { Type = 0 } end }
      Q = {}
      function qpush(len, cmd, data)   -- what EdgeTX's CRSF receive puts in the queue
        Q[#Q + 1] = len Q[#Q + 1] = cmd
        for i = 1, #data do Q[#Q + 1] = data[i] end
      end
      sportTelemetryPop = function()
        if #Q < 8 then return nil end
        local b = {}
        for i = 1, 8 do b[i] = table.remove(Q, 1) end
        return b[1], b[2], b[3] + b[4] * 256, b[5] + b[6] * 256 + b[7] * 65536 + b[8] * 16777216
      end
      crossfireTelemetryPop = function()
        if #Q == 0 or #Q < Q[1] then return nil end
        local len = table.remove(Q, 1)
        local cmd = table.remove(Q, 1)
        local d = {}
        for i = 1, len - 2 do d[i] = table.remove(Q, 1) end
        return cmd, d
      end
    ''')
    def push(cmd, data):
        cmd = int(cmd); d = [int(data[i + 1]) for i in range(len(data))]
        if cmd == 0x80: R.lite_crsf(d[4], bytes(d[5:]))
        return True
    G.crossfireTelemetryPush = push
    G.sportTelemetryPush = lambda *a: True
    tool = L.eval('dofile')(entry_path or (root + '/SCRIPTS/TOOLS/APParam.lua'))
    def tick(ev=0):
        R.t += 0.05
        frames = []
        if int(R.t * 20) % 4 == 0: frames.append((0x1E, [1, 2, 3, 4, 5, 6]))          # attitude
        if int(R.t * 10) % 5 == 0: frames.append((0x80, [0xF0, 0x01, 0x50, 0, 0, 0, 0]))
        frames += getattr(R, 'crsf_down', []); R.crsf_down = []
        for cmd, d in frames: G.qpush(len(d) + 2, cmd, L.table_from(d))
        return tool.run(ev, None)
    G.APPARAM_TEST = L.table()
    tool.init()
    C = G.APPARAM_TEST.C
    for _ in range(100): tick()
    ready = C.blocked is None and C.link == 'CRSF'
    tabs = [C.tabs()[i + 1] for i in range(len(C.tabs()))]
    blocked = 0
    if 'TECS' in tabs:
        for _ in range(tabs.index('TECS')): tick(1)
        tick(3)
    for _ in range(600):
        tick()
        if C.blocked is not None: blocked += 1
    rows = C.rows()
    got, n = C.progress()
    print('   link %s, blocked updates %d, TECS %s/%s, %s' % (C.link, blocked, got, n, C.diagText()))
    return ready and blocked == 0 and n > 0 and got == n


def ethos_entry():
    print('ethos entry lua54')
    R = Rig(54, 'crsf')
    L = R.L
    G = L.globals()
    root = os.path.join(DIST, 'Ethos', 'scripts', 'apparam').replace('\\', '/')
    G.ROOTDIR = root
    pushed = []
    G.pypush = lambda cmd, data: (pushed.append(int(cmd)), R.lite_crsf(int(data[5]), bytes([int(data[i + 1]) for i in range(5, len(data))])) if int(cmd) == 0x80 else None, True)[2]
    G.pypop = lambda: None
    L.execute('''
      FONT_XS, FONT_S, FONT_STD, FONT_L = 1, 2, 3, 4
      LEFT, RIGHT, CENTERED = 0, 1, 2
      EVT_KEY, EVT_TOUCH = 1, 2
      KEY_ROTARY_RIGHT, KEY_ROTARY_LEFT, KEY_ENTER_BREAK, KEY_EXIT_BREAK = 10, 11, 12, 13
      TOUCH_START, TOUCH_MOVE, TOUCH_END = 20, 21, 22
      local realLoadfile = loadfile
      loadfile = function(p) if string.sub(p, 1, 1) ~= "/" then p = ROOTDIR .. "/" .. p end return realLoadfile(p) end
      local clk = 0
      os.clock = function() clk = clk + 0.05 return clk end
      lcd = { loadMask = function(p) return p end, getWindowSize = function() return 800, 480 end,
              RGB = function(r, g, b) return r * 65536 + g * 256 + b end, color = function() end, font = function() end,
              drawFilledRectangle = function() end, drawRectangle = function() end, drawText = function() end,
              getTextSize = function(s) return #s * 12, 24 end, invalidate = function() end }
      EXITED = false
      system = { registerSystemTool = function(t) TOOL = t end, exit = function() EXITED = true end }
      DOWN = {}
      local function sensor()
        return { pushFrame = function(self, cmd, data) return pypush(cmd, data) end,
                 popFrame = function(self, lo, hi)
                   local f = table.remove(DOWN, 1)
                   if not f then return nil end
                   return f[1], f[2]
                 end }
      end
      local s1 = sensor()
      crsf = { getSensor = function() if not S1 then S1 = s1 return s1 end return { popFrame = function() return nil end, pushFrame = function() return true end } end }
    ''')
    G.APPARAM_TEST = L.table()
    L.eval('dofile')(root + '/main.lua').init()
    T = G.TOOL
    check(T is not None and T.name == 'AP Param Editor' and T.icon == 'icon.png', 'registered with icon')
    w = T.create()
    for _ in range(300):
        R.t += 0.05
        if int(R.t * 10) % 5 == 0:
            G.DOWN[len(G.DOWN) + 1] = L.table_from([0x80, L.table_from([0xF0, 0x01, 0x50, 0, 0, 0, 0])])
        G.DOWN[len(G.DOWN) + 1] = L.table_from([0x1E, L.table_from([1, 2, 3, 4, 5, 6])])   # attitude (native)
        for cmd, d in getattr(R, 'crsf_down', []):
            G.DOWN[len(G.DOWN) + 1] = L.table_from([cmd, L.table_from(d)])
        R.crsf_down = []
        T.wakeup(w)
        T.paint(w)
    T.event(w, 1, 12, 0, 0)    # ENTER: FLIGHT tab
    for _ in range(200):
        R.t += 0.05
        if int(R.t * 10) % 5 == 0:
            G.DOWN[len(G.DOWN) + 1] = L.table_from([0x80, L.table_from([0xF0, 0x01, 0x50, 0, 0, 0, 0])])
        G.DOWN[len(G.DOWN) + 1] = L.table_from([0x1E, L.table_from([1, 2, 3, 4, 5, 6])])   # attitude (native)
        for cmd, d in getattr(R, 'crsf_down', []):
            G.DOWN[len(G.DOWN) + 1] = L.table_from([cmd, L.table_from(d)])
        R.crsf_down = []
        T.wakeup(w)
        T.paint(w)
    check(0x80 in pushed and 0xAA in pushed, 'pushed MAVlite and heartbeat: %s' % sorted(set(pushed)))
    EC = G.APPARAM_TEST.C
    got, n = EC.progress()
    print('   link %s, %s/%s FLIGHT rows answered, %s' % (EC.link, got, n, EC.diagText()))
    check(EC.link == 'CRSF' and EC.blocked is None, 'Ethos on CRSF, not blocked')
    check(EC.diag.crNat > 0, 'native frames reach the engine through the one CRSF reader')
    check(n > 0 and got == n, 'Ethos read the open tab')
    T.event(w, 1, 13, 0, 0)
    check(not G.EXITED, 'RTN from rows only goes back')
    T.event(w, 1, 13, 0, 0)
    check(G.EXITED, 'RTN on tab list exits')


for ver in (52, 54):
    for kind, (w, h) in (('crsf', (480, 272)), ('sport', (800, 480)), ('mlrs', (320, 480)), ('crsf', (480, 320))):
        if ver == 54 or (w, h) == (480, 272):
            colour(ver, kind, w, h)
    mono(ver, 128)
mono(52, 212)
edgetx_entry(52)
check(edgetx_shared_queue(), 'EdgeTX shared queue, ELRS model, Auto: reads TECS, never blocked')
ethos_entry()
f = test_core.fails
print('FAILURES: %d' % f if f else 'ALL PASS')
sys.exit(1 if f else 0)
