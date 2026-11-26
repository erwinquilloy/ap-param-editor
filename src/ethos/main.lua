--[[
  OmniDeck Param Editor for FrSky Ethos (System > Tools): read and change flight controller settings.
    ArduPilot   S.Port / F.Port passthrough (SERIALn_PROTOCOL 10); ELRS / Crossfire with ArduPlane
                Stratos v2.2.1+ (MAVlite over CRSF); mLRS Tx Ser Port = cbridge (MAVLink)
    INAV 9.1+, Betaflight 4.5 - 2026.6  MSP over ELRS / Crossfire, S.Port / F.Port or mLRS
  Files in /scripts/parameditor/: pick.lua (which engine), core.lua (ArduPilot), mspcore.lua (INAV,
  Betaflight), list.lua / ilist.lua / bflist.lua (tabs), ui_color.lua (screen), custom.txt and
  custom_inav.txt (CUSTOM tabs), settings.txt (telemetry source).

  Ethos hands every reader its own copy of the frames (no shared queue as on EdgeTX), and a pop
  with a type range DROPS the frames outside it for that reader (X20S + mLRS 2026-11-14). So the
  one CRSF reader here pops with no range: passthrough / MAVlite (0x7F-0x80), the FC's native
  frames (they keep a slow ELRS link counted as alive) and the mLRS bridge (0x82, or 0xAA) all come through.
]]
local DIR = "/scripts/parameditor/"
local NAME = "OmniDeck Param Editor"
local VERSION = "0.2.1"

local function mod(name) return assert(loadfile(name))() end

local IO = {
  clock = os.clock,
  version = VERSION,
  loadCore = function() return mod("core.lua") end,
  loadMsp = function() return mod("mspcore.lua") end,
  loadList = function() return mod("list.lua") end,
  loadInav = function() return mod("ilist.lua") end,
  loadBf = function() return mod("bflist.lua") end,
  customPath = DIR .. "custom.txt",
  inavCustomPath = DIR .. "custom_inav.txt",
  settingsPath = DIR .. "settings.txt",
  -- line by line, as OmniDeck reads its files on Ethos
  readFile = function(path)
    local ok, f = pcall(io.open, path, "r")
    if not ok or not f then return nil end
    local lines = {}
    for _ = 1, 200 do
      local ok2, l = pcall(f.read, f, "*l")
      if not ok2 or l == nil then break end
      lines[#lines + 1] = l
    end
    pcall(f.close, f)
    return table.concat(lines, "\n")
  end,
  writeFile = function(path, s)
    local ok, f = pcall(io.open, path, "w")
    if not ok or not f then return end
    f:write(s)
    f:close()
  end,
}

-- ---------------------------------------------------------------- readers
local sp, spMav, cr = nil, nil, nil
local function sensors()
  if sport and sport.getSensor and not sp then
    local ok, s = pcall(sport.getSensor, { appIdStart = 0x5000, appIdEnd = 0x50FF })
    sp = ok and s or false
    ok, s = pcall(sport.getSensor, { primId = 0x32 })
    spMav = ok and s or false
  end
  if crsf and crsf.getSensor and not cr then
    local ok, s = pcall(crsf.getSensor)
    cr = ok and s or false
  end
end

IO.push = function(f)
  if f.sp then
    local s = spMav or sp
    if not s then return false end
    return s:pushFrame({ physId = 0x0D, primId = 0x30, appId = f.appId, value = f.value })
  end
  if not cr then return false end
  return cr:pushFrame(f.cmd, f.data)
end

local C, U = nil, nil
local function drain()
  sensors()
  -- each reader passes only its own kind: a MAVlite reply's appId (sequence + a message byte) can
  -- fall inside 0x5000-0x50FF too, and the same frame twice would break the reassembly
  for i, s in ipairs({ sp or false, spMav or false }) do
    if s then
      local want = i == 1 and 0x10 or 0x32
      for _ = 1, 24 do
        local ok, fr = pcall(s.popFrame, s)
        if not ok or not fr then break end
        local prim = fr:primId()
        if prim == want then C.sport(prim, fr:appId(), fr:value()) end
      end
    end
  end
  if cr then
    for _ = 1, 48 do
      local ok, cmd, data = pcall(cr.popFrame, cr)
      if not ok or cmd == nil then break end
      C.crsf(cmd, data)
    end
  end
end

-- ---------------------------------------------------------------- drawing adapter
local G = nil
local FONTS = nil
local function gfx()
  local w, h = lcd.getWindowSize()
  local g = { W = w, H = h }
  if h >= 400 then FONTS = { s = FONT_S, m = FONT_STD, l = FONT_L }
  elseif h >= 300 then FONTS = { s = FONT_XS, m = FONT_STD, l = FONT_L }
  else FONTS = { s = FONT_XS, m = FONT_S, l = FONT_STD } end
  local fh = {}
  function g.rgb(r, gr, b) return lcd.RGB(r, gr, b) end
  function g.rect(x, y, ww, hh, c) lcd.color(c) lcd.drawFilledRectangle(x, y, ww, hh) end
  function g.frame(x, y, ww, hh, c) lcd.color(c) lcd.drawRectangle(x, y, ww, hh) end
  function g.text(x, y, s, font, c, align)
    lcd.font(FONTS[font] or FONT_STD)
    lcd.color(c)
    lcd.drawText(x, y, s, (align == "r" and RIGHT) or (align == "c" and CENTERED) or LEFT)
  end
  function g.textW(s, font)
    lcd.font(FONTS[font] or FONT_STD)
    return (lcd.getTextSize(s))
  end
  function g.fontH(font)
    if not fh[font] then
      lcd.font(FONTS[font] or FONT_STD)
      local _, th = lcd.getTextSize("Ag")
      fh[font] = th
    end
    return fh[font]
  end
  return g
end

-- ---------------------------------------------------------------- tool
local lastPaint = 0
local function create()
  C = mod("pick.lua")(IO)
  if type(PARAMED_TEST) == "table" then PARAMED_TEST.C = C end   -- off-radio tests only
  U = mod("ui_color.lua")(C, NAME)
  G = nil
  return {}
end

local function wakeup()
  if not C then return end
  drain()
  C.update()
  local t = os.clock()
  if t - lastPaint >= 0.2 then lastPaint = t lcd.invalidate() end
end

local function paint()
  if not U then return end
  if not G then G = gfx() end
  U.paint(G)
end

local function event(w, category, value, x, y)
  if not U then return false end
  local handled = true
  if category == EVT_KEY then
    local k = (value == KEY_ROTARY_RIGHT and "next") or (value == KEY_ROTARY_LEFT and "prev")
      or (value == KEY_ENTER_BREAK and "enter") or (value == KEY_EXIT_BREAK and "back")
      or (KEY_RTN_BREAK and value == KEY_RTN_BREAK and "back") or nil
    if not k then return false end
    handled = U.key(k)
  elseif category == EVT_TOUCH then
    -- a drag scrolls; a touch that moved more than 12 px is not a tap
    if TOUCH_START and value == TOUCH_START then w.ty, w.moved = y, 0
    elseif TOUCH_MOVE and value == TOUCH_MOVE and w.ty then
      U.slide(y - w.ty)
      w.moved = (w.moved or 0) + math.abs(y - w.ty)
      w.ty = y
    elseif value == TOUCH_END then
      if (w.moved or 0) <= 12 then handled = U.tap(x, y) end
      w.ty, w.moved = nil, 0
    end
  else
    return false
  end
  lcd.invalidate()
  if handled == false then
    -- RTN on the tab list or on a screen the editor can't work from: close the tool
    if system.exit then system.exit() end
    return true
  end
  return true
end

local function close() C, U, G = nil, nil, nil end

local icon = lcd.loadMask("icon.png")

local function init()
  system.registerSystemTool({ name = NAME, icon = icon, create = create, wakeup = wakeup, paint = paint,
                              event = event, close = close })
end

return { init = init }
