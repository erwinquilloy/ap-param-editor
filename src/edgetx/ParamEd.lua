local toolName = "TNS|OmniDeck Param Editor|TNE"
--[[
  OmniDeck Param Editor for EdgeTX (SYS > Tools): read and change flight controller settings from the radio.
    ArduPilot   S.Port / F.Port passthrough (SERIALn_PROTOCOL 10); ELRS / Crossfire with ArduPlane
                Stratos v2.2.1+ (MAVlite over CRSF); mLRS Tx Ser Port = cbridge (MAVLink)
    INAV 9.1+, Betaflight 4.5 - 2026.6  MSP over ELRS / Crossfire, S.Port / F.Port or mLRS
  The code is in /SCRIPTS/PARAMED/: pick.lua (which engine), core.lua (ArduPilot), mspcore.lua (INAV,
  Betaflight), list.lua / ilist.lua / bflist.lua (the tabs), ui_color.lua
  (colour screens, touch) or ui_bw.lua (128x64 / 212x64). custom.txt there adds a CUSTOM tab;
  settings.txt keeps the telemetry source.
  EdgeTX pauses telemetry scripts while a tool runs, so this one reads the telemetry queues itself.
]]

-- Some EdgeTX builds run without Lua's table library (TX12 MK2, EdgeTX 2.12, 2026-11-13): the few
-- calls used here, in plain Lua, only where the library is missing
if rawget(_G, "table") == nil then
  local T = {}
  function T.insert(t, a, b)
    if b == nil then t[#t + 1] = a return end
    for i = #t, a, -1 do t[i + 1] = t[i] end
    t[a] = b
  end
  function T.remove(t, pos)
    local n = #t
    if n == 0 then return nil end
    pos = pos or n
    local v = t[pos]
    for i = pos, n - 1 do t[i] = t[i + 1] end
    t[n] = nil
    return v
  end
  local function unpack(t, i, j)
    i, j = i or 1, j or #t
    if i > j then return end
    return t[i], unpack(t, i + 1, j)
  end
  T.unpack = unpack
  table = T
end

local ROOT = "/SCRIPTS/PARAMED/"
-- the title bar: the full name where it fits; a 128-pixel screen (TX12) has room for 12 letters
-- beside the memory figure, so it keeps the short one
local TITLE = (LCD_W or 480) > 160 and "OmniDeck Param Editor" or "Param Editor"
local VERSION = "0.2.1"
local C, U, err = nil, nil, nil
local colour = false
local base = getTime()

local function load(name)
  local chunk, e = loadScript(ROOT .. name, "bt")
  if not chunk then error(name .. ": " .. tostring(e)) end
  return chunk()
end

local IO = {
  clock = function() return (getTime() - base) / 100 end,
  push = function(f)
    if f.sp then
      if not sportTelemetryPush then return false end
      return sportTelemetryPush(0x0D, 0x30, f.appId, f.value)
    end
    if not crossfireTelemetryPush then return false end
    return crossfireTelemetryPush(f.cmd, f.data)
  end,
  version = VERSION,
  loadCore = function() return load("core.lua") end,
  loadMsp = function() return load("mspcore.lua") end,
  loadList = function() return load("list.lua") end,
  loadInav = function() return load("ilist.lua") end,
  loadBf = function() return load("bflist.lua") end,
  customPath = ROOT .. "custom.txt",
  inavCustomPath = ROOT .. "custom_inav.txt",
  settingsPath = ROOT .. "settings.txt",
  readFile = function(path)
    local f = io.open(path, "r")
    if not f then return nil end
    local s = io.read(f, 4096)
    io.close(f)
    return s
  end,
  writeFile = function(path, s)
    local f = io.open(path, "w")
    if not f then return end
    io.write(f, s)
    io.close(f)
  end,
  -- the mLRS bridge stream is read only while this update has used under 40 % of the per-call limit
  busy = getUsage and function() return getUsage() >= 40 end or nil,
  crsfOk = crossfireTelemetryPush ~= nil,
}

-- The first run compiles each .lua and saves a .luac beside it. Compiled while the others are in
-- memory (with their debug info, as just compiled), the last one ran out on a TX12 MK2 (EdgeTX
-- 2.12.2, 2026-11-19: "list.lua: not enough memory"; the second run, from .luac, used 83 of
-- ~127 KB). So each file is compiled alone first and dropped; the real loads then read the
-- smaller .luac. Once the .luac files are there this only reads them.
local function precompile(files)
  for i = 1, #files do
    pcall(loadScript, ROOT .. files[i], "bt")
    collectgarbage()
    collectgarbage()
  end
end

local function init()
  local ok, e = pcall(function()
    colour = lcd.RGB ~= nil and (LCD_W or 0) >= 320
    local ui = colour and "ui_color.lua" or "ui_bw.lua"
    precompile({ "pick.lua", "core.lua", "list.lua", "mspcore.lua", "ilist.lua", "bflist.lua", ui })
    C = load("pick.lua")(IO)
    if type(PARAMED_TEST) == "table" then PARAMED_TEST.C = C end   -- off-radio tests only
    U = load(ui)(C, TITLE)
  end)
  if not ok then err = tostring(e) end
  collectgarbage()
end

-- EdgeTX keeps ONE byte queue for Lua telemetry: sportTelemetryPop takes 8 raw bytes whatever they
-- are, crossfireTelemetryPop a length-prefixed frame (api_general.cpp, 2.12.5). Calling both eats
-- the other's bytes: on ELRS the S.Port pop turned CRSF frames into fake S.Port packets and broke
-- the CRSF stream (TX16S + ELRS, 2026-11-19: "S.Port", list, then no telemetry). So only one is
-- called: the one the chosen telemetry source uses, or on Auto the one the model's module speaks
-- (model.getModule Type, MODULE_TYPE_* in modules_constants.h: 5 = CROSSFIRE (ELRS, TBS, mLRS);
-- 2 XJT, 3 ISRM, 7-10 and 12 R9M, 14 XJT Lite = FrSky S.Port; 6 = multi-protocol, S.Port).
local SPORT_TYPES = { [2] = true, [3] = true, [6] = true, [7] = true, [8] = true, [9] = true, [10] = true,
                      [12] = true, [14] = true }
local modKind, modAt = nil, -10
local function moduleKind()
  local t = IO.clock()
  if modKind and t - modAt < 5 then return modKind end
  modAt = t
  local kind = nil
  if model and model.getModule then
    for i = 0, 1 do
      local ok, m = pcall(model.getModule, i)
      local ty = ok and m and m.Type
      if ty == 5 then kind = "crsf" break end
      if ty and SPORT_TYPES[ty] and not kind then kind = "sport" end
    end
  end
  modKind = kind or (crossfireTelemetryPop and "crsf" or "sport")
  return modKind
end
local function queueKind()
  local src = C.source
  if src == "SPORT" then return "sport" end
  if src == "CRSF" or src == "MAVLINK" then return "crsf" end
  return moduleKind()
end

local function drain()
  local kind = queueKind()
  if kind == "sport" and sportTelemetryPop then
    for _ = 1, 40 do
      local physId, primId, appId, value = sportTelemetryPop()
      if physId == nil then break end
      C.sport(primId, appId, value)
    end
  elseif kind == "crsf" and crossfireTelemetryPop then
    for _ = 1, 40 do
      local cmd, data = crossfireTelemetryPop()
      if cmd == nil then break end
      C.crsf(cmd, data)
    end
  end
end

local function keyOf(e)
  if e == 0 or e == nil then return nil end
  if e == EVT_VIRTUAL_NEXT or e == EVT_VIRTUAL_NEXT_REPT or e == EVT_VIRTUAL_INC or e == EVT_VIRTUAL_INC_REPT
     or e == EVT_ROT_RIGHT or e == EVT_PLUS_FIRST or e == EVT_PLUS_REPT then return "next" end
  if e == EVT_VIRTUAL_PREV or e == EVT_VIRTUAL_PREV_REPT or e == EVT_VIRTUAL_DEC or e == EVT_VIRTUAL_DEC_REPT
     or e == EVT_ROT_LEFT or e == EVT_MINUS_FIRST or e == EVT_MINUS_REPT then return "prev" end
  if e == EVT_VIRTUAL_ENTER then return "enter" end
  if e == EVT_VIRTUAL_EXIT then return "back" end
  return nil
end

-- drawing adapter for ui_color.lua
local FONT = { s = SMLSIZE, m = 0, l = MIDSIZE }
local FH = {}
local G = nil
local function gfx()
  local g = { W = LCD_W, H = LCD_H }
  function g.rgb(r, gr, b) return lcd.RGB(r, gr, b) end
  function g.rect(x, y, w, h, c) lcd.drawFilledRectangle(x, y, w, h, c) end
  function g.frame(x, y, w, h, c) lcd.drawRectangle(x, y, w, h, c) end
  function g.text(x, y, s, font, c, align)
    local fl = FONT[font] or 0
    if align == "r" then fl = fl + RIGHT elseif align == "c" then fl = fl + CENTER end
    lcd.drawText(x, y, s, fl + (c or 0))
  end
  function g.textW(s, font)
    if lcd.sizeText then return (lcd.sizeText(s, FONT[font] or 0)) end
    return #s * ((font == "s" and 7) or (font == "l" and 14) or 9)
  end
  function g.fontH(font)
    if not FH[font] then
      local h = nil
      if lcd.sizeText then local _ _, h = lcd.sizeText("Ag", FONT[font] or 0) end
      FH[font] = h or ((font == "s" and 14) or (font == "l" and 28) or 20)
    end
    return FH[font]
  end
  return g
end

local function run(event, touchState)
  if err then
    lcd.clear()
    lcd.drawText(1, 1, TITLE, 0)
    for i = 0, 5 do lcd.drawText(1, 12 + i * 9, string.sub(err, i * 30 + 1, i * 30 + 30), SMLSIZE) end
    return (event == EVT_VIRTUAL_EXIT) and 1 or 0
  end
  drain()
  C.update()
  local k = keyOf(event)
  if k and U.key(k) == false then return 1 end
  if colour and touchState then
    if event == EVT_TOUCH_TAP then
      if U.tap(touchState.x, touchState.y) == false then return 1 end
    elseif event == EVT_TOUCH_SLIDE and touchState.slideY then
      U.slide(touchState.slideY)
    end
  end
  if colour then
    if not G then G = gfx() end
    U.paint(G)
  else
    U.paint()
  end
  return 0
end

return { init = init, run = run }
