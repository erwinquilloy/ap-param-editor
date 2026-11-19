local toolName = "TNS|AP Param Editor|TNE"
--[[
  AP Param Editor for EdgeTX (SYS > Tools): read and change ArduPilot parameters from the radio.
    S.Port / F.Port   any ArduPilot with FrSky passthrough (SERIALn_PROTOCOL 10)
    ELRS / Crossfire  ArduPlane Stratos v2.2.1+ (MAVlite over CRSF)
    mLRS              Tx Ser Port = cbridge, any ArduPilot (MAVLink)
  The code is in /SCRIPTS/APPARAM/: core.lua (links, requests), list.lua (the tabs), ui_color.lua
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

local ROOT = "/SCRIPTS/APPARAM/"
local TITLE = "AP Param Editor"
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
  loadList = function() return load("list.lua") end,
  customPath = ROOT .. "custom.txt",
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

local function init()
  local ok, e = pcall(function()
    C = load("core.lua")(IO)
    colour = lcd.RGB ~= nil and (LCD_W or 0) >= 320
    U = load(colour and "ui_color.lua" or "ui_bw.lua")(C, TITLE)
  end)
  if not ok then err = tostring(e) end
  collectgarbage()
end

local function drain()
  if sportTelemetryPop then
    for _ = 1, 40 do
      local physId, primId, appId, value = sportTelemetryPop()
      if physId == nil then break end
      C.sport(primId, appId, value)
    end
  end
  if crossfireTelemetryPop then
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
