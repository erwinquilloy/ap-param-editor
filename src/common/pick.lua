--[[
  Param Editor: which engine talks to the flight controller (EdgeTX and Ethos share this file).
    core.lua      ArduPilot (MAVlite over S.Port / CRSF, MAVLink through the mLRS bridge)
    mspcore.lua   INAV 9.1+ and Betaflight 4.5 / 2025.12 (MSP over CRSF, S.Port, the mLRS bridge)
  Only one is in memory. The tool starts with ArduPilot's; when telemetry comes in but nothing
  from ArduPilot (no passthrough frame, no ArduPilot HEARTBEAT) for a few seconds, the MSP one asks
  the flight controller what it is. ArduPilot heard again, or no MSP answer: back to ArduPilot's
  (and MSP is not asked again for 15 s).

  The screens hold the table returned here: every call and field goes to the engine in use, so
  they need no change when the engine is swapped (Air edits and the open tab start over).
  io: as for core.lua, plus loadCore(), loadMsp() (each returns that engine's maker).
]]
return function(io)
  local E = {}
  local cur, kind = nil, nil
  local firstAt, apAt, backoff = nil, -100, -100
  local AP_WAIT = 5   -- s of telemetry without ArduPilot before MSP is asked

  local function use(k)
    cur = nil
    collectgarbage()
    collectgarbage()
    cur = (k == "MSP" and io.loadMsp() or io.loadCore())(io)
    kind = k
    collectgarbage()
  end
  function E.kind() return kind end
  -- the flight controller, for the tab list
  function E.fcText()
    if kind == "MSP" then return cur.fcText() end
    return cur.fc or "-"
  end

  -- ArduPilot's own frames: passthrough on CRSF (0x80 / 0x7F F0-F4) and on S.Port (0x5000-0x50FF)
  function E.sport(primId, appId, value)
    local now = io.clock()
    firstAt = firstAt or now
    if primId == 0x10 and appId and appId >= 0x5000 and appId <= 0x50FF then apAt = now end
    cur.sport(primId, appId, value)
  end
  function E.crsf(cmd, d)
    local now = io.clock()
    firstAt = firstAt or now
    if d and (cmd == 0x80 or cmd == 0x7F) and d[1] and d[1] >= 0xF0 and d[1] <= 0xF4 then apAt = now end
    cur.crsf(cmd, d)
  end

  function E.update()
    local now = io.clock()
    if kind == "AP" then
      if cur.fc == nil and firstAt and now - firstAt >= AP_WAIT and now - apAt >= AP_WAIT and now >= backoff then
        use("MSP")
      end
    elseif now - apAt < 2 then
      use("AP")
      firstAt = now
    elseif cur.noMsp() or (cur._in and cur._in.state == "notmsp") then
      use("AP")
      firstAt, backoff = now, now + 15
    end
    cur.update()
  end

  -- last: E's own functions are set above; any other field read or set goes to the engine
  setmetatable(E, {
    __index = function(_, k) return cur[k] end,
    __newindex = function(_, k, v) cur[k] = v end,
  })
  use("AP")
  return E
end
