--[[
  Param Editor, MSP engine for INAV and Betaflight (EdgeTX and Ethos share this file byte for byte).
  The same calls as core.lua (the ArduPilot engine), so the screens work with either; pick.lua loads
  this one only once a flight controller that is not ArduPilot is heard.

    INAV 9.1+        settings by their CLI name, MSPv2: MSP2_COMMON_SETTING_INFO 0x1007 (name\0) ->
                     name\0, pgn u16, type u8, section u8, mode u8, min i32, max u32, index u16,
                     profile u8, profile count u8, [lookup names\0 ...], value;
                     MSP2_COMMON_SET_SETTING 0x1004 (name\0, value); MSP2_COMMON_SETTING 0x1003
                     (name\0) -> value (the read-back after a save). INAV 9.0.x answers the setting
                     commands without name and value over the radio link (fixed in 9.1.0-RC1).
    Betaflight       4.5 (MSP API 1.46), 2025.12 (1.47) and 2026.6 (1.48): fixed blocks (bflist.lua), read, the one
                     field changed, written back up to the end of its group.
    Both             MSP_EEPROM_WRITE 250 stores; refused while armed, so a change made in flight is
                     stored once the flight controller says it is disarmed.

  Over the link (as OmniDeck's PARAMS page):
    ELRS / Crossfire  CRSF MSP frames: 0x7A to the FC (dest 0xC8, origin 0xEA), replies 0x7B; a chunk
                      is status (0x40 = v2 | 0x10 first | sequence 0-15; 0x80 error) + up to 57 bytes
    S.Port / F.Port   the same chunks, 6 bytes each: physId 0x0D primId 0x30 up, primId 0x32 down
    mLRS              raw MSPv2 frames ($X< ... crc8) in mLRS bridge frames 0x81 up, 0x82 down
  On CRSF each request goes both ways (native and bridge) until the first answer says which.

  Returns a maker: io -> C (io as for core.lua, plus loadInav() / loadBf() for the lists and
  inavCustomPath for the INAV CUSTOM tab).
]]
local floor = math.floor

local LN2 = math.log(2)
local function f2b(x)
  if x ~= x or x == 0 then return { 0, 0, 0, 0 } end
  local sign = 0
  if x < 0 then sign, x = 1, -x end
  local e = floor(math.log(x) / LN2)
  local m = x / 2 ^ e
  if m >= 2 then m, e = m / 2, e + 1 elseif m < 1 then m, e = m * 2, e - 1 end
  if e < -126 then return { 0, 0, 0, sign * 128 } end
  if e > 127 then e, m = 127, 2 - 2 ^ -23 end
  local frac = floor((m - 1) * 8388608 + 0.5)
  if frac >= 8388608 then frac, e = 0, e + 1 end
  local be = e + 127
  return { frac % 256, floor(frac / 256) % 256, (be % 2) * 128 + floor(frac / 65536), sign * 128 + floor(be / 2) }
end
local function b2f(b0, b1, b2, b3)
  local sign = b3 >= 128 and -1 or 1
  local be = (b3 % 128) * 2 + floor(b2 / 128)
  local frac = (b2 % 128) * 65536 + b1 * 256 + b0
  if be == 0 then return sign * frac * 2 ^ -149 end
  if be == 255 then return 0 end
  return sign * (1 + frac / 8388608) * 2 ^ (be - 127)
end

-- CRC-8/DVB-S2 (MSPv2), for the requests sent through the mLRS bridge
local crc8
if bit32 then
  local band, bxor, shl = bit32.band, bit32.bxor, bit32.lshift
  crc8 = function(c, b)
    c = bxor(c, b)
    for _ = 1, 8 do c = band(c, 0x80) ~= 0 and band(bxor(shl(c, 1), 0xD5), 0xFF) or band(shl(c, 1), 0xFF) end
    return c
  end
else
  crc8 = load("return function(c, b) c = c ~ b for _ = 1, 8 do "
    .. "c = (c & 0x80) ~= 0 and (((c << 1) ~ 0xD5) & 0xFF) or ((c << 1) & 0xFF) end return c end")()
end

-- a 32-bit S.Port value -> its 16-bit halves (as core.lua: Ethos may run Lua with 32-bit integers,
-- where "v % 4294967296" loses the low bits; & and >> where Lua has them)
local splitInt = math.type and load("return function(v) return v & 0xFFFF, (v >> 16) & 0xFFFF end")
splitInt = splitInt and splitInt() or nil
local function split32(v)
  if splitInt and math.type(v) == "integer" then return splitInt(v) end
  v = v % 4294967296
  return v % 65536, floor(v / 65536)
end

local MSP_API, MSP_FC_VARIANT, MSP_FC_VERSION, MSP_EEPROM_WRITE = 1, 2, 3, 250
local MSP2_INAV_STATUS, MSP_STATUS_EX = 0x2000, 150
local M_SETTING, M_SET_SETTING, M_SETTING_INFO = 0x1003, 0x1004, 0x1007
local VSIZE = { [0] = 1, [1] = 1, [2] = 2, [3] = 2, [4] = 4, [5] = 4 }
local MC_PLAT = { MULTIROTOR = true, TRICOPTER = true, HELICOPTER = true }
local STATUS_EVERY = 2

local function u16(b, o) return (b[o] or 0) + (b[o + 1] or 0) * 256 end
local function u32(b, o) return u16(b, o) + u16(b, o + 2) * 65536 end
local function decode(t, b, o)
  if t == 0 then return b[o] or 0 end
  if t == 1 then local v = b[o] or 0 return v >= 128 and v - 256 or v end
  if t == 2 then return u16(b, o) end
  if t == 3 then local v = u16(b, o) return v >= 32768 and v - 65536 or v end
  if t == 4 then return u32(b, o) end
  if t == 5 then return b2f(b[o] or 0, b[o + 1] or 0, b[o + 2] or 0, b[o + 3] or 0) end
  return nil
end
local function encode(t, raw, into)
  if t == 5 then local f = f2b(raw) for k = 1, 4 do into[#into + 1] = f[k] end return into end
  local n = VSIZE[t] or 1
  raw = floor(raw + 0.5)
  if raw < 0 then raw = raw + 2 ^ (8 * n) end
  for _ = 1, n do into[#into + 1] = raw % 256 raw = floor(raw / 256) end
  return into
end
local function zname(name)
  local t = {}
  for k = 1, #name do t[k] = string.byte(name, k) end
  t[#t + 1] = 0
  return t
end

return function(io)
  local C = { BITS = {}, skipped = function() return false end, skipCount = function() return 0 end,
              f2b = f2b, b2f = b2f }
  C.VERSION = io.version or "?"
  C.fc, C.link, C.armed, C.airEdits = nil, nil, false, false
  C.SOURCES = { "AUTO", "SPORT", "CRSF", "MAVLINK" }
  C.SOURCE_NAME = { AUTO = "Auto", SPORT = "S.Port / F.Port", CRSF = "ELRS / TBS (CRSF)", MAVLINK = "mLRS" }
  C.source = "AUTO"
  do
    local t = io.readFile and io.settingsPath and io.readFile(io.settingsPath)
    local v = t and string.match(t, "source=(%u+)")
    if v and C.SOURCE_NAME[v] then C.source = v end
  end
  C.msgs = {}
  C.stats = { tx = 0, rx = 0, bad = 0 }
  C.diag = { cr = 0, crPt = 0, crNat = 0, sp = 0, spPt = 0, env = 0, lost = 0 }
  local clock = io.clock
  local seenCr, seenSp, seenEnv, lastRx = -100, -100, -100, -100
  function C.alive() return clock() - lastRx < 3 end

  -- the flight controller: state "variant" -> "version" (INAV) / "bfapi" -> "version" (Betaflight)
  -- -> "ready"; "notmsp" (another FC), "old" (INAV before 9.0), "buggy" (INAV 9.0.x), "bfold"
  local IN = { state = "variant", variant = nil, ver = nil, meta = {}, profile = nil, rprof = nil,
               plat = nil, dirty = false, armed = nil, armedAt = nil, statusAt = -10, storeHold = 0 }
  C._in = IN
  local LIST = nil              -- the list module of this FC (ilist / bflist)
  local BF = nil                -- bflist.lua once a Betaflight 4.5 / 2025.12 / 2026.6 answered
  local blk = {}                -- Betaflight: the last reply of each block

  -- ---------------------------------------------------------------- send
  local out = {}
  local function clearOut() for i = #out, 1, -1 do out[i] = nil end end
  local link = nil              -- "CRSF" or "SPORT"
  local path = nil              -- CRSF: nil (both ways until an answer), "native", "bridge"
  local seq, last = 0, nil      -- request sequence; the last request (sent again on a timeout)

  local function bridgeSend(body)
    local f, c = { 36, 88, 60 }, 0   -- $X<
    for i = 1, #body do f[#f + 1] = body[i] c = crc8(c, body[i]) end
    f[#f + 1] = c
    local i = 1
    while i <= #f do
      local n = math.min(57, #f - i + 1)
      local e = { 0x66, 0, n }
      for k = 0, n - 1 do e[k + 4] = f[i + k] end
      out[#out + 1] = { cmd = 0x81, data = e }
      i = i + n
    end
  end
  local rxOn = false
  local function mspSend(cmd, payload)
    payload = payload or {}
    last = { cmd, payload }
    local body = { 0, cmd % 256, floor(cmd / 256), #payload % 256, floor(#payload / 256) }
    for i = 1, #payload do body[#body + 1] = payload[i] end
    rxOn = false
    if link == "CRSF" and path ~= "native" then
      bridgeSend(body)
      if path == "bridge" then return end
    end
    local per = link == "SPORT" and 5 or 57
    local i, first = 1, true
    repeat
      local c = { 0x40 + (first and 0x10 or 0) + seq }
      seq = (seq + 1) % 16
      for k = i, math.min(#body, i + per - 1) do c[#c + 1] = body[k] end
      i, first = i + per, false
      if link == "SPORT" then
        while #c < 6 do c[#c + 1] = 0 end
        out[#out + 1] = { sp = true, appId = c[1] + c[2] * 256, value = c[3] + c[4] * 256 + c[5] * 65536 + c[6] * 16777216 }
      else
        local d = { 0xC8, 0xEA }
        for k = 1, #c do d[k + 2] = c[k] end
        out[#out + 1] = { cmd = 0x7A, data = d }
      end
    until i > #body
  end
  local function again() if last then mspSend(last[1], last[2]) end end

  local function pushOne()
    local ok, r = pcall(io.push, out[1])
    if ok and r ~= false then
      table.remove(out, 1)
      C.stats.tx = C.stats.tx + 1
    end
  end

  -- ---------------------------------------------------------------- values
  local vals, vbs, txt = {}, {}, {}   -- name -> shown value (false: n/a), its raw-value key, text
  local tab, rows, rowsBy = nil, nil, {}
  local custom, GV = nil, {}
  C.customInfo = nil
  local edit, draft, base = nil, nil, nil
  C.phase, C.phaseKind = "", ""
  local function phase(t, k) C.phase, C.phaseKind = t, k or "" end
  local pend, saveReq = nil, nil
  local answered, silent, quietAt = false, 0, nil

  local function fmt(r, v)
    if v == nil then return "..." end
    if v == false then return "n/a" end
    if r[8] then
      local i, k = floor(v + 0.5), 0
      for o in string.gmatch(r[8], "[^|]+") do
        if k == i then return o end
        k = k + 1
      end
      return tostring(i)
    end
    local t = string.format("%." .. (r[7] or 0) .. "f", v)
    if r[3] and r[3] ~= "" then t = t .. " " .. r[3] end
    return t
  end
  C.fmt = fmt

  local function setVal(name, v, key)
    vals[name], vbs[name] = v, key
    local r = rowsBy[name]
    txt[name] = r and fmt(r, v) or nil
  end
  local function forgetPids()
    if not rows then return end
    for _, r in ipairs(rows) do if r.pid then vals[r[1]], txt[r[1]] = nil, nil end end
  end
  local function forgetAll() vals, vbs, txt = {}, {}, {} end

  -- a value arrived; same: for the read-back after a save, whether it is what was sent
  local function gotValue(name, v, key, same)
    setVal(name, v, key)
    if pend and pend.name == name then
      if pend.check then
        if same then
          IN.dirty = true
          phase((C.armed and "Set now, stored after the disarm") or "Saved, storing...", C.armed and "bad" or "ok")
        else phase("Not saved: FC has " .. fmt(rowsBy[name] or {}, v), "bad") end
        if edit and edit[1] == name then draft, base = v, v end
      elseif edit and edit[1] == name and draft == base then
        draft, base = v, v
      end
      pend = nil
    end
  end

  -- Betaflight: a row in list.lua's layout from bflist's (same labels / scale as it has now)
  local function bfRow(src)
    return { src[2], src[3], src[4], src[5], src[6], src[7], src[8], src[9], k = src.k, bf = src.bf,
             rmin = src.rmin, rmax = src.rmax, feat = src.feat, src = src }
  end
  local function bfRelabel()   -- rates_type read: the rate rows' labels, units and scale follow it
    if not rows then return end
    for _, r in ipairs(rows) do
      local s = r.src
      if s then r[2], r[3], r[4], r[5], r[6], r[7], r.k = s[3], s[4], s[5], s[6], s[7], s[8], s.k end
    end
  end

  -- ---------------------------------------------------------------- replies
  local function fcName()
    if IN.variant == "BTFL" then return "Betaflight" end
    if IN.variant == "INAV" then return "INAV" end
    return "flight controller"
  end

  local function bfReply(cmd, b, err)
    local name = pend and pend.name
    local r = name and rowsBy[name]
    if not r or not r.bf then return end
    local get = r.bf[1]
    if pend.bfset then
      if cmd ~= pend.bfset then return end
      if err then phase("Not saved: Betaflight refused the value", "bad") pend = nil return end
      pend = { name = name, check = true, checkRaw = pend.setRaw, tries = 0, at = nil }
      mspSend(get)
      return
    end
    if cmd ~= get then return end
    if err or #b < (BF.MINLEN[get] or 0) then
      if not err then C.stats.bad = C.stats.bad + 1 end
      if pend.check or pend.bfpre then phase("No answer: check in Betaflight Configurator", "bad") end
      pend = nil
      setVal(name, false, nil)
      return
    end
    blk[get] = b
    if get == 111 then
      local rt = b[23] or 0
      if rt ~= IN.rtype then IN.rtype = rt BF.ratesType(rt) bfRelabel() end
    end
    if pend.bfpre then
      local set = BF.SET[get]
      pend.bfpre, pend.bfset = nil, set
      mspSend(set, BF.patch(r.src, b, pend.setRaw))
      return
    end
    if rows then
      for _, x in ipairs(rows) do
        if x.bf and x.bf[1] == get and x[1] ~= name then
          if BF.present(x.src, b) then setVal(x[1], BF.raw(x.src, b) * x.k, BF.raw(x.src, b))
          else setVal(x[1], false, nil) end
        end
      end
    end
    if not BF.present(r.src, b) then
      if pend.check then phase("Not in this Betaflight build", "bad") end
      pend = nil
      setVal(name, false, nil)
      return
    end
    local raw = BF.raw(r.src, b)
    gotValue(name, raw * r.k, raw, pend.check and raw == pend.checkRaw)
  end

  local function inavReply(cmd, b, err)
    local name = pend.name
    if pend.set then
      if cmd ~= M_SET_SETTING then return end
      if err then phase("Not saved: INAV refused the value", "bad") pend = nil return end
      pend = { name = name, check = true, checkRaw = pend.setRaw, tries = 0 }
      mspSend(M_SETTING, zname(name))
      return
    end
    if pend.check then
      if cmd ~= M_SETTING then return end
      local mt, r = IN.meta[name], rowsBy[name]
      if err or not mt or #b < (VSIZE[mt.t] or 1) then
        if not err then C.stats.bad = C.stats.bad + 1 end
        pend = nil
        phase("No answer: check in INAV Configurator", "bad")
        return
      end
      local raw = decode(mt.t, b, 1)
      local same
      if mt.t == 5 then
        local x, y = f2b(raw), f2b(pend.checkRaw)
        same = x[1] == y[1] and x[2] == y[2] and x[3] == y[3] and x[4] == y[4]
      else same = raw == pend.checkRaw end
      if mt.look then gotValue(name, raw - mt.min, raw, same)
      else gotValue(name, raw * ((r and r.k) or 1), raw, same) end
      return
    end
    if cmd ~= M_SETTING_INFO then return end
    if err then
      pend = nil
      if name == "platform_type" then IN.plat = "?" return end
      setVal(name, false, nil)
      return
    end
    local i = 1
    while b[i] and b[i] ~= 0 do i = i + 1 end
    local bad = not b[i] or string.char((table.unpack or unpack)(b, 1, i - 1)) ~= name
    i = i + 1
    local t, sec, mode = (b[i + 2] or 0) % 8, (b[i + 3] or 0), (b[i + 4] or 0)
    local mn = u32(b, i + 5)
    if mn >= 2147483648 then mn = mn - 4294967296 end
    local mx = u32(b, i + 9)
    local prof = b[i + 15]
    i = i + 17
    local look = nil
    if mode == 0x40 then
      look = {}
      for _ = mn, mx do
        local s0 = i
        while b[i] and b[i] ~= 0 do i = i + 1 end
        look[#look + 1] = string.char((table.unpack or unpack)(b, s0, i - 1))
        i = i + 1
      end
    end
    if bad or not VSIZE[t] or #b < i + VSIZE[t] - 1 then
      C.stats.bad = C.stats.bad + 1
      pend = nil
      if name == "platform_type" then IN.plat = "?" return end
      setVal(name, false, nil)
      return
    end
    local raw = decode(t, b, i)
    if sec == 0x10 and prof and IN.profile ~= prof then   -- CONTROL_VALUE: the active control profile
      if IN.profile ~= nil then forgetPids() end
      IN.profile = prof
    end
    if name == "platform_type" then
      pend = nil
      local p = look and look[(raw or 0) - mn + 1] or tostring(raw)
      if IN.plat ~= nil and IN.plat ~= "?" and IN.plat ~= p then forgetPids() end
      IN.plat = p
      return
    end
    IN.meta[name] = { t = t, min = mn, max = mx, look = look }
    local r = rowsBy[name]
    if r then
      local k = r.k or 1
      if look then
        r[8], r[4], r[5], r[6], r[7] = table.concat(look, "|"), 0, #look - 1, 1, 0
      elseif t ~= 5 then
        r[4], r[5] = mn * k, mx * k
      end
    end
    if look then gotValue(name, (raw or 0) - mn, raw)
    else gotValue(name, (raw or 0) * ((r and r.k) or 1), raw) end
  end

  -- one complete reply from the flight controller
  local function reply(cmd, b, err)
    C.stats.rx = C.stats.rx + 1
    lastRx = clock()
    answered, silent, quietAt = true, 0, nil
    if not pend then return end
    local m = pend.m
    -- each housekeeping answer must be for its own command (a late reply to an earlier request can
    -- come in between); an error with command 0 is INAV before 9.0 refusing MSPv2
    if m == "variant" then
      if cmd ~= MSP_FC_VARIANT and not (err and cmd == 0) then return end
      pend = nil
      if err then IN.state = "old" return end
      IN.variant = string.char(b[1] or 63, b[2] or 63, b[3] or 63, b[4] or 63)
      if IN.variant == "INAV" then
        IN.state, C.fc = "version", "INAV"
      elseif IN.variant == "BTFL" then
        IN.state, C.fc = "bfapi", "Betaflight"
      else IN.state = "notmsp" end
      return
    end
    if m == "bfapi" then
      if cmd ~= MSP_API and not (err and cmd == 0) then return end
      pend = nil
      local maj, mnr = b[2] or 0, b[3] or 0
      IN.bfapi = string.format("%d.%d", maj, mnr)
      -- 1.46 = 4.5, 1.47 = 2025.12, 1.48 = 2026.6 (2026.6 only appends: RPM filter fields after
      -- byte 49 of MSP_FILTER_CONFIG, battery profile after MSP_STATUS_EX; msp.c 2026.6.2)
      if err or maj ~= 1 or mnr < 46 or mnr > 48 then IN.state = "bfold" return end
      if not BF then
        local ok, mod = pcall(io.loadBf)
        if not ok or type(mod) ~= "table" then IN.state, IN.bfErr = "bfold", tostring(mod) return end
        BF = mod
      end
      BF.forVersion(mnr)
      LIST, blk = BF, {}
      IN.state = "version"
      return
    end
    if m == "version" then
      if cmd ~= MSP_FC_VERSION and not (err and cmd == 0) then return end
      pend = nil
      if IN.variant == "BTFL" then
        local y = b[1] or 0
        if IN.bfapi ~= "1.46" then y = y + 2000 end   -- 2025.12 and later send the year since 2000
        IN.ver = err and "?" or string.format("%d.%d.%d", y, b[2] or 0, b[3] or 0)
        IN.state = "ready"
        return
      end
      if err then IN.state = "old" return end
      IN.ver = string.format("%d.%d.%d", b[1] or 0, b[2] or 0, b[3] or 0)
      local maj, mnr = b[1] or 0, b[2] or 0
      IN.state = (maj > 9 or (maj == 9 and mnr >= 1)) and "ready" or (maj == 9 and "buggy" or "old")
      if IN.state == "ready" then
        local ok, mod = pcall(io.loadInav)
        if ok and type(mod) == "table" then LIST = mod else IN.state = "old" end
      end
      return
    end
    if m == "status" then
      if IN.variant == "BTFL" then
        -- MSP_STATUS_EX: flight mode flags u32 at 7 (bit 0 = ARM), PID profile 11, rate profile 15
        if cmd ~= MSP_STATUS_EX then return end
        pend = nil
        if not err and #b >= 15 then
          IN.armed, IN.armedAt = (b[7] or 0) % 2 == 1, clock()
          local pp, rp = b[11], b[15]
          if (IN.profile ~= nil and IN.profile ~= pp) or (IN.rprof ~= nil and IN.rprof ~= rp) then
            forgetAll() blk = {}
          end
          IN.profile, IN.rprof = pp, rp
        end
      else
        -- MSP2_INAV_STATUS: armingFlags u32 at bytes 10-13, ARMED = bit 2
        if cmd ~= MSP2_INAV_STATUS then return end
        pend = nil
        if not err and #b >= 13 then IN.armed, IN.armedAt = floor(u32(b, 10) / 4) % 2 == 1, clock() end
      end
      C.armed = IN.armed == true
      return
    end
    if m == "store" then
      if cmd ~= MSP_EEPROM_WRITE then return end
      pend = nil
      if err then   -- refused while armed: written once the FC says it is disarmed
        IN.armed, IN.armedAt, IN.storeHold = true, clock(), clock() + STATUS_EVERY
        C.armed = true
        if edit then phase("Set now, stored after the disarm", "bad") end
        return
      end
      IN.dirty = false
      phase("Saved, verified and stored", "ok")
      return
    end
    if IN.variant == "BTFL" then bfReply(cmd, b, err) return end
    inavReply(cmd, b, err)
  end

  -- ---------------------------------------------------------------- receive
  -- one MSP chunk (native CRSF or S.Port): status byte, then the bytes
  local rxSize, rxCmd, rxErr, rxBuf, rxSeq = 0, 0, false, {}, 0
  local function chunk(c)
    if #c < 1 then return end
    local st = c[1]
    local start, sq = floor(st / 16) % 2 == 1, st % 16
    local i = 2
    if start then
      local ver = floor(st / 32) % 4
      rxErr, rxBuf, rxOn = st >= 128, {}, true
      if ver == 2 then rxCmd, rxSize, i = u16(c, 3), u16(c, 5), 7
      else rxCmd, rxSize, i = c[3] or 0, c[2] or 0, 4 end
    elseif not rxOn or ((rxSeq + 1) % 16) ~= sq then
      rxOn = false
      return
    end
    rxSeq = sq
    while i <= #c and #rxBuf < rxSize do rxBuf[#rxBuf + 1] = c[i] i = i + 1 end
    if #rxBuf >= rxSize then
      rxOn = false
      if path == nil and link == "CRSF" then path = "native" end
      if path ~= "bridge" then reply(rxCmd, rxBuf, rxErr) end
    end
  end

  -- raw MSP out of the mLRS bridge ($X> / $X! v2, $M> / $M! v1). No checksum test on the way in
  -- (mLRS checks every link frame; bit by bit it is too slow for EdgeTX); the length must match.
  local BR = { st = 0 }
  local function bridgeIn(data)   -- data: seq byte, size, bytes
    local e = 2 + (data[2] or 0)
    if e > #data then e = #data end
    local i = 3
    while i <= e do
      local x = data[i]
      local s = BR.st
      if s == 0 then if x == 36 then BR.st = 1 end i = i + 1
      elseif s == 1 then
        if x == 88 or x == 77 then BR.v2, BR.st = x == 88, 2 else BR.st = 0 end
        i = i + 1
      elseif s == 2 then
        if x == 62 or x == 33 then BR.err, BR.st, BR.hdr = x == 33, 3, {} else BR.st = 0 end
        i = i + 1
      elseif s == 3 then
        BR.hdr[#BR.hdr + 1] = x
        i = i + 1
        if #BR.hdr == (BR.v2 and 5 or 2) then
          BR.need = BR.v2 and (BR.hdr[4] + BR.hdr[5] * 256) or BR.hdr[1]
          BR.buf, BR.st = {}, BR.need > 0 and 4 or 5
          if BR.need > 1024 then BR.st = 0 end
        end
      elseif s == 4 then
        local b, n = BR.buf, BR.need
        while i <= e and #b < n do b[#b + 1] = data[i] i = i + 1 end
        if #b >= n then BR.st = 5 end
      else
        BR.st = 0   -- the checksum byte ends the frame
        i = i + 1
        if path == nil then path = "bridge" end
        if path == "bridge" then
          reply(BR.v2 and (BR.hdr[2] + BR.hdr[3] * 256) or BR.hdr[2], BR.buf, BR.err)
        end
      end
    end
  end

  local prevSp = nil
  function C.sport(primId, appId, value)
    if appId == nil or value == nil then return end
    C.diag.sp = C.diag.sp + 1
    seenSp = clock()
    if primId ~= 0x32 or link ~= "SPORT" then return end
    local lo, hi = split32(value)
    local c = { appId % 256, floor(appId / 256) % 256, lo % 256, floor(lo / 256), hi % 256, floor(hi / 256) }
    local key = table.concat(c, ",")
    if key == prevSp then return end   -- receivers can repeat a frame
    prevSp = key
    chunk(c)
  end
  function C.crsf(cmd, d)
    if not d then return end
    local D = C.diag
    D.cr = D.cr + 1
    seenCr = clock()
    if cmd == 0x7B then
      if link == "CRSF" and #d >= 3 then
        local c = {}
        for k = 3, #d do c[#c + 1] = d[k] end
        chunk(c)
      end
    elseif cmd == 0x82 and d[1] == 0x66 then
      D.env = D.env + 1
      seenEnv = clock()
      if link == "CRSF" then
        local t = {}
        for i = 2, #d do t[i - 1] = d[i] end
        bridgeIn(t)
      end
    elseif cmd == 0xAA then
      D.env = D.env + 1
      seenEnv = clock()
      if link == "CRSF" then bridgeIn(d) end
    end
  end

  -- ---------------------------------------------------------------- tabs
  -- custom_inav.txt: one INAV setting per line, "name" or "name, label, min, max, step"
  local function readCustom()
    custom = nil
    local text = io.readFile and io.inavCustomPath and io.readFile(io.inavCustomPath)
    if not text then C.customInfo = nil return end
    local list, ln, bad = {}, 0, nil
    for line in string.gmatch(text .. "\n", "([^\n]*)\n") do
      ln = ln + 1
      line = string.gsub(string.gsub(line, "#.*$", ""), "\r", "")
      local f = {}
      for x in string.gmatch(line .. ",", "([^,]*),") do f[#f + 1] = string.match(x, "^%s*(.-)%s*$") end
      local name = f[1] and string.lower(f[1]) or ""
      if name ~= "" then
        if #name > 32 or not string.match(name, "^[a-z0-9_]+$") then bad = bad or ln
        else
          local mn, mx, stp = tonumber(f[3] or ""), tonumber(f[4] or ""), tonumber(f[5] or "")
          local r = { name, (f[2] and f[2] ~= "") and f[2] or name, "", mn, mx, stp or 1, 0, k = 1 }
          if stp then
            local frac = string.match(string.format("%.4f", stp), "%.(%d+)$") or ""
            r[7] = #(string.gsub(frac, "0+$", ""))
          end
          if #list < 40 then list[#list + 1] = r end
        end
      end
    end
    if #list > 0 then custom = list end
    C.customInfo = bad and ("custom_inav.txt line " .. bad .. "?") or nil
  end

  local built = nil
  local function buildTabs()
    GV = {}
    if not LIST then return end
    for _, g in ipairs(LIST.GROUPS) do GV[#GV + 1] = g end
    if IN.variant == "INAV" then
      readCustom()
      if custom then GV[#GV + 1] = "CUSTOM" end
    end
  end
  function C.tabs()
    if built ~= LIST then built = LIST buildTabs() end
    return GV
  end

  function C.open(g)
    rows = nil
    collectgarbage()
    if g == "CUSTOM" then rows = custom or {}
    elseif IN.variant == "BTFL" then
      rows = {}
      for _, s in ipairs(BF.rows(g)) do rows[#rows + 1] = bfRow(s) end
    else rows = LIST and LIST.rows(g) or {} end
    forgetAll()
    rowsBy = {}
    for _, r in ipairs(rows) do rowsBy[r[1]] = r end
    if pend and not pend.m and not pend.set and not pend.check and not pend.bfpre and not pend.bfset then pend = nil end
    tab, edit = g, nil
    phase("")
    collectgarbage()
    return rows
  end
  function C.closeTab() tab, rows, edit = nil, nil, nil end
  function C.tab() return tab end
  function C.rows() return rows end
  function C.value(name) return vals[name] end
  function C.text(r) return txt[r[1]] or fmt(r, vals[r[1]]) end
  function C.progress()
    if not rows then return 0, 0 end
    local got = 0
    for _, r in ipairs(rows) do if vals[r[1]] ~= nil then got = got + 1 end end
    return got, #rows
  end
  local vTop, vN, sel = 1, 8, 1
  function C.view(top, n) vTop, vN = top or 1, n or 8 end
  function C.select(i) sel = i end

  -- the flight controller line for the tab list: "INAV 9.1.0", "Betaflight 4.5.5"
  function C.fcText()
    if not IN.variant then return "-" end
    return fcName() .. (IN.ver and (" " .. IN.ver) or "")
  end

  -- ---------------------------------------------------------------- link state
  local function blocked()
    local now = clock()
    local src = C.source
    if src == "SPORT" and now - seenSp >= 3 then return { "No S.Port / F.Port telemetry" } end
    if (src == "CRSF" or src == "MAVLINK") and now - seenCr >= 3 then return { "No " .. C.SOURCE_NAME[src] .. " telemetry" } end
    if not link then return { "No telemetry" } end
    local st = IN.state
    if st == "old" then return { "INAV too old", "Param Editor needs INAV 9.1", "or later (settings over the", "radio link, MSPv2)" } end
    if st == "buggy" then return { "INAV " .. (IN.ver or "9.0"), "INAV 9.0 sends no values", "over the radio link:", "update to INAV 9.1 or later" } end
    if st == "bfold" and IN.bfErr then return { "Betaflight list", "did not load:", string.sub(IN.bfErr, 1, 60) } end
    if st == "bfold" then return { "Betaflight " .. (IN.bfapi and ("API " .. IN.bfapi) or ""), "Param Editor reads", "Betaflight 4.5, 2025.12", "and 2026.6" } end
    if st == "notmsp" then return { "Unknown flight controller", "It answered MSP as " .. tostring(IN.variant), "(INAV and Betaflight only)" } end
    if quietAt then
      return { "No answer", "MSP over the radio link:", "ELRS / Crossfire / S.Port need", "INAV 9.1+ or Betaflight 4.5+" }
    end
    if st ~= "ready" then return { "Asking the flight controller", "INAV or Betaflight..." } end
    return nil
  end

  local function slowness()
    if path == "bridge" then return 2 end
    if link == "CRSF" then return 1 end
    return 0
  end
  local function timeout(set) return (set and 3 or 1.5) + slowness() end

  -- PID rows of a VTOL's other platform can't be changed (INAV keeps them in the inactive profile)
  local function pidLock(r)
    if not r or not r.pid or not IN.plat or IN.plat == "?" then return nil end
    local mc = MC_PLAT[IN.plat] == true
    if r.pid == "mc" and not mc then return "Hover PIDs: switch the VTOL to multirotor" end
    if r.pid == "fw" and mc then return "Plane PIDs: switch the VTOL to fixed wing" end
    return nil
  end
  local function tabHasPid()
    if not rows then return false end
    for _, r in ipairs(rows) do if r.pid then return true end end
    return false
  end

  local function nextRead()
    if not rows or edit then return nil end
    if IN.variant == "INAV" and IN.plat == nil and tabHasPid() then return "platform_type" end
    local r = rows[sel]
    if r and vals[r[1]] == nil then return r[1] end
    for i = vTop, math.min(#rows, vTop + vN - 1) do if vals[rows[i][1]] == nil then return rows[i][1] end end
    for i = 1, #rows do if vals[rows[i][1]] == nil then return rows[i][1] end end
    return nil
  end

  local function readReq(name)
    pend = { name = name, tries = 0 }
    if IN.variant == "BTFL" then
      local r = rowsBy[name]
      if not r or not r.bf then setVal(name, false, nil) pend = nil return end
      mspSend(r.bf[1])
      return
    end
    mspSend(M_SETTING_INFO, zname(name))
  end

  local function saveNow(s)
    local r = rowsBy[s.name]
    if not r then return end
    if IN.variant == "BTFL" then
      -- the block is read again first and the field set on those fresh bytes, so a change made
      -- elsewhere since (Configurator, another profile) is never written back old
      local raw = floor(s.v / r.k + 0.5)
      raw = math.max(r.rmin, math.min(r.rmax, raw))
      pend = { name = s.name, setRaw = raw, tries = 0, bfpre = true }
      mspSend(r.bf[1])
      return
    end
    local mt = IN.meta[s.name]
    if not mt then phase("Read it first", "bad") return end
    local raw
    if mt.look then raw = floor(s.v + 0.5) + mt.min
    else
      raw = s.v / (r.k or 1)
      if mt.t ~= 5 then raw = floor(raw + 0.5) end
    end
    if mt.t ~= 5 then raw = math.max(mt.min, math.min(mt.max, raw)) end
    pend = { name = s.name, set = true, setRaw = raw, tries = 0 }
    mspSend(M_SET_SETTING, encode(mt.t, raw, zname(s.name)))
  end

  C.blocked = nil
  function C.update()
    local now = clock()
    -- the link: S.Port when chosen or when only S.Port frames come, else CRSF (native or bridge)
    local want = nil
    local src = C.source
    if src == "SPORT" then want = "SPORT"
    elseif src == "CRSF" or src == "MAVLINK" then want = "CRSF"
    elseif now - seenCr < 3 then want = "CRSF"
    elseif now - seenSp < 3 then want = "SPORT"
    else want = link end
    local wantPath = (src == "CRSF" and "native") or (src == "MAVLINK" and "bridge") or nil
    if want ~= link or (wantPath and path ~= wantPath) then
      link, path = want, wantPath
      C.link = (want == "SPORT" and "SPORT") or (path == "bridge" and "MAVLINK") or want
      clearOut()
      pend, rxOn, BR.st, prevSp = nil, false, 0, nil
      answered, silent, quietAt = false, 0, nil
    end
    if link == "CRSF" then C.link = path == "bridge" and "MAVLINK" or "CRSF" end
    if quietAt and now - quietAt >= 15 then quietAt, silent = nil, 0 end
    local b = blocked()
    C.blocked = b
    if not link then return end
    if #out > 0 then
      pushOne()
      if #out == 0 and pend then pend.at = now end
      return
    end
    -- a waiting request: its answer, else asked again (three tries), else given up
    if saveReq and pend and pend.name and not pend.set and not pend.check and not pend.bfpre
       and not pend.bfset and pend.name ~= saveReq.name then pend = nil end
    if pend then
      if not pend.at then pend.at = now end
      local saving = pend.set or pend.bfset or pend.bfpre
      if now - pend.at < timeout(saving) then return end
      if saving then   -- never sent again by itself: it may have been taken
        phase("No answer: check in " .. fcName() .. " Configurator", "bad")
        vals[pend.name or ""], pend = nil, nil
        return
      end
      pend.tries = pend.tries + 1
      if pend.tries >= 3 then
        local m, name = pend.m, pend.name
        pend = nil
        if m == "store" then
          IN.storeHold = now + 10
          phase("Saved, NOT stored yet (no answer): trying again", "bad")
          return
        end
        if m == "status" then return end
        -- a row the FC has answered other requests around: n/a (not in this build / version)
        if name and answered then
          if name == "platform_type" then IN.plat = "?" else setVal(name, false, nil) end
          return
        end
        silent = silent + 1
        if silent >= 2 then quietAt = now end
        return
      end
      pend.at = nil
      again()
      return
    end
    if IN.state == "variant" then pend = { m = "variant", tries = 0 } mspSend(MSP_FC_VARIANT) return end
    if IN.state == "bfapi" then pend = { m = "bfapi", tries = 0 } mspSend(MSP_API) return end
    if IN.state == "version" then pend = { m = "version", tries = 0 } mspSend(MSP_FC_VERSION) return end
    if IN.state ~= "ready" or b then return end
    -- the FC's own arming state, every 2 s (1 s while a change waits to be stored)
    if now - IN.statusAt >= (IN.dirty and 1 or STATUS_EVERY) then
      IN.statusAt = now
      pend = { m = "status", tries = 0 }
      mspSend(IN.variant == "BTFL" and MSP_STATUS_EX or MSP2_INAV_STATUS)
      return
    end
    -- stored only when the FC said "disarmed" within the last 3 s
    if IN.dirty and IN.armed == false and now - (IN.armedAt or -10) < 3 and now >= IN.storeHold then
      pend = { m = "store", tries = 0 }
      mspSend(MSP_EEPROM_WRITE)
      return
    end
    if saveReq then
      local s = saveReq
      saveReq = nil
      saveNow(s)
      return
    end
    local n = nextRead()
    if n then readReq(n) end
  end

  -- ---------------------------------------------------------------- editing
  function C.setSource(src)
    if not C.SOURCE_NAME[src] or src == C.source then return end
    C.source = src
    if io.writeFile and io.settingsPath then pcall(io.writeFile, io.settingsPath, "source=" .. src .. "\n") end
  end
  function C.nextSource()
    for i, v in ipairs(C.SOURCES) do
      if v == C.source then C.setSource(C.SOURCES[i % #C.SOURCES + 1]) return end
    end
  end

  function C.locked(r)
    if pidLock(r) then return true end
    return C.armed and not C.airEdits
  end
  function C.editing() return edit, draft, base end

  function C.beginEdit(i)
    local r = rows and rows[i]
    if not r then return false end
    local v = vals[r[1]]
    if v == nil then phase("Reading...") return false end
    if v == false then vals[r[1]], txt[r[1]] = nil, nil phase("Asking again...") return false end
    edit, draft, base = r, v, v
    phase("")
    local pl = pidLock(r)
    if pl then phase(pl, "bad")
    elseif C.locked(r) then phase("Armed: Air edits off", "bad")
    elseif C.armed then phase("In flight: set at once, stored after the disarm", "bad") end
    if not pend and #out == 0 then readReq(r[1]) end   -- read again: may have changed elsewhere
    return true
  end
  function C.endEdit() edit = nil phase("") end

  function C.nudge(d)
    local r = edit
    if not r or C.locked(r) then return end
    local st = r[6] or 1
    local v = draft + d * st
    v = floor(v / st * 1000 + 0.5) * st / 1000
    if r[4] then v = math.max(r[4], v) end
    if r[5] then v = math.min(r[5], v) end
    draft = v
    phase("")
  end
  function C.toggleBit() end
  function C.setDraft(v) if edit and not C.locked(edit) then draft = v phase("") end end
  function C.save()
    if not edit then return nil end
    if C.locked(edit) then phase(pidLock(edit) or "Armed: Air edits off", "bad") return nil end
    if pend and (pend.set or pend.bfset or pend.bfpre) then return nil end
    if draft == base then edit = nil phase("") return "closed" end
    saveReq = { name = edit[1], v = draft }
    phase("Saving...")
    return "saving"
  end
  function C.saving()
    return saveReq ~= nil or (pend ~= nil and (pend.set or pend.check or pend.bfset or pend.bfpre) ~= nil)
  end

  function C.diagText()
    local D, now = C.diag, clock()
    local function ago(t) return t < 0 and "-" or string.format("%.0fs", now - t) end
    return string.format("MSP %s  CRSF %d last %s  S.Port %d  mLRS %d  rx %d", tostring(path or link or "-"),
      D.cr, ago(seenCr), D.sp, D.env, C.stats.rx)
  end
  -- for pick.lua: nothing answered MSP (back to waiting for ArduPilot)
  function C.noMsp() return IN.state == "variant" and quietAt ~= nil end
  function C.debug()
    return string.format("%s/%s %s tx%d rx%d bad%d q%d %s", tostring(link), tostring(path), IN.state,
      C.stats.tx, C.stats.rx, C.stats.bad, #out, pend and (pend.name or pend.m) or "-")
  end
  return C
end
