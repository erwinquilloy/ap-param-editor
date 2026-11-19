--[[
  AP Param Editor, engine (EdgeTX and Ethos share this file byte for byte).
  Reads and changes ArduPilot parameters from the radio over the link the model already has:

    S.Port / F.Port    any ArduPilot with FrSky passthrough telemetry (SERIALn_PROTOCOL 10, or
                       F.Port with passthrough): ArduPilot's own MAVlite uplink.
                       radio -> FC  physId 0x0D, primId 0x30;  FC -> radio  primId 0x32
                       each frame: appId low byte = sequence (0 starts a message), then 5 message
                       bytes in appId high byte + value bytes 0-3; message = len, msgid, payload,
                       checksum (c = c + b; c = c + (c >> 8); c = c & 0xFF over every byte)
    ELRS / Crossfire   ArduPlane Stratos v2.2.1+ (or ArduPilot with PR #35160): MAVlite in one CRSF
                       frame: radio -> FC 0x80 C8 EA F4 len msgid payload, FC -> radio 0x80 F4 len
                       msgid payload
    mLRS               Tx Ser Port = cbridge, any ArduPilot: MAVLink v2 in CRSF frames 0xAA (chunk
                       byte, size, up to 58 bytes) both ways. The mLRS module sends nothing over the
                       bridge until the radio has, so a GCS HEARTBEAT goes out every 2 s on CRSF
                       (ELRS and Crossfire ignore the frame type).

  PARAM_REQUEST_READ 20 / PARAM_SET 23 -> PARAM_VALUE 22 (float32 value, name). Over MAVlite a name
  the FC doesn't have gets only the text "Param read failed (NAME)"; over MAVLink nothing (n/a after
  the tries).

  Rules (as OmniDeck's PARAMS page): only the open tab is read, the rows on screen first; one request
  at a time; a save is sent once and is "Saved" only when the value read back is the same four bytes,
  and never sent again by itself; view only while armed, except the PID tabs and the rows marked
  fl = true when Air edits is on.

  Returns a maker: io -> C. io (the platform's part):
    clock()               seconds
    push(f)               send one frame, false if the radio can't take it now (tried again later)
                          f.sp: S.Port { sp = true, appId, value }   f.cmd: CRSF { cmd, data }
    loadList()            the tab list module (list.lua): function(tab) -> rows, and .GROUPS
    readFile(path)        the file's text or nil (CUSTOM tab io.customPath, settings io.settingsPath)
    writeFile(path, s)    optional: keeps the telemetry source choice (io.settingsPath)
    busy()                optional: true when this update has used enough CPU (EdgeTX getUsage)
  The platform hands every frame over: C.sport(primId, appId, value), C.crsf(cmd, data).
]]
local floor = math.floor
local REQ_READ, PARAM_VALUE, PARAM_SET = 20, 22, 23
local unpack = table.unpack or unpack

-- ---------------------------------------------------------------- float32 <-> bytes
-- byte by byte, so every step stays below 2^24 (exact where Lua numbers are 32-bit floats)
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

-- A 32-bit S.Port value -> its 16-bit halves. Ethos may run Lua with 32-bit integers (X18), where
-- "v % 4294967296" turns the value into a float32 first and loses its low bits: split with & and >>
-- where Lua has them (compiled from a string: EdgeTX's Lua 5.2 can't parse the operators).
local splitInt = math.type and load("return function(v) return v & 0xFFFF, (v >> 16) & 0xFFFF end")
splitInt = splitInt and splitInt() or nil
local function split32(v)
  if splitInt and math.type(v) == "integer" then return splitInt(v) end
  v = v % 4294967296
  return v % 65536, floor(v / 65536)
end

-- CRC-16/MCRF4XX (MAVLink), a byte at a time: a 256-entry table would be 2 KB more on a 128 KB radio
local crcAcc
if bit32 then
  local band, bxor, shl, shr = bit32.band, bit32.bxor, bit32.lshift, bit32.rshift
  crcAcc = function(b, c)
    local t = band(bxor(b, c), 0xFF)
    t = band(bxor(t, shl(t, 4)), 0xFF)
    return band(bxor(bxor(shr(c, 8), shl(t, 8)), bxor(shl(t, 3), shr(t, 4))), 0xFFFF)
  end
else
  crcAcc = load("return function(b, c) local t = (b ~ c) & 0xFF t = (t ~ (t << 4)) & 0xFF "
    .. "return ((c >> 8) ~ (t << 8) ~ (t << 3) ~ (t >> 4)) & 0xFFFF end")()
end
local CRCX = { [0] = 50, [20] = 214, [22] = 220, [23] = 168 }   -- CRC_EXTRA of the messages used

-- GCS HEARTBEAT (255/25, type GCS, autopilot invalid) as one bridge frame, worked out beforehand
-- (tests check it): it wakes the mLRS bridge
local HB = { 0, 21, 253, 9, 0, 0, 0, 255, 25, 0, 0, 0, 0, 0, 0, 0, 6, 8, 0, 0, 3, 186, 120 }

-- ARMING_SKIPCHK (Plane bits, AP_Arming.cpp): a set bit = that check is skipped
local BITS = { { 1, "Barometer" }, { 2, "Compass" }, { 3, "GPS lock" }, { 4, "INS" }, { 5, "Parameters" },
  { 6, "RC channels" }, { 7, "Board voltage" }, { 8, "Battery level" }, { 9, "Airspeed" }, { 10, "Logging" },
  { 11, "Safety switch" }, { 12, "GPS config" }, { 13, "System" }, { 14, "Mission" }, { 15, "Rangefinder" },
  { 16, "Camera" }, { 17, "AUX auth" }, { 19, "FFT" } }
local ALL_MASK = 0
for _, b in ipairs(BITS) do ALL_MASK = ALL_MASK + 2 ^ b[1] end
local function skipped(v, bit) return v == -1 or (v >= 0 and floor(v / 2 ^ bit) % 2 == 1) end
local function skipCount(v)
  local n = 0
  for _, b in ipairs(BITS) do if skipped(v, b[1]) then n = n + 1 end end
  return n
end

return function(io)
  local C = { BITS = BITS, skipped = skipped, skipCount = skipCount, f2b = f2b, b2f = b2f }
  C.VERSION = "0.1.0"
  C.fc, C.link, C.armed, C.airEdits = nil, nil, false, false
  -- telemetry source: AUTO picks whichever brought ArduPilot frames last (mLRS bridge, then CRSF,
  -- then S.Port); the others use that link only (two links at once, or a link Auto mistakes)
  C.SOURCES = { "AUTO", "SPORT", "CRSF", "MAVLINK" }
  C.SOURCE_NAME = { AUTO = "Auto", SPORT = "S.Port / F.Port", CRSF = "ELRS / TBS (CRSF)", MAVLINK = "mLRS (MAVLink)" }
  C.source = "AUTO"
  do
    local t = io.readFile and io.settingsPath and io.readFile(io.settingsPath)
    local v = t and string.match(t, "source=(%u+)")
    if v and C.SOURCE_NAME[v] then C.source = v end
  end
  C.msgs = {}                       -- last status texts { text, sev }
  C.stats = { tx = 0, rx = 0, bad = 0 }
  local clock = io.clock

  local seenSp, seenCr, seenEnv, lastRx = -100, -100, -100, -100
  -- CRSF: ArduPilot's passthrough frames (0x80 F0-F4) say who is there, but on a slow ELRS / TBS
  -- telemetry rate they can be seconds apart. The native frames the FC sends (GPS, battery,
  -- attitude, flight mode...) keep the link counted as alive for 30 s after the last passthrough.
  local seenCrNat = -100
  local NATIVE = { [0x02] = true, [0x07] = true, [0x08] = true, [0x09] = true, [0x1E] = true, [0x21] = true }
  -- what came in: shown on the "No ... telemetry" screen (frames since the tool opened)
  C.diag = { cr = 0, crPt = 0, crNat = 0, sp = 0, spPt = 0, env = 0 }
  local function crAlive(now) return now - seenCr < 3 or (now - seenCrNat < 3 and now - seenCr < 30) end
  local function heard(t) lastRx = t C.fc = "ArduPilot" end
  function C.alive() return clock() - lastRx < 3 end

  local function pushMsg(text, sev)
    local m = C.msgs
    if #m >= 6 then table.remove(m, 1) end
    m[#m + 1] = { text = text, sev = sev }
  end

  -- ---------------------------------------------------------------- send queue
  local out = {}
  local function clearOut() for i = #out, 1, -1 do out[i] = nil end end

  -- MAVLink v2 (mLRS bridge)
  local MV = { sys = 1, comp = 1, seq = 0, found = false }
  local function mavQueue(msgid, pl)
    local p = { 0xFD, #pl, 0, 0, MV.seq, 255, 25, msgid, 0, 0 }   -- we are a GCS, 255/25
    MV.seq = (MV.seq + 1) % 256
    for i = 1, #pl do p[#p + 1] = pl[i] end
    local crc = 0xFFFF
    for i = 2, #p do crc = crcAcc(p[i], crc) end
    crc = crcAcc(CRCX[msgid], crc)
    p[#p + 1] = crc % 256
    p[#p + 1] = floor(crc / 256)
    local i = 1
    while i <= #p do
      local n = math.min(58, #p - i + 1)
      local e = { 0, n }
      for k = 0, n - 1 do e[k + 3] = p[i + k] end
      out[#out + 1] = { cmd = 0xAA, data = e }
      i = i + n
    end
  end
  local function name16(name, t)
    for k = 1, 16 do t[#t + 1] = string.byte(name, k) or 0 end
    return t
  end

  local function ck(c, b) c = c + b c = c + floor(c / 256) return c % 256 end

  local link = nil   -- "SPORT", "CRSF", "MAVLINK"
  local function send(msgid, name, vb)
    if link == "MAVLINK" then
      if msgid == REQ_READ then   -- index -1, target, name[16]
        mavQueue(20, name16(name, { 0xFF, 0xFF, MV.sys, MV.comp }))
      else                        -- value, target, name[16], type REAL32
        local t = name16(name, { vb[1], vb[2], vb[3], vb[4], MV.sys, MV.comp })
        t[#t + 1] = 9
        mavQueue(23, t)
      end
      return
    end
    local pl = {}
    if vb then pl[1], pl[2], pl[3], pl[4] = vb[1], vb[2], vb[3], vb[4] end
    for k = 1, math.min(16, #name) do pl[#pl + 1] = string.byte(name, k) end
    if link == "CRSF" then
      local d = { 0xC8, 0xEA, 0xF4, #pl, msgid }
      for i = 1, #pl do d[#d + 1] = pl[i] end
      out[#out + 1] = { cmd = 0x80, data = d }
      return
    end
    -- S.Port: 6-byte chunks (sequence + 5 message bytes), the checksum after the last byte
    local bytes = { #pl, msgid }
    for i = 1, #pl do bytes[#bytes + 1] = pl[i] end
    local c, seq, i, done = 0, 0, 1, false
    repeat
      local ch = { seq }
      c = ck(c, seq)
      seq = seq + 1
      while #ch < 6 and i <= #bytes do ch[#ch + 1] = bytes[i] c = ck(c, bytes[i]) i = i + 1 end
      if i > #bytes and #ch < 6 then ch[#ch + 1] = c done = true end
      while #ch < 6 do ch[#ch + 1] = 0 end
      out[#out + 1] = { sp = true, appId = ch[1] + ch[2] * 256,
                        value = ch[3] + ch[4] * 256 + ch[5] * 65536 + ch[6] * 16777216 }
    until done
  end

  local function pushOne()
    local f = out[1]
    local ok, r = pcall(io.push, f)
    if ok and r ~= false then
      table.remove(out, 1)
      C.stats.tx = C.stats.tx + 1
    end
  end

  -- ---------------------------------------------------------------- values
  local vals, vbs, txt = {}, {}, {}  -- name -> value (false: n/a), its 4 bytes as a string, shown text
  local qOn = nil                    -- Q_ENABLE: nil not read yet, true / false
  local LISTMOD = nil
  local GROUPS, QG, AIR = {}, {}, {}
  local custom = nil                 -- CUSTOM tab rows (custom.txt), nil: none
  C.customInfo = nil                 -- "n rows" / "line 3?" for the tab list

  local function fmt(r, v)
    if v == nil then return "..." end
    if v == false then return "n/a" end
    if r[9] == "bits" then
      if v == 0 then return "All checks" end
      if v == -1 then return "All skipped" end
      return skipCount(v) .. " skipped"
    end
    if r[8] then
      local i, k = floor(v + 0.5), 0
      for o in string.gmatch(r[8], "[^|]+") do
        if k == i then return o end
        k = k + 1
      end
      return tostring(i)
    end
    local t = string.format("%." .. (r[7] or 3) .. "f", v)
    if r[3] and r[3] ~= "" then t = t .. " " .. r[3] end
    return t
  end
  C.fmt = fmt

  -- CUSTOM rows: the value's own decimals (up to 4) give the step until the file says otherwise
  local function autoStep(r, v)
    if not r.free then return end
    local d = 0
    local s = string.format("%.4f", math.abs(v))
    local frac = string.match(s, "%.(%d+)$") or ""
    frac = string.gsub(frac, "0+$", "")
    d = #frac
    r[7] = r.dec or d
    r[6] = r.st or 10 ^ -(r.dec or d)
  end

  local tab, rows, rowsBy = nil, nil, {}
  local function setVal(name, v, bytes)
    vals[name], vbs[name] = v, bytes
    local r = rowsBy[name]
    if r and v then autoStep(r, v) end
    txt[name] = r and fmt(r, v) or nil
    if name == "Q_ENABLE" then qOn = v ~= false and v >= 1 end
  end

  local edit, draft, base = nil, nil, nil
  C.phase, C.phaseKind = "", ""
  local function phase(t, k) C.phase, C.phaseKind = t, k or "" end
  local pend, saveReq = nil, nil   -- request waiting for its answer { name, set|check = bytes, tries, at }
  local answered, silent, quietAt = false, 0, nil
  local hbAt, prefAt = -10, {}

  local function handle(id, b)
    C.stats.rx = C.stats.rx + 1
    answered, silent, quietAt = true, 0, nil
    if id ~= PARAM_VALUE or #b < 5 then return end
    local n = 4
    for i = 5, math.min(#b, 20) do if b[i] == 0 then break end n = i end
    local name = string.char(unpack(b, 5, n))
    if name ~= "Q_ENABLE" and not rowsBy[name] then return end   -- another tab, or a GCS's request
    local bytes = string.char(b[1], b[2], b[3], b[4])
    local v = b2f(b[1], b[2], b[3], b[4])
    setVal(name, v, bytes)
    if pend and pend.name == name then
      local want = pend.set or pend.check
      if want then
        if want == bytes then phase("Saved and verified", "ok")
        else phase("Not saved: FC has " .. fmt(rowsBy[name] or {}, v), "bad") end
        if edit and edit[1] == name then draft, base = v, v end
      elseif edit and edit[1] == name and draft == base then
        draft, base = v, v       -- a fresh read while the value is not being changed
      end
      pend = nil
    end
  end

  -- ---------------------------------------------------------------- receive: S.Port
  local rxSt, rxExp, rxLen, rxId, rxSum, rxBuf = nil, 0, 0, 0, 0, {}
  local prev = nil          -- last MAVlite frame (F.Port sends each one twice)
  local function onLite(b)
    local seq = b[1] % 64
    if seq == 0 then rxSt, rxExp, rxSum, rxBuf = 1, 0, 0, {} end
    if rxSt == nil or seq ~= rxExp then rxSt = nil return end
    rxSum = ck(rxSum, seq)
    rxExp = seq + 1
    for i = 2, 6 do
      local x = b[i]
      if rxSt == 1 then rxLen, rxSt = x, 2 rxSum = ck(rxSum, x)
      elseif rxSt == 2 then rxId, rxSt = x, (rxLen == 0 and 4 or 3) rxSum = ck(rxSum, x)
      elseif rxSt == 3 then
        rxBuf[#rxBuf + 1] = x rxSum = ck(rxSum, x)
        if #rxBuf >= rxLen then rxSt = 4 end
      elseif rxSt == 4 then
        rxSt = nil
        if x == rxSum then
          if link == "SPORT" then handle(rxId, rxBuf) end
        else C.stats.bad = C.stats.bad + 1 end
        return
      end
    end
  end

  -- status text 0x5000: 4 characters per frame (7 bits each, high bits carry the severity), a 0
  -- ends it. ArduPilot sends each chunk twice: an exact copy of the last one is dropped.
  local tbuf, tlast = "", nil
  local function onText(lo, hi)
    local key = hi * 65536 + lo
    if key == tlast then return end
    tlast = key
    local cs = { floor(hi / 256) % 128, hi % 128, floor(lo / 256) % 128, lo % 128 }
    for i = 1, 4 do
      if cs[i] == 0 then
        if #tbuf > 0 then pushMsg(tbuf, 0) end
        tbuf = ""
        return
      end
      tbuf = tbuf .. string.char(cs[i])
    end
    if #tbuf > 60 then tbuf = "" end
  end

  function C.sport(primId, appId, value)
    if appId == nil or value == nil then return end
    local now = clock()
    C.diag.sp = C.diag.sp + 1
    if primId == 0x32 then
      local lo, hi = split32(value)
      local b = { appId % 256, floor(appId / 256) % 256, lo % 256, floor(lo / 256), hi % 256, floor(hi / 256) }
      local p = prev
      if p and p[1] == b[1] and p[2] == b[2] and p[3] == b[3] and p[4] == b[4] and p[5] == b[5] and p[6] == b[6] then return end
      prev = b
      seenSp = now
      heard(now)
      onLite(b)
    elseif primId == 0x10 and appId >= 0x5000 and appId <= 0x50FF then
      seenSp = now
      heard(now)
      if appId == 0x5001 then
        local lo = split32(value)
        C.armed = floor(lo / 256) % 2 == 1
      elseif appId == 0x5000 then
        onText(split32(value))
      end
    end
  end

  -- ---------------------------------------------------------------- receive: CRSF
  local envQ = {}
  local function value5001(appLo, appHi, v1) if appLo == 0x01 and appHi == 0x50 then C.armed = v1 % 2 == 1 end end
  function C.crsf(cmd, d)
    if not d then return end
    local now = clock()
    local D = C.diag
    D.cr = D.cr + 1
    if NATIVE[cmd] then
      D.crNat = D.crNat + 1
      seenCrNat = now
      if now - seenCr < 30 then heard(now) end
      return
    end
    if cmd == 0xAA then
      D.env = D.env + 1
      seenEnv = now
      if MV.found then heard(now) end
      if #envQ >= 16 then table.remove(envQ, 1) end
      envQ[#envQ + 1] = d
      return
    end
    if cmd ~= 0x80 and cmd ~= 0x7F then return end
    D.crPt = D.crPt + 1
    local k = d[1]
    if k == 0xF4 then
      seenCr = now
      heard(now)
      if link ~= "CRSF" then return end
      local n = d[2] or 0
      if #d < 3 + n then C.stats.bad = C.stats.bad + 1 return end
      local b = {}
      for i = 1, n do b[i] = d[3 + i] end
      handle(d[3], b)
    elseif k == 0xF0 or k == 0xF1 or k == 0xF2 then
      seenCr = now
      heard(now)
      if k == 0xF0 and #d >= 7 then
        value5001(d[2], d[3], d[5])
      elseif k == 0xF2 then
        for i = 0, math.min((d[2] or 0) - 1, 9) do
          local b = 3 + 6 * i
          if d[b + 5] == nil then break end
          value5001(d[b], d[b + 1], d[b + 3])
        end
      elseif k == 0xF1 and #d >= 3 then
        local t = string.char(unpack(d, 3, #d))
        local z = string.find(t, "\0", 1, true)
        if z then t = string.sub(t, 1, z - 1) end
        pushMsg(t, d[2])
      end
    end
  end

  -- ---------------------------------------------------------------- receive: MAVLink (mLRS bridge)
  local function u8(b, i) return b[i] or 0 end   -- MAVLink 2 leaves out trailing zero bytes
  local function mavMessage(id, b, sys, comp)
    if id == 0 then
      if u8(b, 6) == 3 and comp == 1 and (not MV.found or sys == MV.sys) then   -- autopilot ArduPilotMega
        MV.sys, MV.comp, MV.found = sys, comp, true
        C.armed = u8(b, 7) >= 128                                             -- base_mode SAFETY_ARMED
        heard(clock())
      end
      return
    end
    if sys ~= MV.sys then return end
    local nb = { u8(b, 1), u8(b, 2), u8(b, 3), u8(b, 4) }
    for i = 9, 24 do
      local c = u8(b, i)
      if c == 0 then break end
      nb[#nb + 1] = c
    end
    if link == "MAVLINK" then handle(22, nb) end
  end
  -- packets run across frames: header (10 bytes), then the body read or skipped
  local st, hn, hdr, skip, need, got, body, mid = 0, 0, {}, 0, 0, 0, {}, 0
  local function feed(data)
    local e = 2 + (data[2] or 0)
    if e > #data then e = #data end
    local i = 3
    while i <= e do
      if skip > 0 then
        local k = e - i + 1
        if k > skip then k = skip end
        skip, i = skip - k, i + k
      elseif st == 0 then
        if data[i] == 0xFD then st, hn, hdr[1] = 1, 1, 0xFD end
        i = i + 1
      elseif st == 1 then
        hn = hn + 1
        hdr[hn] = data[i]
        i = i + 1
        if hn == 10 then
          local len, signed = hdr[2], hdr[3] % 2 == 1
          local m = (hdr[9] == 0 and hdr[10] == 0) and hdr[8] or -1
          if not signed and (m == 22 or m == 0) then
            st, need, got, body, mid = 2, len + 2, 0, {}, m
          else
            st, skip = 0, len + 2 + (signed and 13 or 0)
          end
        end
      else
        got = got + 1
        body[got] = data[i]
        i = i + 1
        if got == need then
          st = 0
          local len = need - 2
          local crc = 0xFFFF
          for k = 2, 10 do crc = crcAcc(hdr[k], crc) end
          for k = 1, len do crc = crcAcc(body[k], crc) end
          crc = crcAcc(CRCX[mid], crc)
          if crc == body[len + 1] + body[len + 2] * 256 then
            body[len + 1], body[len + 2] = nil, nil
            mavMessage(mid, body, hdr[6], hdr[7])
          else
            C.stats.bad = C.stats.bad + 1
          end
        end
      end
    end
  end

  -- ---------------------------------------------------------------- tabs
  local function listMod()
    if not LISTMOD then
      LISTMOD = io.loadList()
      GROUPS, QG, AIR = LISTMOD.GROUPS, LISTMOD.QG, LISTMOD.AIR
    end
    return LISTMOD
  end

  -- custom.txt: one parameter per line, "NAME" or "NAME, label, min, max, step"
  -- (anything after the name may be left out; # starts a comment)
  local function readCustom()
    custom = nil
    local text = io.readFile and io.customPath and io.readFile(io.customPath)
    if not text then C.customInfo = nil return end
    local list, ln, bad = {}, 0, nil
    for line in string.gmatch(text .. "\n", "([^\n]*)\n") do
      ln = ln + 1
      line = string.gsub(line, "#.*$", "")
      line = string.gsub(line, "\r", "")
      local f = {}
      for x in string.gmatch(line .. ",", "([^,]*),") do f[#f + 1] = string.match(x, "^%s*(.-)%s*$") end
      local name = f[1] and string.upper(f[1]) or ""
      if name ~= "" then
        if #name > 16 or not string.match(name, "^[A-Z0-9_]+$") then bad = bad or ln
        else
          local mn, mx, stp = tonumber(f[3] or ""), tonumber(f[4] or ""), tonumber(f[5] or "")
          local r = { name, (f[2] and f[2] ~= "") and f[2] or name, "", mn, mx, stp or 1, nil, free = true }
          if stp then
            r.st = stp
            local frac = string.match(string.format("%.4f", stp), "%.(%d+)$") or ""
            r.dec = #(string.gsub(frac, "0+$", ""))
            r[7] = r.dec
          end
          if #list < 40 then list[#list + 1] = r end
        end
      end
    end
    if #list > 0 then custom = list end
    C.customInfo = bad and ("custom.txt line " .. bad .. "?") or nil
  end

  local GV = {}
  local function buildTabs()
    listMod()
    GV = {}
    for _, g in ipairs(GROUPS) do if not QG[g] or qOn == true then GV[#GV + 1] = g end end
    if custom then GV[#GV + 1] = "CUSTOM" end
  end
  function C.tabs() return GV end

  function C.open(g)
    rows = nil
    collectgarbage()
    if g == "CUSTOM" then rows = custom or {}
    else rows = listMod().rows(g) or {} end
    -- only this tab's values are kept (and Q_ENABLE): another tab is read again when opened
    local q, qb = vals.Q_ENABLE, vbs.Q_ENABLE
    vals, vbs, txt, rowsBy, prefAt = { Q_ENABLE = q }, { Q_ENABLE = qb }, {}, {}, {}
    for _, r in ipairs(rows) do rowsBy[r[1]] = r end
    if pend and pend.name ~= "Q_ENABLE" and not pend.set then pend = nil end
    tab, edit = g, nil
    phase("")
    collectgarbage()
    return rows
  end
  function C.closeTab() tab, rows, edit = nil, nil, nil end
  function C.tab() return tab end
  function C.rows() return rows end
  function C.value(name) return vals[name] end
  function C.text(r)
    local v = vals[r[1]]
    return txt[r[1]] or fmt(r, v)
  end
  function C.progress()
    if not rows then return 0, 0 end
    local got = 0
    for _, r in ipairs(rows) do if vals[r[1]] ~= nil then got = got + 1 end end
    return got, #rows
  end
  local vTop, vN = 1, 8
  function C.view(top, n) vTop, vN = top or 1, n or 8 end   -- rows on screen: read first
  local sel = 1
  function C.select(i) sel = i end

  -- ---------------------------------------------------------------- link state
  local function failedText(name)
    local m = C.msgs
    for i = #m, math.max(1, #m - 5), -1 do
      if string.find(m[i].text, "failed (" .. name .. ")", 1, true) then return true end
    end
    return false
  end

  local HINT = {
    SPORT = { "FC: SERIALn_PROTOCOL 10", "(FrSky passthrough)" },
    CRSF = { "FC: ArduPlane Stratos v2.2.1+", "(MAVlite over CRSF)" },
    MAVLINK = { "mLRS Tx Ser Port = cbridge" },
  }
  -- why the editor can't work now (nil = it can): a title and up to four lines
  local function blocked()
    local src = C.source
    if src ~= "AUTO" then
      local now = clock()
      local seen = (src == "SPORT" and seenSp) or (src == "CRSF" and seenCr) or seenEnv
      local alive = (src == "CRSF" and crAlive(now)) or (src ~= "CRSF" and now - seen < 3)
      if not alive or (src == "MAVLINK" and not MV.found) then
        local l = { "No " .. C.SOURCE_NAME[src] .. " telemetry" }
        if src == "MAVLINK" and now - seen < 3 then l[2] = "bridge on, no HEARTBEAT yet" end
        for _, h in ipairs(HINT[src]) do l[#l + 1] = h end
        return l
      end
    end
    if C.fc == nil then
      if clock() - seenEnv < 3 then return { "Waiting for ArduPilot", "mLRS bridge on,", "no HEARTBEAT yet" } end
      return { "Waiting for ArduPilot", "S.Port / F.Port passthrough,", "ELRS / TBS (Stratos 2.2.1+)", "or mLRS (Tx Ser Port cbridge)" }
    end
    if not C.alive() then return { "No telemetry" } end
    if quietAt then
      if link == "MAVLINK" then return { "No answer", "mLRS bridge on, no", "parameter answers yet" } end
      if link == "CRSF" then return { "No answer", "ELRS / TBS needs ArduPlane", "Stratos v2.2.1 or later", "(or mLRS: cbridge)" } end
      return { "No answer", "S.Port: SERIALn_PROTOCOL 10", "(passthrough) on the FC" }
    end
    return nil
  end

  local function timeout(set)
    if link == "SPORT" then return set and 3 or 1.5 end
    return (set and 4 or 2.5) + (link == "MAVLINK" and 1 or 0)
  end
  local function tries() return link == "MAVLINK" and 4 or (link == "SPORT" and 2 or 3) end

  local function nextRead()
    if qOn == nil then return "Q_ENABLE" end
    if not rows or edit then return nil end
    local r = rows[sel]
    if r and vals[r[1]] == nil then return r[1] end
    for i = vTop, math.min(#rows, vTop + vN - 1) do if vals[rows[i][1]] == nil then return rows[i][1] end end
    for i = 1, #rows do if vals[rows[i][1]] == nil then return rows[i][1] end end
    return nil
  end

  local function readReq(name)
    pend = { name = name, tries = 0 }
    send(REQ_READ, name)
  end

  local custRead, qBuilt = false, "unset"   -- Q_ENABLE changes the tab list (VTOL tabs)
  C.blocked = nil
  function C.update()
    local now = clock()
    if not custRead then custRead = true readCustom() buildTabs() end
    -- the link: the mLRS bridge, else CRSF, else S.Port (whichever brought frames last 3 s)
    local want = C.source
    if want == "AUTO" then
      -- bridge frames (0xAA) come only from an mLRS cbridge, so they win at once, as in OmniDeck:
      -- the HEARTBEAT that sets MV.found is parsed only on the MAVLINK link, and mLRS also sends
      -- CRSF telemetry (native + passthrough), so waiting for it kept Auto on CRSF (radio,
      -- 2026-11-19: "CRSF", then "No answer")
      want = (now - seenEnv < 3 and "MAVLINK") or (crAlive(now) and "CRSF")
        or (now - seenSp < 3 and "SPORT") or link
    end
    if want ~= link then
      link = want
      C.link = want
      clearOut()
      pend, prefAt = nil, {}
      answered, silent, quietAt = false, 0, nil
    end
    if link == "MAVLINK" then
      for _ = 1, 4 do
        if io.busy and io.busy() then break end
        local d = table.remove(envQ, 1)
        if not d then break end
        feed(d)
      end
    elseif #envQ > 0 then envQ = {} end
    if qOn ~= qBuilt then qBuilt = qOn buildTabs() end
    if quietAt and now - quietAt >= 15 then quietAt, silent = nil, 0 end
    local b = blocked()
    C.blocked = b
    if b then
      if pend or #out > 0 then clearOut() pend = nil end
    end
    -- GCS HEARTBEAT every 2 s on anything but S.Port: wakes the mLRS bridge, which sends nothing
    -- before the radio has (ELRS / Crossfire ignore the frame; no CRSF module: the push fails)
    if link ~= "SPORT" and io.crsfOk ~= false and now - hbAt >= 2 then
      hbAt = now
      pcall(io.push, { cmd = 0xAA, data = HB })
    end
    if b then return end
    if #out > 0 then
      pushOne()
      if #out == 0 and pend then pend.at = now end
      return
    end
    -- a save goes first: a row read still waiting is dropped (asked again later; a late answer is
    -- kept anyway). Not a read of the row being saved: its old value would look like the answer.
    if saveReq and pend and not pend.set and not pend.check and pend.name ~= saveReq.name then pend = nil end
    if pend then
      if not pend.at then pend.at = now end
      local name = pend.name
      -- ArduPilot answers a name it doesn't have with a text only: taken as soon as it is here
      if not pend.set and link ~= "MAVLINK" and failedText(name) then
        answered, silent = true, 0
        setVal(name, false, nil)
        pend = nil
        return
      end
      if now - pend.at < timeout(pend.set) then return end
      if pend.set then
        if link == "MAVLINK" then
          -- mLRS: ArduPilot saves at once but its answer can be lost on a full link: read it back
          pend = { name = name, check = pend.set, tries = 0 }
          phase("Checking...")
          send(REQ_READ, name)
          return
        end
        phase("No answer: check in a GCS", "bad")   -- never sent again by itself: it may have saved
        vals[name], pend = nil, nil
        return
      end
      pend.tries = pend.tries + 1
      if pend.tries >= tries() then
        if pend.check then
          phase("No answer: check in a GCS", "bad")
          vals[name], pend = nil, nil
          return
        end
        if not answered then   -- nothing ever came back on this link: say so, try again later
          silent = silent + 1
          pend = nil
          if silent >= 2 then quietAt = now end
          return
        end
        setVal(name, false, nil)
        pend = nil
      else
        pend.at = nil
        send(REQ_READ, name)
      end
      return
    end
    if saveReq then
      pend = { name = saveReq.name, set = saveReq.bytes, tries = 0 }
      send(PARAM_SET, saveReq.name, saveReq.b)
      saveReq = nil
      return
    end
    local n = nextRead()
    if n then
      readReq(n)
      -- mLRS: each answer waits behind the telemetry stream, so one more unread row is asked at once
      if link == "MAVLINK" and rows then
        for _, r in ipairs(rows) do
          local nm = r[1]
          if nm ~= n and vals[nm] == nil and now - (prefAt[nm] or -10) >= 4 then
            prefAt[nm] = now
            send(REQ_READ, nm)
            break
          end
        end
      end
    end
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
    if not C.armed then return false end
    return not (C.airEdits and r and (AIR[tab] or r.fl))
  end
  function C.editing() return edit, draft, base end

  -- open row i for editing: false (+ phase) when it can't be (not read yet / n/a: asked again)
  function C.beginEdit(i)
    local r = rows and rows[i]
    if not r then return false end
    local v = vals[r[1]]
    if v == nil then phase("Reading...") return false end
    if v == false then vals[r[1]], txt[r[1]] = nil, nil phase("Asking again...") return false end
    edit, draft, base = r, v, v
    phase("")
    if C.locked(r) then phase("Armed: view only", "bad")
    elseif C.armed then phase("In flight: saves at once", "bad") end
    if not pend and #out == 0 then readReq(r[1]) end   -- read again: may have changed in a GCS
    return true
  end
  function C.endEdit() edit = nil phase("") end

  function C.nudge(d)
    local r = edit
    if not r or C.locked(r) or r[9] == "bits" then return end
    local st = r[6] or 1
    local v = draft + d * st
    v = floor(v / st * 1000 + 0.5) * st / 1000
    -- a minimum of -1 (RTL_ALTITUDE, Q_ASSIST_SPEED...) is a setting of its own: below 0 goes to -1
    if r[4] == -1 then
      if d < 0 and draft <= 0 then v = -1 elseif d > 0 and draft < 0 then v = 0 end
    end
    if r[4] then v = math.max(r[4], v) end
    if r[5] then v = math.min(r[5], v) end
    draft = v
    phase("")
  end
  function C.toggleBit(i)
    if not edit or C.locked(edit) then return end
    local bit = BITS[i][1]
    local v = draft
    if v < 0 then v = ALL_MASK end
    if floor(v / 2 ^ bit) % 2 == 1 then v = v - 2 ^ bit else v = v + 2 ^ bit end
    draft = v
    phase("")
  end
  function C.setDraft(v) if edit and not C.locked(edit) then draft = v phase("") end end
  -- returns "closed" (unchanged), "saving", or nil (refused, see C.phase)
  function C.save()
    if not edit then return nil end
    if C.locked(edit) then phase("Disarm to save", "bad") return nil end
    if pend and pend.set then return nil end
    local b = f2b(draft)
    local bytes = string.char(b[1], b[2], b[3], b[4])
    if bytes == vbs[edit[1]] then edit = nil phase("") return "closed" end
    saveReq = { name = edit[1], b = b, bytes = bytes }
    phase("Saving...")
    return "saving"
  end
  function C.saving() return saveReq ~= nil or (pend ~= nil and (pend.set ~= nil or pend.check ~= nil)) end

  -- one line for the "No ... telemetry" screen: frames since the tool opened, last passthrough
  function C.diagText()
    local D, now = C.diag, clock()
    local function ago(t) return t < 0 and "-" or string.format("%.0fs", now - t) end
    return string.format("CRSF %d (AP %d, native %d) last AP %s  S.Port %d  mLRS %d",
      D.cr, D.crPt, D.crNat, ago(seenCr), D.sp, D.env)
  end
  function C.debug()
    return string.format("%s tx%d rx%d bad%d q%d %s", tostring(link), C.stats.tx, C.stats.rx, C.stats.bad, #out,
      pend and pend.name or "-")
  end
  C._mav = MV
  C._HB = HB
  return C
end
