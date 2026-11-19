--[[
  AP Param Editor, black-and-white EdgeTX screens (128x64, 212x64). Same keys as the colour UI:
  U.key(k) with k = "next", "prev", "enter", "back" (false: leave the tool), U.paint().
  Wheel or +/- moves (in the editor: changes the value), ENTER opens / saves, RTN goes back.
]]
local floor = math.floor

return function(C, title)
  local U = {}
  local W = LCD_W or 128
  local CH = floor(W / 5.2)        -- characters per line in SMLSIZE (24 on 128, 40 on 212)
  local view = "tabs"
  local tabSel, sel, top, bitCur = 1, 1, 1, 1
  local up = string.upper

  local function items()
    local t = C.tabs()
    local l = {}
    for i = 1, #t do l[i] = t[i] end
    l[#l + 1] = "@src"
    l[#l + 1] = "@air"
    return l
  end
  local SHORT = { AUTO = "AUTO", SPORT = "S.PORT", CRSF = "CRSF", MAVLINK = "MLRS" }
  local function srcText()
    local t = SHORT[C.source]
    if C.source == "AUTO" and C.link then t = t .. " " .. SHORT[C.link] end
    return t
  end

  function U.key(k)
    if C.blocked then
      if view == "edit" then C.endEdit() view = "rows" end
      if k == "enter" then C.nextSource() end
      return k ~= "back"
    end
    local d = (k == "next" and 1) or (k == "prev" and -1) or 0
    if view == "tabs" then
      local l = items()
      if d ~= 0 then tabSel = math.max(1, math.min(#l, tabSel + d)) end
      if k == "enter" then
        local it = l[tabSel]
        if it == "@src" then C.nextSource()
        elseif it == "@air" then C.airEdits = not C.airEdits
        elseif it then C.open(it) view, sel, top = "rows", 1, 1 C.select(1) end
      elseif k == "back" then return false end
    elseif view == "rows" then
      local rows = C.rows() or {}
      if d ~= 0 then sel = math.max(1, math.min(#rows, sel + d)) C.select(sel) end
      if k == "enter" then
        if C.beginEdit(sel) then view, bitCur = "edit", 1 end
      elseif k == "back" then view = "tabs" C.closeTab() end
    else
      local r = C.editing()
      if not r then view = "rows" return true end
      if r[9] == "bits" then
        if d ~= 0 then bitCur = math.max(1, math.min(#C.BITS + 1, bitCur + d)) end
        if k == "enter" then
          if bitCur <= #C.BITS then C.toggleBit(bitCur)
          elseif C.save() == "closed" then view = "rows" end
        end
      else
        if d ~= 0 then C.nudge(d) end
        if k == "enter" and C.save() == "closed" then view = "rows" end
      end
      if k == "back" then C.endEdit() view = "rows" end
    end
    return true
  end

  local function bar(left, right)
    lcd.drawFilledRectangle(0, 0, W, 9, FORCE)
    lcd.drawText(1, 1, left, INVERS)
    if right then lcd.drawText(W - 1, 1, right, INVERS + RIGHT) end
  end

  local function line(y, left, right, hi)
    local f = SMLSIZE
    if hi then
      lcd.drawFilledRectangle(0, y, W, 7, FORCE)
      f = SMLSIZE + INVERS
    end
    if #left + #right > CH then left = string.sub(left, 1, math.max(4, CH - #right)) end
    lcd.drawText(1, y + 1, left, f)
    lcd.drawText(W - 1, y + 1, right, f + RIGHT)
  end

  local function drawTabs()
    bar(up(title), floor(collectgarbage("count")) .. "K")
    local l = items()
    local first = math.max(1, math.min(tabSel - 3, #l - 6))
    for k = 0, 6 do
      local i = first + k
      local it = l[i]
      if not it then break end
      local right = ">"
      local left = it
      if it == "@src" then left, right = "TELEMETRY", srcText()
      elseif it == "@air" then left, right = "AIR EDITS", C.airEdits and "ON" or "OFF" end
      line(10 + k * 7, left, right, i == tabSel)
    end
    if C.customInfo then lcd.drawText(1, 57, up(C.customInfo), SMLSIZE) end
  end

  local function drawRows()
    local rows = C.rows() or {}
    local got, n = C.progress()
    bar(C.tab() or "", got < n and (got .. "/" .. n) or SHORT[C.link or "AUTO"])
    if sel < top then top = sel elseif sel > top + 6 then top = sel - 6 end
    C.view(top, 7)
    for k = 0, 6 do
      local i = top + k
      local r = rows[i]
      if not r then break end
      line(10 + k * 7, up(r[2]), up(C.text(r)), i == sel)
    end
  end

  local function drawBits(r, draft)
    bar("SKIP CHECKS", C.skipCount(draft) .. " SKIP")
    local n = #C.BITS + 1
    local first = math.max(1, math.min(bitCur - 2, n - 5))
    for k = 0, 5 do
      local i = first + k
      if i > n then break end
      if i <= #C.BITS then
        line(10 + k * 7, up(C.BITS[i][2]), C.skipped(draft, C.BITS[i][1]) and "SKIP" or "ON", i == bitCur)
      else
        line(10 + k * 7, C.locked(r) and "VIEW ONLY" or "SAVE", "", i == bitCur)
      end
    end
    lcd.drawText(1, 57, C.phase ~= "" and up(C.phase) or "ENT TOGGLE  RTN BACK", SMLSIZE)
  end

  local function drawEdit()
    local r, draft, base = C.editing()
    if not r then view = "rows" return drawRows() end
    if r[9] == "bits" then return drawBits(r, draft) end
    bar(up(r[2]), draft ~= base and "*" or nil)
    lcd.drawText(1, 11, r[1], SMLSIZE)
    local t = up(C.fmt(r, draft))
    lcd.drawText(2, 20, t, #t <= floor(CH / 2.6) and MIDSIZE or 0)
    local range
    if r[8] then range = "WHEEL: CHOOSE"
    elseif r[4] and r[5] then
      local f = "%." .. (r[7] or 0) .. "f"
      range = string.format(f, r[4]) .. ".." .. string.format(f, r[5]) .. " STEP " .. r[6]
    else range = "STEP " .. r[6] end
    lcd.drawText(1, 38, range, SMLSIZE)
    lcd.drawText(1, 46, up(C.phase), SMLSIZE)
    lcd.drawText(1, 57, C.locked(r) and "VIEW ONLY  RTN BACK" or "ENT SAVE  RTN BACK", SMLSIZE)
  end

  local function drawBlocked()
    bar(up(title), nil)
    local l = C.blocked
    for i = 1, math.min(#l, 3) do lcd.drawText(1, 1 + i * 9, up(l[i]), SMLSIZE) end
    local D = C.diag
    lcd.drawText(1, 39, string.format("CR %d AP %d SP %d ML %d", D.cr, D.crPt, D.sp, D.env), SMLSIZE)
    lcd.drawText(1, 48, "SRC: " .. srcText(), SMLSIZE)
    lcd.drawText(1, 57, "ENT SOURCE  RTN EXIT", SMLSIZE)
  end

  function U.paint()
    lcd.clear()
    if C.blocked then
      if view == "edit" then C.endEdit() view = "rows" end
      drawBlocked()
    elseif view == "tabs" then drawTabs()
    elseif view == "rows" then drawRows()
    else drawEdit() end
  end
  return U
end
