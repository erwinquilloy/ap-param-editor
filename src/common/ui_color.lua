--[[
  AP Param Editor, colour screen UI (EdgeTX colour radios and Ethos share this file byte for byte).
  Any screen size: everything is laid out from the window's width and height.

  The platform gives a drawing adapter g:
    g.W, g.H                    window size
    g.rect(x, y, w, h, col)     filled rectangle
    g.frame(x, y, w, h, col)    outline
    g.text(x, y, s, font, col, align)   font "s" / "m" / "l", align nil (left) / "r" / "c"
    g.textW(s, font)            text width in pixels
    g.fontH(font)               line height in pixels
    g.rgb(r, gr, b)             a colour value
  and calls
    U.key(k)       k = "next", "prev", "enter", "back"; returns false when RTN on the tab list
                   (or on a screen the editor can't work from) should leave the tool
    U.tap(x, y)    a touch: same rule
    U.slide(dy)    a vertical drag in pixels (scrolls lists)
    U.paint(g)
]]
local floor = math.floor

return function(C, title)
  local U = {}
  local view = "tabs"          -- "tabs", "rows", "edit"
  local tabSel, sel, top = 1, 1, 1
  local bitCur, bitTop = 1, 1
  local boxes = {}             -- touch areas of the last paint { x, y, w, h, act }
  local rowsN, rowH = 8, 30
  local col = nil
  local dragAcc = 0

  local function colours(g)
    col = {
      bg = g.rgb(16, 20, 26), head = g.rgb(28, 74, 120), headTx = g.rgb(255, 255, 255),
      tx = g.rgb(230, 234, 240), dim = g.rgb(140, 150, 165), sel = g.rgb(44, 100, 160),
      line = g.rgb(44, 50, 60), ok = g.rgb(70, 200, 110), bad = g.rgb(240, 90, 80),
      warn = g.rgb(245, 190, 60), btn = g.rgb(52, 60, 74), btnTx = g.rgb(255, 255, 255),
      save = g.rgb(30, 130, 70),
    }
  end

  local function box(x, y, w, h, act) boxes[#boxes + 1] = { x, y, w, h, act } end

  -- the list of the tab view: tabs, then the telemetry source and the Air edits switch
  local function tabItems()
    local t = C.tabs()
    local items = {}
    for i = 1, #t do items[i] = t[i] end
    items[#items + 1] = "@src"
    items[#items + 1] = "@air"
    return items
  end
  local function srcText()
    local t = C.SOURCE_NAME[C.source]
    if C.source == "AUTO" and C.link then
      t = t .. " (" .. ((C.link == "SPORT" and "S.Port") or (C.link == "MAVLINK" and "mLRS") or "CRSF") .. ")"
    end
    return t
  end

  local function clampSel(n)
    if sel > n then sel = n end
    if sel < 1 then sel = 1 end
  end

  local function openTab(g)
    C.open(g)
    view, sel, top = "rows", 1, 1
    C.select(1)
  end

  local function openRow(i)
    sel = i
    C.select(i)
    if C.beginEdit(i) then view, bitCur, bitTop = "edit", 1, 1 end
  end

  local function leaveEdit()
    C.endEdit()
    view = "rows"
  end

  -- ---------------------------------------------------------------- keys
  function U.key(k)
    if C.blocked then
      if view == "edit" then C.endEdit() view = "rows" end
      if k == "enter" then C.nextSource() end   -- the screen says which source is waited for
      return k ~= "back"
    end
    local d = (k == "next" and 1) or (k == "prev" and -1) or 0
    if view == "tabs" then
      local items = tabItems()
      if d ~= 0 then tabSel = math.max(1, math.min(#items, tabSel + d)) end
      if k == "enter" then
        local it = items[tabSel]
        if it == "@air" then C.airEdits = not C.airEdits
        elseif it == "@src" then C.nextSource()
        elseif it then openTab(it) end
      elseif k == "back" then return false end
    elseif view == "rows" then
      local rows = C.rows() or {}
      if d ~= 0 then sel = math.max(1, math.min(#rows, sel + d)) C.select(sel) end
      if k == "enter" then openRow(sel)
      elseif k == "back" then view = "tabs" C.closeTab() end
    else
      local r = C.editing()
      if not r then view = "rows" return true end
      if r[9] == "bits" then
        local n = #C.BITS + 1
        if d ~= 0 then bitCur = math.max(1, math.min(n, bitCur + d)) end
        if k == "enter" then
          if bitCur <= #C.BITS then C.toggleBit(bitCur)
          elseif C.save() == "closed" then view = "rows" end
        end
      else
        if d ~= 0 then C.nudge(d) end
        if k == "enter" and C.save() == "closed" then view = "rows" end
      end
      if k == "back" then leaveEdit() end
    end
    return true
  end

  function U.tap(x, y)
    for i = #boxes, 1, -1 do
      local b = boxes[i]
      if x >= b[1] and x < b[1] + b[3] and y >= b[2] and y < b[2] + b[4] then
        local a = b[5]
        if type(a) == "function" then return a() ~= false end
        return U.key(a)
      end
    end
    return true
  end

  function U.slide(dy)
    dragAcc = dragAcc + dy
    local steps = floor(math.abs(dragAcc) / rowH)
    if steps == 0 then return end
    local s = dragAcc < 0 and 1 or -1      -- finger up: the list moves up, later rows come in
    dragAcc = dragAcc + s * steps * rowH
    if view == "rows" then
      local n = #(C.rows() or {})
      top = math.max(1, math.min(math.max(1, n - rowsN + 1), top + s * steps))
      if sel < top then sel = top elseif sel > top + rowsN - 1 then sel = top + rowsN - 1 end
      C.select(sel)
    elseif view == "tabs" then
      local n = #tabItems()
      tabSel = math.max(1, math.min(n, tabSel + s * steps))
    elseif view == "edit" then
      local r = C.editing()
      if r and r[9] == "bits" then bitCur = math.max(1, math.min(#C.BITS + 1, bitCur + s * steps)) end
    end
  end

  -- ---------------------------------------------------------------- drawing
  local function header(g, hH, left, right, back)
    g.rect(0, 0, g.W, hH, col.head)
    local fh = g.fontH("m")
    local x = 8
    if back then
      g.text(8, floor((hH - fh) / 2), "<", "m", col.headTx)
      x = 8 + g.textW("< ", "m")
      box(0, 0, math.max(hH * 2, x + 20), hH, "back")
    end
    g.text(x, floor((hH - fh) / 2), left, "m", col.headTx)
    if right then g.text(g.W - 8, floor((hH - g.fontH("s")) / 2), right, "s", col.headTx, "r") end
  end

  local function fit(g, s, font, w)
    if g.textW(s, font) <= w then return s end
    while #s > 1 and g.textW(s .. "..", font) > w do s = string.sub(s, 1, #s - 1) end
    return s .. ".."
  end

  local function linkText()
    local l = C.link
    local n = (l == "SPORT" and "S.Port") or (l == "MAVLINK" and "mLRS") or (l == "CRSF" and "CRSF") or "-"
    if C.armed then n = n .. "  ARMED" end
    return n
  end

  local function listRows(g, y0, y1)
    rowH = math.max(g.fontH("m") + 10, floor((y1 - y0) / 9))
    rowsN = math.max(1, floor((y1 - y0) / rowH))
    return rowH, rowsN
  end

  local function scrollbar(g, y0, h, first, shown, n)
    if n <= shown then return end
    local bh = math.max(12, floor(h * shown / n))
    local by = y0 + floor((h - bh) * (first - 1) / math.max(1, n - shown))
    g.rect(g.W - 4, by, 3, bh, col.dim)
  end

  local function drawTabs(g, hH)
    header(g, hH, title, linkText(), false)
    local items = tabItems()
    local y0, y1 = hH + 4, g.H - 4
    local rh, n = listRows(g, y0, y1)
    local first = math.max(1, math.min(tabSel - floor(n / 2), #items - n + 1))
    for k = 0, n - 1 do
      local i = first + k
      local it = items[i]
      if not it then break end
      local y = y0 + k * rh
      if i == tabSel then g.rect(0, y, g.W, rh - 2, col.sel) end
      local ty = y + floor((rh - 2 - g.fontH("m")) / 2)
      if it == "@src" then
        g.text(12, ty, "Telemetry", "m", col.tx)
        g.text(g.W - 12, ty, fit(g, srcText(), "m", g.W - 40 - g.textW("Telemetry", "m")), "m", col.warn, "r")
        box(0, y, g.W, rh, function() tabSel = i C.nextSource() end)
      elseif it == "@air" then
        g.text(12, ty, "Air edits (PID, trim while armed)", "m", col.tx)
        g.text(g.W - 12, ty, C.airEdits and "ON" or "OFF", "m", C.airEdits and col.warn or col.dim, "r")
        box(0, y, g.W, rh, function() tabSel = i C.airEdits = not C.airEdits end)
      else
        g.text(12, ty, it, "m", col.tx)
        g.text(g.W - 12, ty, ">", "m", col.dim, "r")
        box(0, y, g.W, rh, function() tabSel = i openTab(it) end)
      end
      g.rect(0, y + rh - 2, g.W, 1, col.line)
    end
    scrollbar(g, y0, y1 - y0, first, n, #items)
    if C.customInfo then g.text(12, g.H - g.fontH("s") - 2, C.customInfo, "s", col.bad) end
  end

  local function drawRows(g, hH)
    local rows = C.rows() or {}
    local got, n = C.progress()
    header(g, hH, C.tab() or "", got < n and (got .. "/" .. n .. "  " .. linkText()) or linkText(), true)
    local y0, y1 = hH + 4, g.H - 4
    local rh, rn = listRows(g, y0, y1)
    clampSel(#rows)
    if sel < top then top = sel elseif sel > top + rn - 1 then top = sel - rn + 1 end
    if top > math.max(1, #rows - rn + 1) then top = math.max(1, #rows - rn + 1) end
    C.view(top, rn)
    local wide = g.W >= 600
    for k = 0, rn - 1 do
      local i = top + k
      local r = rows[i]
      if not r then break end
      local y = y0 + k * rh
      if i == sel then g.rect(0, y, g.W, rh - 2, col.sel) end
      local v = C.value(r[1])
      local vt = C.text(r)
      local vc = (v == nil and col.dim) or (v == false and col.bad) or col.tx
      local vw = g.textW(vt, "m")
      local ty = y + floor((rh - 2 - g.fontH("m")) / 2)
      local lw = g.W - 30 - vw
      local label = r[2]
      if wide and r[2] ~= r[1] then
        -- the parameter's own name in a column of its own (wide screens)
        local nx = floor(g.W * 0.42)
        g.text(12, ty, fit(g, label, "m", nx - 20), "m", col.tx)
        g.text(nx, ty + g.fontH("m") - g.fontH("s"), fit(g, r[1], "s", math.max(20, g.W - 24 - vw - nx)), "s", col.dim)
      else
        g.text(12, ty, fit(g, label, "m", lw), "m", col.tx)
      end
      g.text(g.W - 12, ty, vt, "m", vc, "r")
      g.rect(0, y + rh - 2, g.W, 1, col.line)
      box(0, y, g.W, rh, function() if sel == i then openRow(i) else sel = i C.select(i) end end)
    end
    scrollbar(g, y0, y1 - y0, top, rn, #rows)
  end

  local function button(g, x, y, w, h, label, act, c)
    g.rect(x, y, w, h, c or col.btn)
    g.text(x + floor(w / 2), y + floor((h - g.fontH("m")) / 2), label, "m", col.btnTx, "c")
    box(x, y, w, h, act)
  end

  local function phaseLine(g, y)
    local p = C.phase
    if p == "" then return end
    local c = (C.phaseKind == "ok" and col.ok) or (C.phaseKind == "bad" and col.bad) or col.warn
    g.text(12, y, fit(g, p, "m", g.W - 24), "m", c)
  end

  local function drawBits(g, hH, r, draft)
    header(g, hH, r[2], C.skipCount(draft) .. " skipped", true)
    local bH = math.max(g.fontH("m") + 14, floor(g.H / 8))
    local y0, y1 = hH + 4, g.H - bH - 8 - g.fontH("m")
    local rh, rn = listRows(g, y0, y1)
    local n = #C.BITS
    if bitCur <= n then
      if bitCur < bitTop then bitTop = bitCur elseif bitCur > bitTop + rn - 1 then bitTop = bitCur - rn + 1 end
    end
    local locked = C.locked(r)
    for k = 0, rn - 1 do
      local i = bitTop + k
      if i > n then break end
      local y = y0 + k * rh
      if i == bitCur then g.rect(0, y, g.W, rh - 2, col.sel) end
      local ty = y + floor((rh - 2 - g.fontH("m")) / 2)
      local sk = C.skipped(draft, C.BITS[i][1])
      g.text(12, ty, C.BITS[i][2], "m", col.tx)
      g.text(g.W - 12, ty, sk and "SKIP" or "check", "m", sk and col.warn or col.ok, "r")
      g.rect(0, y + rh - 2, g.W, 1, col.line)
      box(0, y, g.W, rh, function() bitCur = i if not locked then C.toggleBit(i) end end)
    end
    scrollbar(g, y0, y1 - y0, bitTop, rn, n)
    phaseLine(g, g.H - bH - 6 - g.fontH("m"))
    local by, bw = g.H - bH - 4, floor((g.W - 24) / 2)
    button(g, 8, by, bw, bH, "Back", "back")
    if not locked then
      local hl = bitCur == n + 1
      button(g, 16 + bw, by, bw, bH, hl and "> Save <" or "Save", function()
        bitCur = n + 1
        if C.save() == "closed" then view = "rows" end
      end, col.save)
    end
  end

  local function drawEdit(g, hH)
    local r, draft, base = C.editing()
    if not r then view = "rows" return drawRows(g, hH) end
    if r[9] == "bits" then return drawBits(g, hH, r, draft) end
    local locked = C.locked(r)
    header(g, hH, r[2] .. (draft ~= base and " *" or ""), locked and "view only" or nil, true)
    local y = hH + 8
    g.text(12, y, r[1], "s", col.dim)
    y = y + g.fontH("s") + 6
    local t = C.fmt(r, draft)
    local big = g.textW(t, "l") <= g.W - 24 and "l" or "m"
    g.text(floor(g.W / 2), y, t, big, draft ~= base and col.warn or col.tx, "c")
    y = y + g.fontH(big) + 8
    local range
    if r[8] then range = "wheel / buttons: choose"
    elseif r[4] and r[5] then
      local f = "%." .. (r[7] or 0) .. "f"
      range = string.format(f, r[4]) .. " .. " .. string.format(f, r[5]) .. "   step " .. tostring(r[6])
    else range = "step " .. tostring(r[6]) end
    g.text(floor(g.W / 2), y, range, "s", col.dim, "c")
    y = y + g.fontH("s") + 6
    phaseLine(g, y)
    -- buttons: -10 - + +10 on one row, Back / Save under them
    local bH = math.max(g.fontH("m") + 14, floor(g.H / 8))
    local gap = 8
    local by2 = g.H - bH - 4
    local by1 = by2 - bH - gap
    local bw4 = floor((g.W - gap * 5) / 4)
    if not locked then
      local opts = r[8] ~= nil
      button(g, gap, by1, bw4, bH, opts and "<<" or "-10", function() for _ = 1, 10 do C.nudge(-1) end end)
      button(g, gap * 2 + bw4, by1, bw4, bH, opts and "<" or "-", function() C.nudge(-1) end)
      button(g, gap * 3 + bw4 * 2, by1, bw4, bH, opts and ">" or "+", function() C.nudge(1) end)
      button(g, gap * 4 + bw4 * 3, by1, bw4, bH, opts and ">>" or "+10", function() for _ = 1, 10 do C.nudge(1) end end)
    end
    local bw2 = floor((g.W - gap * 3) / 2)
    button(g, gap, by2, bw2, bH, "Back", "back")
    if not locked then
      button(g, gap * 2 + bw2, by2, bw2, bH, C.saving() and "Saving..." or "Save", function()
        if C.save() == "closed" then view = "rows" end
      end, col.save)
    end
  end

  local function drawBlocked(g, hH)
    header(g, hH, title, linkText(), false)
    local lines = C.blocked
    local y = hH + 14
    for i = 1, #lines do
      g.text(14, y, lines[i], i == 1 and "l" or "m", i == 1 and col.warn or col.tx)
      y = y + g.fontH(i == 1 and "l" or "m") + 8
    end
    local bH = math.max(g.fontH("m") + 14, floor(g.H / 8))
    button(g, 8, g.H - bH - 6, g.W - 16, bH, "Telemetry: " .. srcText(), function() C.nextSource() end)
    g.text(14, g.H - bH - 12 - g.fontH("s"), "ENTER / tap: change source    RTN: exit", "s", col.dim)
  end

  function U.paint(g)
    if not col then colours(g) end
    boxes = {}
    g.rect(0, 0, g.W, g.H, col.bg)
    local hH = math.max(g.fontH("m") + 10, floor(g.H / 10))
    if C.blocked then
      if view == "edit" then C.endEdit() view = "rows" end
      drawBlocked(g, hH)
    elseif view == "tabs" then drawTabs(g, hH)
    elseif view == "rows" then drawRows(g, hH)
    else drawEdit(g, hH) end
  end

  function U.view() return view end
  return U
end
