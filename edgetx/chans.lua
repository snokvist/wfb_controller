------------------------------------------------------------
--  Streams CH1‑CH16 @1 Hz **only when SF = ON**
--  Listens anytime for
--      GV,<idx>,<val>
--      GV,<fm>,<idx>,<val>
--  Replies: OK / ERR
--
--  Monitors in flight‑mode 0:
--      GVAR 1 – sends "SET,WIFI,<val>"   (idle = 0)
--      GVAR 2 – sends "EXEC,<val>"       (idle = 0)
--  Values are debounced so the same <val> is not retransmitted
--  unless the GVAR has first been cleared back to 0.
------------------------------------------------------------
local BAUD         = 115200
local PERIOD_TICKS = 100          -- 100 × 10 ms ≈ 1 s  (CH dump)

-- ✦ original special‑GVAR watchdog -----------------------
local SPECIAL_FM       = 0        -- flight‑mode 0
local SPECIAL_IDX      = 0        -- GVAR 0
local SPECIAL_TIMEOUT  = 100      -- 1 000 ms = 100 ticks

-- ✦ command‑GVAR monitor ---------------------------------
local MONITOR_FM       = 0        -- look in FM 0
local SET_IDX          = 1        -- GVAR 1  (SET)
local CMD_IDX          = 2        -- GVAR 2  (EXEC)
local CMD_PERIOD_TICKS = 50       -- 500 ms polling

------------------------------------------------------------
local nextChTime   = 0
local nextCmdTime  = 0
local lastSpecialUpdate = 0
local rxBuf        = ""

-- debounce memories
local lastSetSent, lastCmdSent = 0, 0

-- New API on EdgeTX ≥ 2.8
local haveOutValue = type(getOutputValue) == "function"

------------------------------------------------------------
-- Output helpers
local function ch(i)                           -- 1‑based
  if haveOutValue then
    return getOutputValue(i - 1) or 0          -- fast path
  else
    local ok, v = pcall(getValue, "ch" .. i)   -- legacy path
    return (ok and v) or 0
  end
end

local function sendChannels()
  local line = "CH"
  for i = 1, 16 do
    line = line .. "," .. ch(i)
  end
  serialWrite(line .. "\n")
end

-- ✦ watchdog helper --------------------------------------
local function touchSpecial(fm, idx)
  if fm == SPECIAL_FM and idx == SPECIAL_IDX then
    lastSpecialUpdate = getTime()
  end
end
------------------------------------------------------------

local function setGvar(fm, idx, val)
  if fm < 0 or fm > 8 or idx < 0 or idx > 15 or val < -1024 or val > 1024 then
    serialWrite("ERR\n")
  else
    model.setGlobalVariable(idx, fm, val)
    touchSpecial(fm, idx)
    serialWrite("OK\n")
  end
end

------------------------------------------------------------
-- Parser (ignore everything except GV commands)
local function handleLine(line)
  if line == "" then return end
  line = string.upper(line)

  -- only process commands that begin with “GV,”
  if not string.match(line, "^GV,") then return end

  -- GV,<idx>,<val>  (FM0)
  local idx, val = string.match(line, "^GV,(%d+),([%-]?%d+)$")
  if idx then
    setGvar(0, tonumber(idx), tonumber(val)); return end

  -- GV,<fm>,<idx>,<val>
  local fm, i2, v2 = string.match(line, "^GV,(%d+),(%d+),([%-]?%d+)$")
  if fm then
    setGvar(tonumber(fm), tonumber(i2), tonumber(v2)); return end

  -- If we reach here, the GV line was malformed
  serialWrite("ERR\n")
end

------------------------------------------------------------
-- Special‑GVAR enforcement  -------------------------------
local function enforceSpecial()
  local now = getTime()
  if now - lastSpecialUpdate > SPECIAL_TIMEOUT then
    if model.getGlobalVariable(SPECIAL_IDX, SPECIAL_FM) ~= -1024 then
      model.setGlobalVariable(SPECIAL_IDX, SPECIAL_FM, -1024)
    end
  end
end
------------------------------------------------------------

-- Command‑GVAR polling / debounce -------------------------
local function monitorCommands()
  local now = getTime()
  if now < nextCmdTime then return end
  nextCmdTime = now + CMD_PERIOD_TICKS

  -- GVAR 1  -> SET
  local setVal = model.getGlobalVariable(SET_IDX, MONITOR_FM)
  if setVal == 0 then
    lastSetSent = 0                           -- reset debounce
  elseif setVal ~= lastSetSent then
    serialWrite(string.format("SET,WIFI,%d\n", setVal))
    lastSetSent = setVal
    model.setGlobalVariable(SET_IDX, MONITOR_FM, 0)
  end

  -- GVAR 2 -> EXEC
  local cmdVal = model.getGlobalVariable(CMD_IDX, MONITOR_FM)
  if cmdVal == 0 then
    lastCmdSent = 0
  elseif cmdVal ~= lastCmdSent then
    serialWrite(string.format("EXEC,%d\n", cmdVal))
    lastCmdSent = cmdVal
    model.setGlobalVariable(CMD_IDX, MONITOR_FM, 0)
  end
end
------------------------------------------------------------

local function processSerial()
  while true do
    local pos = string.find(rxBuf, "[\r\n]")
    if not pos then break end
    local line = string.sub(rxBuf, 1, pos - 1)
    repeat                                    -- drop CR/LF pair(s)
      rxBuf = string.sub(rxBuf, pos + 1)
      pos   = string.find(rxBuf, "^[\r\n]")
    until not pos
    handleLine(line)
  end
end

------------------------------------------------------------
-- EdgeTX entry points
local function init()
  setSerialBaudrate(BAUD)
  local now = getTime()
  nextChTime     = now + PERIOD_TICKS
  nextCmdTime    = now + CMD_PERIOD_TICKS
  lastSpecialUpdate = now
end

local function rxLoop()
  local fresh = serialRead()
  if fresh ~= "" then rxBuf = rxBuf .. fresh end
  if rxBuf ~= "" then processSerial() end
  enforceSpecial()
  monitorCommands()
end

-- Special Function ON
local function run()
  rxLoop()
  local now = getTime()
  if now >= nextChTime then
    sendChannels()
    nextChTime = now + PERIOD_TICKS
  end
end

-- Special Function OFF
local function background()
  rxLoop()
end

return { init = init, run = run, background = background }
