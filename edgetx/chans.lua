------------------------------------------------------------
--  Streams CH1-CH16 @1 Hz **only when SF=ON**
--  Listens anytime for
--      GV,<idx>,<val>
--      GV,<fm>,<idx>,<val>
--  Replies: OK / ERR
------------------------------------------------------------
local BAUD         = 115200
local PERIOD_TICKS = 100          -- 100 × 10 ms ≈ 1 s

local nextTime     = 0            -- next CH dump deadline
local rxBuf        = ""           -- rolling RX buffer

-- New API on EdgeTX ≥2.8
local haveOutValue = type(getOutputValue) == "function"

------------------------------------------------------------
-- Output helpers
local function ch(i)                           -- 1-based
  if haveOutValue then
    return getOutputValue(i - 1) or 0          -- fast path
  else
    local ok, v = pcall(getValue, "ch" .. i)   -- legacy
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

local function setGvar(fm, idx, val)
  if fm < 0 or fm > 8 or idx < 0 or idx > 15 or val < -1024 or val > 1024 then
    serialWrite("ERR\n")
  else
    model.setGlobalVariable(idx, fm, val)
    serialWrite("OK\n")
  end
end

------------------------------------------------------------
-- Parser (ignores own outbound messages)
local function handleLine(line)
  if line == "" then return end
  local tag = string.sub(line, 1, 2)
  if tag == "CH" or tag == "OK" or tag == "ER" then return end -- skip self

  line = string.upper(line)

  -- GV,<idx>,<val> (FM0)
  local idx, val = string.match(line, "^GV,(%d+),([%-]?%d+)$")
  if idx then
    setGvar(0, tonumber(idx), tonumber(val)); return end

  -- GV,<fm>,<idx>,<val>
  local fm, i2, v2 = string.match(line, "^GV,(%d+),(%d+),([%-]?%d+)$")
  if fm then
    setGvar(tonumber(fm), tonumber(i2), tonumber(v2)); return end

  serialWrite("ERR\n")  -- unknown command
end

local function processSerial()
  while true do
    local pos = string.find(rxBuf, "[\r\n]")
    if not pos then break end
    local line = string.sub(rxBuf, 1, pos - 1)
    repeat                                    -- drop CR+LF pair(s)
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
  nextTime = getTime() + PERIOD_TICKS
end

-- Common RX drain + command processing
local function rxLoop()
  local fresh = serialRead()
  if fresh ~= "" then rxBuf = rxBuf .. fresh end
  if rxBuf ~= "" then processSerial() end
end

-- Called when Special Function is **ON**
local function run()
  rxLoop()
  local now = getTime()
  if now >= nextTime then
    sendChannels()
    nextTime = now + PERIOD_TICKS
  end
end

-- Called when SF is **OFF** – no TX, just keep buffer clean
local function background()
  rxLoop()
end

return { init = init, run = run, background = background }
