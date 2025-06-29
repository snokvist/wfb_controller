------------------------------------------------------------
--  GV_PUSH.lua  – 128×64 LCD (RadioMaster Boxer)
--
--  Wheel L/R  or +/– :                      ┐
--      • if on a value field  → change 1–200│
--      • if on a [SEND] button → jump line ─┘
--
--  ENTER short : toggle Value ↔ SEND on the CURRENT line
--                • on [SEND] it also writes the GVAR
--
--  ENTER long  : ignored (blocks “Reset” menu)
--
--  GVAR1 (FM0) = WiFi / SET
--  GVAR2 (FM0) = Command / EXEC
------------------------------------------------------------
local FM          = 0
local WIFI_IDX    = 1   -- GVAR1
local CMD_IDX     = 2   -- GVAR2
local MIN_VAL     = 1
local MAX_VAL     = 200

------------------------------------------------------------
local wifiVal, cmdVal = 1, 1   -- initial values
-- cursor: 0 WiFiVal , 1 WiFiSend , 2 CmdVal , 3 CmdSend
local cursor = 0

------------------------------------------------------------
local function clamp(v)
  if v > MAX_VAL then return MAX_VAL
  elseif v < MIN_VAL then return MIN_VAL
  else return v end
end
local function inc(v) return clamp(v + 1) end
local function dec(v) return clamp(v - 1) end

------------------------------------------------------------
local function draw()
  lcd.clear()

  local VAL_X        = 95
  local WIFI_Y       = 4
  local WIFI_BTN_Y   = WIFI_Y + 12
  local CMD_Y        = 28
  local CMD_BTN_Y    = CMD_Y + 12

  -- labels
  lcd.drawText(0,  WIFI_Y,    "WiFi ch:", SMLSIZE)
  lcd.drawText(0,  CMD_Y,     "Command:", SMLSIZE)

  -- values
  lcd.drawNumber(VAL_X, WIFI_Y, wifiVal,
                 (cursor==0) and (INVERS+SMLSIZE) or SMLSIZE)
  lcd.drawNumber(VAL_X, CMD_Y,  cmdVal,
                 (cursor==2) and (INVERS+SMLSIZE) or SMLSIZE)

  -- SEND buttons
  lcd.drawText(40, WIFI_BTN_Y, "[SEND]",
               (cursor==1) and INVERS or 0)
  lcd.drawText(40, CMD_BTN_Y,  "[SEND]",
               (cursor==3) and INVERS or 0)

  lcd.drawText(0, 54, "ENTER: toggle / send", SMLSIZE)
end

------------------------------------------------------------
local function pushGvar()
  if cursor == 1 then
    model.setGlobalVariable(WIFI_IDX, FM, wifiVal)
  elseif cursor == 3 then
    model.setGlobalVariable(CMD_IDX,  FM, cmdVal)
  end
end

------------------------------------------------------------
-- Jump from a SEND button to the other line's value field
local function hopLine()
  if cursor == 1 then cursor = 2 else cursor = 0 end
end

------------------------------------------------------------
local function run(event)
  ----------------------------------------------------------
  --  Wheel & +/- keys
  ----------------------------------------------------------
  local wheelFwd  = (event == EVT_ROT_RIGHT)
                 or (event == EVT_PLUS_FIRST)
                 or (event == EVT_PLUS_REPEAT)

  local wheelBack = (event == EVT_ROT_LEFT)
                 or (event == EVT_MINUS_FIRST)
                 or (event == EVT_MINUS_REPEAT)

  if wheelFwd or wheelBack then
    if cursor == 0 then
      wifiVal = wheelFwd and inc(wifiVal) or dec(wifiVal)
    elseif cursor == 2 then
      cmdVal  = wheelFwd and inc(cmdVal)  or dec(cmdVal)
    else  -- on a SEND button → hop to other line’s value
      hopLine()
    end
  ----------------------------------------------------------
  --  ENTER short  (toggle / send)
  ----------------------------------------------------------
  elseif event == EVT_ENTER_BREAK then
    if cursor % 2 == 0 then         -- on a value field
      cursor = cursor + 1           -- highlight SEND
    else                            -- on a SEND button
      pushGvar()                    -- transmit
      cursor = cursor - 1           -- back to value
    end
  ----------------------------------------------------------
  --  ENTER long  (block “Reset” menu)
  ----------------------------------------------------------
  elseif event == EVT_ENTER_LONG or event == EVT_LONG_BREAK then
    killEvents(event)
  end

  draw()
  return 0
end

------------------------------------------------------------
return { run = run }
