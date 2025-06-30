------------------------------------------------------------
--  GV_PUSH.lua  – 128×64 LCD (RadioMaster Boxer)
--
--  Wheel L/R  or +/– :
--      • on a value field   → cycle through the curated list
--      • on a [SEND] button → hop to the other line’s value
--
--  ENTER short :
--      • on a value field   → highlight [SEND]
--      • on [SEND]          → write GVAR & return to value
--
--  ENTER long  : ignored (blocks “Reset” menu)
--
--  GVAR1 (FM0) = WiFi / SET   – sends selected channel number
--  GVAR2 (FM0) = Command / EXEC – sends command index
------------------------------------------------------------
local FM          = 0
local WIFI_IDX    = 1   -- GVAR1
local CMD_IDX     = 2   -- GVAR2

-- curated lists -----------------------------------------------------------
local wifiList = {44, 48, 104, 128, 140, 149, 157, 161}

local cmdList = {
  {val = 1, label = "Reboot"},
  {val = 2, label = "Shutdown"},
  {val = 3, label = "Update"},
  {val = 4, label = "FactoryRST"},
  {val = 5, label = "Start"},
  {val = 6, label = "Stop"},
}
---------------------------------------------------------------------------

-- indices into the lists (1-based Lua style)
local wifiIdx, cmdIdx = 1, 1
-- cursor: 0 WiFiVal , 1 WiFiSend , 2 CmdVal , 3 CmdSend
local cursor = 0

-- helpers -----------------------------------------------------------------
local function incIdx(idx, n) return (idx % n) + 1 end        -- wrap 1…n
local function decIdx(idx, n) return (idx - 2) % n + 1 end

local function draw()
  lcd.clear()

  -- layout constants
  local WIFI_Y      = 4
  local WIFI_BTN_Y  = WIFI_Y + 12
  local CMD_Y       = 28
  local CMD_BTN_Y   = CMD_Y  + 12
  local VAL_X_WIFI  = 95                    -- right-aligned 3-digit number
  local VAL_X_CMD   = 70                    -- room for “idx label”

  -- labels
  lcd.drawText(0, WIFI_Y, "WiFi ch:", SMLSIZE)
  lcd.drawText(0,  CMD_Y, "Command:", SMLSIZE)

  -- Wi-Fi value
  lcd.drawNumber(VAL_X_WIFI, WIFI_Y, wifiList[wifiIdx],
                 (cursor==0) and (INVERS+SMLSIZE) or SMLSIZE)

  -- Command value (idx + label)
  local cmdStr = cmdList[cmdIdx].val .. " " .. cmdList[cmdIdx].label
  lcd.drawText(VAL_X_CMD, CMD_Y, cmdStr,
               (cursor==2) and INVERS or 0)

  -- SEND buttons
  lcd.drawText(40, WIFI_BTN_Y, "[SEND]",
               (cursor==1) and INVERS or 0)
  lcd.drawText(40, CMD_BTN_Y,  "[SEND]",
               (cursor==3) and INVERS or 0)

  lcd.drawText(0, 54, "ENTER toggle / send", SMLSIZE)
end

-- send current selection to the proper GVAR ------------------------------
local function pushGvar()
  if cursor == 1 then
    model.setGlobalVariable(WIFI_IDX, FM, wifiList[wifiIdx])
  elseif cursor == 3 then
    model.setGlobalVariable(CMD_IDX,  FM, cmdList[cmdIdx].val)
  end
end

-- hop from a SEND button to the other line’s value field ------------------
local function hopLine()
  cursor = (cursor == 1) and 2 or 0
end

-- main event loop ---------------------------------------------------------
local function run(event)
  local wheelFwd  = (event == EVT_ROT_RIGHT)
                 or (event == EVT_PLUS_FIRST)
                 or (event == EVT_PLUS_REPEAT)

  local wheelBack = (event == EVT_ROT_LEFT)
                 or (event == EVT_MINUS_FIRST)
                 or (event == EVT_MINUS_REPEAT)

  --------------------------------------------------------------------------
  -- wheel / +/- -----------------------------------------------------------
  if wheelFwd or wheelBack then
    if cursor == 0 then                           -- Wi-Fi value
      wifiIdx = wheelFwd and incIdx(wifiIdx, #wifiList)
                          or decIdx(wifiIdx, #wifiList)

    elseif cursor == 2 then                       -- Command value
      cmdIdx  = wheelFwd and incIdx(cmdIdx,  #cmdList)
                          or decIdx(cmdIdx,  #cmdList)

    else                                          -- on a SEND button
      hopLine()                                   -- hop to other value
    end
  --------------------------------------------------------------------------
  -- ENTER short -----------------------------------------------------------
  elseif event == EVT_ENTER_BREAK then
    if cursor % 2 == 0 then                       -- on a value field
      cursor = cursor + 1                         -- highlight SEND
    else                                          -- on [SEND]
      pushGvar()                                  -- transmit
      cursor = cursor - 1                         -- back to value
    end
  --------------------------------------------------------------------------
  -- block ENTER long (Reset menu) -----------------------------------------
  elseif event == EVT_ENTER_LONG or event == EVT_LONG_BREAK then
    killEvents(event)
  end

  draw()
  return 0
end

return { run = run }
