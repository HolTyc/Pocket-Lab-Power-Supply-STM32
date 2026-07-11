/* Pocket Lab-Bench Power Supply - STM32L031G6U6 port
 *
 * Port of firmware/Pocket-Lab-Bench-Power-V1.ino (ATtiny3216) by
 * "Ben Makes Everything": https://www.youtube.com/watch?v=i2HRpcJS6Vk
 *
 * Differences from the original:
 *   - Display is a 0.91" 128x32 SSD1306 OLED on the shared I2C1 bus
 *     (was an HCMS-297X LED display on dedicated pins). The UI is a
 *     dashboard showing set/measured voltage, current and power at once.
 *   - The rotary encoder is decoded by the TIM2 hardware quadrature
 *     interface (PA8 = A, PA1 = B) instead of software polling.
 *   - The optional voltage memory uses the L031 built-in data EEPROM.
 *
 * All tunable settings (GPIO pins, addresses, limits) are in app_config.h.
 *
 * Safety Disclaimer: Lithium-Ion batteries can be dangerous. Use at your own risk.
 */

#include "app.h"

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include "app_config.h"
#include "ssd1306.h"

extern I2C_HandleTypeDef hi2c1;
extern TIM_HandleTypeDef htim2;

#define I2C_TIMEOUT_MS 50u

/* ----------------------------------------------------
 * TPS55289 registers
 * ---------------------------------------------------- */
#define REG_REF_LSB     0x00
#define REG_REF_MSB     0x01
#define REG_IOUT_LIMIT  0x02
#define REG_VOUT_SR     0x03
#define REG_VOUT_FS     0x04
#define REG_MODE        0x06
#define REG_STATUS      0x07

/* ----------------------------------------------------
 * INA226 registers and scaling
 * ---------------------------------------------------- */
#define INA_REG_SHUNT_VOLTAGE 0x01
#define INA_REG_BUS_VOLTAGE   0x02

#define SHUNT_LSB_VOLTS 0.0000025f /* 2.5 uV per bit */
#define BUS_LSB_VOLTS   0.00125f   /* 1.25 mV per bit */

/* ----------------------------------------------------
 * Data EEPROM storage for the optional remembered voltage
 * ----------------------------------------------------
 * One 32-bit word: magic in the upper half, millivolts in the lower half. */
#define EEPROM_VOLTAGE_ADDR   (DATA_EEPROM_BASE)
#define EEPROM_MAGIC_VALUE    0x5529u

/* ----------------------------------------------------
 * Application state
 * ---------------------------------------------------- */
static uint8_t tpsAddr = TPS_ADDR_1;
static bool tpsFound = false;
static bool oledOk = false;

static int appliedTargetMv = STARTUP_MV;
static int appliedCurrentLimitMa = DEFAULT_CURRENT_LIMIT_MA;

/* false = encoder adjusts voltage; true = encoder adjusts current limit */
static bool adjustingCurrentLimit = false;

/* Encoder */
static uint16_t lastEncCount = 0;

static bool lastButtonState = true;  /* true = released (HIGH) */
static bool previousStableButtonState = true;
static uint32_t lastButtonChangeMs = 0;

/* Display timing */
static uint32_t knobDisplayUntil = 0;
static uint32_t lastDashboardRefresh = 0;

/* Latched TPS55289 fault flags for debugging.
 * Reading REG_STATUS clears the TPS55289 fault bits, so save them here. */
static uint8_t latchedTpsFaults = 0;
static uint32_t lastTpsStatusCheckMs = 0;

static bool lastMainOutputState = false;

/* ----------------------------------------------------
 * Small helpers
 * ---------------------------------------------------- */
static int clampInt(int value, int lo, int hi)
{
  if (value < lo) return lo;
  if (value > hi) return hi;
  return value;
}

static bool mainOutputIsOn(void)
{
  return HAL_GPIO_ReadPin(SWITCH_STATE_GPIO_PORT, SWITCH_STATE_GPIO_PIN)
         == GPIO_PIN_RESET; /* HIGH = OFF, LOW = ON */
}

static bool encoderButtonIsPressed(void)
{
  return HAL_GPIO_ReadPin(ENC_SW_GPIO_PORT, ENC_SW_GPIO_PIN) == GPIO_PIN_RESET;
}

/* ----------------------------------------------------
 * Basic I2C helpers (addr = 7-bit address)
 * ---------------------------------------------------- */
static bool writeReg8(uint8_t addr, uint8_t reg, uint8_t value)
{
  return HAL_I2C_Mem_Write(&hi2c1, (uint16_t)(addr << 1), reg,
                           I2C_MEMADD_SIZE_8BIT, &value, 1,
                           I2C_TIMEOUT_MS) == HAL_OK;
}

static bool readReg8(uint8_t addr, uint8_t reg, uint8_t *value)
{
  return HAL_I2C_Mem_Read(&hi2c1, (uint16_t)(addr << 1), reg,
                          I2C_MEMADD_SIZE_8BIT, value, 1,
                          I2C_TIMEOUT_MS) == HAL_OK;
}

static bool readReg16(uint8_t addr, uint8_t reg, int16_t *value)
{
  uint8_t raw[2];

  if (HAL_I2C_Mem_Read(&hi2c1, (uint16_t)(addr << 1), reg,
                       I2C_MEMADD_SIZE_8BIT, raw, 2,
                       I2C_TIMEOUT_MS) != HAL_OK) {
    return false;
  }

  *value = (int16_t)(((uint16_t)raw[0] << 8) | raw[1]);
  return true;
}

/* ----------------------------------------------------
 * Number formatting (avoids float printf, not in newlib-nano by default)
 * ---------------------------------------------------- */

/* "5.02V" below 10 V, "12.3V" from 10 V up */
static void fmtVolts(char *buf, int n, float volts)
{
  if (volts < 0) {
    volts = 0;
  }

  if (volts < 9.995f) {
    int centi = (int)(volts * 100.0f + 0.5f);
    snprintf(buf, n, "%d.%02dV", centi / 100, centi % 100);
  } else {
    int deci = (int)(volts * 10.0f + 0.5f);
    snprintf(buf, n, "%d.%dV", deci / 10, deci % 10);
  }
}

/* Always one decimal: "16.4V" */
static void fmtVolts1dp(char *buf, int n, float volts)
{
  if (volts < 0) {
    volts = 0;
  }

  int deci = (int)(volts * 10.0f + 0.5f);
  snprintf(buf, n, "%d.%dV", deci / 10, deci % 10);
}

/* "250mA" below 1 A, "2.50A" from 1 A up. Takes the magnitude. */
static void fmtAmps(char *buf, int n, float amps)
{
  if (amps < 0) {
    amps = -amps;
  }

  if (amps < 0.9995f) {
    int ma = (int)(amps * 1000.0f + 0.5f);
    snprintf(buf, n, "%dmA", ma);
  } else {
    int centi = (int)(amps * 100.0f + 0.5f);
    snprintf(buf, n, "%d.%02dA", centi / 100, centi % 100);
  }
}

static void fmtWatts(char *buf, int n, float watts)
{
  if (watts < 0) {
    watts = -watts;
  }

  int deci = (int)(watts * 10.0f + 0.5f);
  snprintf(buf, n, "%d.%dW", deci / 10, deci % 10);
}

static void fmtSetMv(char *buf, int n, int millivolts)
{
  snprintf(buf, n, "%d.%dV", millivolts / 1000, (millivolts % 1000) / 100);
}

static void fmtLimitMa(char *buf, int n, int milliamps)
{
  snprintf(buf, n, "%d.%02dA", milliamps / 1000, (milliamps % 1000) / 10);
}

/* ----------------------------------------------------
 * OLED screens
 * ---------------------------------------------------- */
static void drawTextRight(int y, const char *str, int scale)
{
  ssd1306_draw_text(OLED_WIDTH - ssd1306_text_width(str, scale), y, str, scale);
}

static void drawTextCentered(int y, const char *str, int scale)
{
  ssd1306_draw_text((OLED_WIDTH - ssd1306_text_width(str, scale)) / 2, y,
                    str, scale);
}

static void oledShow(void)
{
  if (oledOk) {
    ssd1306_update();
  }
}

/* Full-screen message, e.g. "TPS?" or "SET ERR" */
static void showMessage(const char *msg)
{
  ssd1306_clear();
  drawTextCentered(9, msg, 2);
  oledShow();
}

/* Startup loading bar, filledBlocks = 0..8 */
static void showLoadingBar(uint8_t filledBlocks)
{
  if (filledBlocks > 8) {
    filledBlocks = 8;
  }

  ssd1306_clear();
  drawTextCentered(2, "Pocket Lab PSU", 1);
  ssd1306_fill_rect(3, 17, 122, 12, true);       /* border */
  ssd1306_fill_rect(4, 18, 120, 10, false);      /* hollow inside */
  ssd1306_fill_rect(5, 19, filledBlocks * 118 / 8, 8, true);
  oledShow();
}

/* Big "set value" screen shown while the knob is being used */
static void showSetVoltage(void)
{
  char buf[12];

  ssd1306_clear();
  drawTextCentered(0, "SET VOLTAGE", 1);
  fmtSetMv(buf, sizeof(buf), appliedTargetMv);
  drawTextCentered(13, buf, 2);
  oledShow();
}

static void showCurrentLimit(void)
{
  char buf[12];

  ssd1306_clear();
  drawTextCentered(0, "CURRENT LIMIT", 1);
  fmtLimitMa(buf, sizeof(buf), appliedCurrentLimitMa);
  drawTextCentered(13, buf, 2);
  oledShow();
}

/* ----------------------------------------------------
 * TPS55289 functions
 * ---------------------------------------------------- */
static bool findTPS55289(void)
{
  uint8_t dummy;

  if (readReg8(TPS_ADDR_1, REG_STATUS, &dummy)) {
    tpsAddr = TPS_ADDR_1;
    return true;
  }

  if (readReg8(TPS_ADDR_2, REG_STATUS, &dummy)) {
    tpsAddr = TPS_ADDR_2;
    return true;
  }

  return false;
}

static bool configureTPS55289(void)
{
  if (!mainOutputIsOn()) {
    return false;
  }

  bool ok = true;

  /* Internal feedback, 0.8 V to 22 V range, 10 mV steps */
  ok &= writeReg8(tpsAddr, REG_VOUT_FS, 0x03);
  ok &= writeReg8(tpsAddr, REG_VOUT_SR, TPS_VOUT_SR_VALUE);
  ok &= writeReg8(tpsAddr, REG_MODE, TPS_MODE_VALUE);

  return ok;
}

static bool writeTPS55289VoltageMvRaw(int millivolts)
{
  millivolts = clampInt(millivolts, MIN_MV, MAX_MV);
  int code = clampInt((millivolts - 800) / 10, 0, 0x7FF);

  uint8_t refLsb = (uint8_t)(code & 0xFF);
  uint8_t refMsb = (uint8_t)((code >> 8) & 0x07);

  /* Write REF_LSB first, then REF_MSB as required by the TPS55289 */
  bool ok = writeReg8(tpsAddr, REG_REF_LSB, refLsb);
  ok &= writeReg8(tpsAddr, REG_REF_MSB, refMsb);
  return ok;
}

static bool setTPS55289VoltageMv(int millivolts)
{
  if (!mainOutputIsOn()) {
    return false;
  }

  return writeTPS55289VoltageMvRaw(millivolts);
}

static bool setTPS55289CurrentLimitMa(int milliamps)
{
  if (!mainOutputIsOn()) {
    return false;
  }

  milliamps = clampInt(milliamps, MIN_CURRENT_LIMIT_MA, MAX_CURRENT_LIMIT_MA);

  /* Bits 6:0 are the limit in 50 mA steps; bit 7 enables limiting */
  int limitCode = clampInt((milliamps + 25) / 50, 0, 127);
  uint8_t registerValue = 0x80 | (uint8_t)limitCode;

  return writeReg8(tpsAddr, REG_IOUT_LIMIT, registerValue);
}

static void checkTPS55289Status(void)
{
  if (!tpsFound || !mainOutputIsOn()) {
    return;
  }

  uint32_t now = HAL_GetTick();

  if (now - lastTpsStatusCheckMs < TPS_STATUS_CHECK_INTERVAL_MS) {
    return;
  }

  lastTpsStatusCheckMs = now;

  uint8_t status = 0;

  if (!readReg8(tpsAddr, REG_STATUS, &status)) {
    return;
  }

  /* Bits 7:5 are latched protection flags:
   * bit 7 = short-circuit protection
   * bit 6 = output overcurrent
   * bit 5 = output overvoltage */
  latchedTpsFaults |= status & 0xE0;
}

/* Returns true if a fault screen was drawn */
static bool showLatchedTPSFault(void)
{
  const char *msg = NULL;

  if (latchedTpsFaults & 0x80) {
    msg = "TPS SCP";
  } else if (latchedTpsFaults & 0x40) {
    msg = "TPS OCP";
  } else if (latchedTpsFaults & 0x20) {
    msg = "TPS OVP";
  }

  if (msg == NULL) {
    return false;
  }

  ssd1306_clear();
  drawTextCentered(4, msg, 2);
  drawTextCentered(24, "turn knob to clear", 1);
  oledShow();
  return true;
}

/* ----------------------------------------------------
 * INA226 functions
 * ---------------------------------------------------- */
static bool readINA226(uint8_t addr, float shuntOhms, bool invertCurrent,
                       float *busVolts, float *amps, float *watts)
{
  int16_t rawShunt = 0;
  int16_t rawBus = 0;

  bool ok1 = readReg16(addr, INA_REG_SHUNT_VOLTAGE, &rawShunt);
  bool ok2 = readReg16(addr, INA_REG_BUS_VOLTAGE, &rawBus);

  if (!ok1 || !ok2) {
    return false;
  }

  float shuntVolts = rawShunt * SHUNT_LSB_VOLTS;
  *busVolts = rawBus * BUS_LSB_VOLTS;

  *amps = shuntVolts / shuntOhms;

  if (invertCurrent) {
    *amps = -*amps;
  }

#if SHOW_DRAW_AS_POSITIVE
  if (*amps < 0) {
    *amps = -*amps;
  }
#endif

  *watts = *busVolts * *amps;

  return true;
}

/* Same battery INA reading, but keeps the sign.
 * Needed for charge detection because the normal display code
 * intentionally turns current draw into a positive number. */
static bool readBatteryINASigned(float *busVolts, float *signedAmps)
{
  int16_t rawShunt = 0;
  int16_t rawBus = 0;

  bool ok1 = readReg16(INA_BATTERY_ADDR, INA_REG_SHUNT_VOLTAGE, &rawShunt);
  bool ok2 = readReg16(INA_BATTERY_ADDR, INA_REG_BUS_VOLTAGE, &rawBus);

  if (!ok1 || !ok2) {
    return false;
  }

  float shuntVolts = rawShunt * SHUNT_LSB_VOLTS;
  *busVolts = rawBus * BUS_LSB_VOLTS;

  *signedAmps = shuntVolts / BATTERY_SHUNT_OHMS;

#if INVERT_BATTERY_CURRENT
  *signedAmps = -*signedAmps;
#endif

  return true;
}

static bool batteryCurrentIndicatesCharging(float signedAmps)
{
#if BATTERY_CHARGING_CURRENT_IS_POSITIVE
  return signedAmps > BATTERY_CHARGING_CURRENT_THRESHOLD_A;
#else
  return signedAmps < -BATTERY_CHARGING_CURRENT_THRESHOLD_A;
#endif
}

static bool readBatteryINA(float *busVolts, float *amps, float *watts)
{
  return readINA226(INA_BATTERY_ADDR, BATTERY_SHUNT_OHMS,
                    INVERT_BATTERY_CURRENT, busVolts, amps, watts);
}

static bool readOutputINA(float *busVolts, float *amps, float *watts)
{
  return readINA226(INA_OUTPUT_ADDR, OUTPUT_SHUNT_OHMS,
                    INVERT_OUTPUT_CURRENT, busVolts, amps, watts);
}

/* ----------------------------------------------------
 * Encoder functions
 * ---------------------------------------------------- */

/* Number of detents turned since the last call (hardware TIM2 counter). */
static int readEncoderDetents(void)
{
  uint16_t count = (uint16_t)__HAL_TIM_GET_COUNTER(&htim2);
  int16_t delta = (int16_t)(count - lastEncCount);

  int detents = delta / ENCODER_COUNTS_PER_DETENT;

  if (detents == 0) {
    return 0;
  }

  /* Keep the sub-detent remainder for the next call */
  lastEncCount = (uint16_t)(lastEncCount
                            + detents * ENCODER_COUNTS_PER_DETENT);

#if ENCODER_REVERSE_DIRECTION
  detents = -detents;
#endif

  return detents;
}

static void resetEncoderState(void)
{
  lastEncCount = (uint16_t)__HAL_TIM_GET_COUNTER(&htim2);

  bool reading = encoderButtonIsPressed();
  lastButtonState = !reading;          /* stored as "HIGH = released" */
  previousStableButtonState = !reading;
  lastButtonChangeMs = HAL_GetTick();
}

static bool encoderButtonPressedEvent(void)
{
  bool readingHigh = !encoderButtonIsPressed();
  uint32_t now = HAL_GetTick();

  if (readingHigh != lastButtonState) {
    lastButtonChangeMs = now;
    lastButtonState = readingHigh;
  }

  if ((now - lastButtonChangeMs) > BUTTON_DEBOUNCE_MS) {
    bool currentStable = readingHigh;

    if (previousStableButtonState && !currentStable) {
      previousStableButtonState = currentStable;
      return true;
    }

    previousStableButtonState = currentStable;
  }

  return false;
}

/* ----------------------------------------------------
 * Optional data-EEPROM voltage memory
 * ---------------------------------------------------- */
static int loadRememberedOutputVoltageMv(void)
{
  uint32_t word = *(volatile uint32_t *)EEPROM_VOLTAGE_ADDR;

  if ((word >> 16) != EEPROM_MAGIC_VALUE) {
    return STARTUP_MV;
  }

  int storedMv = (int)(word & 0xFFFFu);

  if (storedMv < MIN_MV || storedMv > MAX_MV) {
    return STARTUP_MV;
  }

  /* Snap to the nearest STEP_MV increment in case the stored value was odd */
  storedMv = ((storedMv + (STEP_MV / 2)) / STEP_MV) * STEP_MV;
  return clampInt(storedMv, MIN_MV, MAX_MV);
}

static void saveRememberedOutputVoltageMv(int millivolts)
{
  millivolts = clampInt(millivolts, MIN_MV, MAX_MV);

  uint32_t word = ((uint32_t)EEPROM_MAGIC_VALUE << 16) | (uint32_t)millivolts;

  /* Avoid unnecessary EEPROM wear if the value has not changed */
  if (*(volatile uint32_t *)EEPROM_VOLTAGE_ADDR == word) {
    return;
  }

  HAL_FLASHEx_DATAEEPROM_Unlock();
  HAL_FLASHEx_DATAEEPROM_Program(FLASH_TYPEPROGRAMDATA_WORD,
                                 EEPROM_VOLTAGE_ADDR, word);
  HAL_FLASHEx_DATAEEPROM_Lock();
}

/* ----------------------------------------------------
 * Dashboard
 * ---------------------------------------------------- */
static void renderDashboard(void)
{
  /* Keep any detected TPS55289 fault visible */
  if (showLatchedTPSFault()) {
    return;
  }

  char buf[16];

  float batteryVolts = 0, batterySignedAmps = 0;
  bool batteryOk = readBatteryINASigned(&batteryVolts, &batterySignedAmps);
  bool charging = SHOW_CHARGING_STATUS && batteryOk
                  && batteryCurrentIndicatesCharging(batterySignedAmps);

  float outputVolts = 0, outputAmps = 0, outputWatts = 0;
  bool outputOk = readOutputINA(&outputVolts, &outputAmps, &outputWatts);

  ssd1306_clear();

  /* Top row: set voltage and current limit */
  fmtSetMv(buf, sizeof(buf), appliedTargetMv);
  char line[24];
  snprintf(line, sizeof(line), "S:%s", buf);
  ssd1306_draw_text(0, 0, line, 1);

  fmtLimitMa(buf, sizeof(buf), appliedCurrentLimitMa);
  snprintf(line, sizeof(line), "I:%s", buf);
  drawTextRight(0, line, 1);

  /* Middle row (big): measured output voltage and load current */
  if (outputOk) {
    fmtVolts(buf, sizeof(buf), outputVolts);
    ssd1306_draw_text(0, 10, buf, 2);

    float displayAmps = outputAmps < 0 ? -outputAmps : outputAmps;
    fmtAmps(buf, sizeof(buf), displayAmps);
    drawTextRight(10, buf, 2);
  } else {
    drawTextCentered(10, "O INA?", 2);
  }

  /* Bottom row: battery voltage and total system power / charge status */
  if (batteryOk) {
    fmtVolts1dp(buf, sizeof(buf), batteryVolts);
    snprintf(line, sizeof(line), "B:%s", buf);
    ssd1306_draw_text(0, 25, line, 1);

    if (charging) {
      float chgAmps = batterySignedAmps < 0 ? -batterySignedAmps
                                            : batterySignedAmps;
      fmtAmps(buf, sizeof(buf), chgAmps);
      snprintf(line, sizeof(line), "CHG %s", buf);
      drawTextRight(25, line, 1);
    } else {
      fmtWatts(buf, sizeof(buf), batteryVolts * batterySignedAmps);
      snprintf(line, sizeof(line), "P:%s", buf);
      drawTextRight(25, line, 1);
    }
  } else {
    ssd1306_draw_text(0, 25, "B INA?", 1);
  }

  oledShow();
}

/* ----------------------------------------------------
 * Main output on/off transition handling
 * ---------------------------------------------------- */
static void handleMainOutputState(void)
{
  bool currentMainOutputState = mainOutputIsOn();

  if (currentMainOutputState == lastMainOutputState) {
    return;
  }

  lastMainOutputState = currentMainOutputState;

  if (currentMainOutputState) {
    HAL_Delay(100); /* allow switched rail and I2C devices to settle */

    oledOk = ssd1306_init();
    showLoadingBar(0);

    for (uint8_t blocks = 1; blocks <= 3; blocks++) {
      HAL_Delay(70);
      showLoadingBar(blocks);
    }

    tpsFound = findTPS55289();

    if (!tpsFound) {
      showMessage("TPS?");
      return;
    }

    showLoadingBar(4);

    /* Choose the voltage to apply on this power-on:
     * with memory disabled this is STARTUP_MV, with memory enabled it is
     * the voltage saved at the last power-off. */
#if REMEMBER_OUTPUT_VOLTAGE_ON_POWER_OFF
    appliedTargetMv = loadRememberedOutputVoltageMv();
#else
    appliedTargetMv = STARTUP_MV;
#endif

    appliedCurrentLimitMa = DEFAULT_CURRENT_LIMIT_MA;

    /* Always start in voltage-adjustment mode. Also reset the encoder and
     * button state so a startup transient cannot immediately toggle the UI
     * into current-limit mode. */
    adjustingCurrentLimit = false;
    resetEncoderState();

    latchedTpsFaults = 0;

    bool settingsOk = configureTPS55289();
    settingsOk &= setTPS55289CurrentLimitMa(appliedCurrentLimitMa);
    settingsOk &= setTPS55289VoltageMv(appliedTargetMv);

    if (settingsOk) {
      /* Finish filling the bar after initialization succeeds */
      for (uint8_t blocks = 5; blocks <= 8; blocks++) {
        HAL_Delay(70);
        showLoadingBar(blocks);
      }

      HAL_Delay(100);
      showSetVoltage();
    } else {
      showMessage("SET ERR");
    }

    /* Re-confirm voltage-adjustment mode after the startup sequence */
    adjustingCurrentLimit = false;
    resetEncoderState();

    knobDisplayUntil = HAL_GetTick() + KNOB_DISPLAY_TIMEOUT_MS;
    lastDashboardRefresh = HAL_GetTick() - DASHBOARD_REFRESH_MS;
    lastTpsStatusCheckMs = HAL_GetTick();
  }
  else {
    /* Main output is being switched off.
     *
     * If voltage memory is enabled, save the currently selected voltage.
     * Otherwise, try to return the TPS55289 to STARTUP_MV before the next
     * startup. This assumes the MCU and I2C bus stay powered long enough
     * after the switch-state pin changes for this I2C write to complete. */
    if (tpsFound || findTPS55289()) {
#if REMEMBER_OUTPUT_VOLTAGE_ON_POWER_OFF
      saveRememberedOutputVoltageMv(appliedTargetMv);
#else
      writeTPS55289VoltageMvRaw(STARTUP_MV);
      appliedTargetMv = STARTUP_MV;
#endif
    } else {
      /* If the TPS55289 is already gone, still keep the firmware-side
       * target safe */
#if !REMEMBER_OUTPUT_VOLTAGE_ON_POWER_OFF
      appliedTargetMv = STARTUP_MV;
#endif
    }

    /* The OLED may lose power with the switched rail; a failed command
     * is ignored and the display is re-initialized on the next power-on. */
    ssd1306_display_off();
    oledOk = false;
    tpsFound = false;
    latchedTpsFaults = 0;
  }
}

/* ----------------------------------------------------
 * GPIO setup from app_config.h
 * ---------------------------------------------------- */
static void appGpioClockEnable(GPIO_TypeDef *port)
{
  if (port == GPIOA) {
    __HAL_RCC_GPIOA_CLK_ENABLE();
  } else if (port == GPIOB) {
    __HAL_RCC_GPIOB_CLK_ENABLE();
  } else if (port == GPIOC) {
    __HAL_RCC_GPIOC_CLK_ENABLE();
  }
}

static void appGpioInit(void)
{
  GPIO_InitTypeDef gpio = {0};

  /* Power switch sense */
  appGpioClockEnable(SWITCH_STATE_GPIO_PORT);
  gpio.Pin = SWITCH_STATE_GPIO_PIN;
  gpio.Mode = GPIO_MODE_INPUT;
  gpio.Pull = SWITCH_STATE_GPIO_PULL;
  HAL_GPIO_Init(SWITCH_STATE_GPIO_PORT, &gpio);

  /* Encoder push button */
  appGpioClockEnable(ENC_SW_GPIO_PORT);
  gpio.Pin = ENC_SW_GPIO_PIN;
  gpio.Mode = GPIO_MODE_INPUT;
  gpio.Pull = ENC_SW_GPIO_PULL;
  HAL_GPIO_Init(ENC_SW_GPIO_PORT, &gpio);

#if ENC_AB_INTERNAL_PULLUPS
  /* Re-init the TIM2 encoder pins with pull-ups (CubeMX sets no pull).
   * Fixed pins: PA1 = TIM2_CH2 (AF2), PA8 = TIM2_CH1 (AF5). */
  gpio.Mode = GPIO_MODE_AF_PP;
  gpio.Pull = GPIO_PULLUP;
  gpio.Speed = GPIO_SPEED_FREQ_LOW;

  gpio.Pin = GPIO_PIN_1;
  gpio.Alternate = GPIO_AF2_TIM2;
  HAL_GPIO_Init(GPIOA, &gpio);

  gpio.Pin = GPIO_PIN_8;
  gpio.Alternate = GPIO_AF5_TIM2;
  HAL_GPIO_Init(GPIOA, &gpio);
#endif
}

/* ----------------------------------------------------
 * Public entry points
 * ---------------------------------------------------- */
void App_Init(void)
{
  appGpioInit();

  HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);

  HAL_Delay(50);

  resetEncoderState();

  /* Firmware-side defaults before the first main-output transition is
   * handled. The actual TPS55289 voltage is applied inside
   * handleMainOutputState(). */
#if REMEMBER_OUTPUT_VOLTAGE_ON_POWER_OFF
  appliedTargetMv = loadRememberedOutputVoltageMv();
#else
  appliedTargetMv = STARTUP_MV;
#endif

  appliedCurrentLimitMa = DEFAULT_CURRENT_LIMIT_MA;
  adjustingCurrentLimit = false;

  handleMainOutputState();

  /* Optional battery INA check happens silently here */
  float v, a, w;
  readBatteryINA(&v, &a, &w);
}

void App_Loop(void)
{
  handleMainOutputState();

  if (!mainOutputIsOn()) {
    HAL_Delay(20);
    return;
  }

  checkTPS55289Status();

  int detents = readEncoderDetents();

  if (detents != 0) {
    latchedTpsFaults = 0;

    if (!tpsFound) {
      tpsFound = findTPS55289();
    }

    if (adjustingCurrentLimit) {
      appliedCurrentLimitMa += detents * CURRENT_LIMIT_STEP_MA;
      appliedCurrentLimitMa = clampInt(appliedCurrentLimitMa,
                                       MIN_CURRENT_LIMIT_MA,
                                       MAX_CURRENT_LIMIT_MA);

      if (tpsFound && setTPS55289CurrentLimitMa(appliedCurrentLimitMa)) {
        showCurrentLimit();
      } else {
        showMessage("SET ERR");
      }
    }
    else {
      appliedTargetMv += detents * STEP_MV;
      appliedTargetMv = clampInt(appliedTargetMv, MIN_MV, MAX_MV);

      /* Apply each voltage change immediately as the encoder turns */
      if (tpsFound && setTPS55289VoltageMv(appliedTargetMv)) {
        showSetVoltage();
      } else {
        showMessage("SET ERR");
      }
    }

    knobDisplayUntil = HAL_GetTick() + KNOB_DISPLAY_TIMEOUT_MS;
  }

  if (encoderButtonPressedEvent()) {
    /* Toggle between voltage adjustment and current-limit adjustment */
    adjustingCurrentLimit = !adjustingCurrentLimit;

    if (adjustingCurrentLimit) {
      showCurrentLimit();
    } else {
      showSetVoltage();
    }

    knobDisplayUntil = HAL_GetTick() + KNOB_DISPLAY_TIMEOUT_MS;
  }

  uint32_t now = HAL_GetTick();

  if ((int32_t)(now - knobDisplayUntil) < 0) {
    return; /* keep the set-value screen until the timeout */
  }

  if (now - lastDashboardRefresh >= DASHBOARD_REFRESH_MS) {
    lastDashboardRefresh = now;
    renderDashboard();
  }
}
