/* Pocket Lab-Bench Power Supply - STM32L031G6U6 port
 *
 * All user-adjustable settings live in this file: GPIO assignments,
 * I2C addresses, voltage/current limits and display behaviour.
 *
 * Ported from the original ATtiny3216 Arduino sketch
 * (firmware/Pocket-Lab-Bench-Power-V1.ino) by "Ben Makes Everything".
 *
 * Display: 0.91" 128x32 SSD1306 OLED on the shared I2C1 bus.
 */

#ifndef APP_CONFIG_H
#define APP_CONFIG_H

#include "stm32l0xx_hal.h"

/* ----------------------------------------------------
 * GPIO assignments
 * ----------------------------------------------------
 * These pins are plain GPIO inputs and can be moved to any free pin;
 * App_Init() configures them from these defines (the CubeMX settings
 * are overridden). Remember to also enable the port clock if you move
 * a pin to a port other than GPIOA/GPIOB (see app_gpio_init in app.c).
 *
 * NOT freely movable:
 *   - Encoder A/B: fixed to TIM2_CH1 = PA8 (AF5) and TIM2_CH2 = PA1 (AF2),
 *     because the hardware quadrature decoder of TIM2 is used.
 *   - I2C: fixed to I2C1 on PA9 (SCL) / PA10 (SDA).
 * Change those in CubeMX (.ioc) if the board layout ever changes.
 */

/* Power switch sense: HIGH = main output OFF, LOW = main output ON */
#define SWITCH_STATE_GPIO_PORT      GPIOA
#define SWITCH_STATE_GPIO_PIN       GPIO_PIN_7
#define SWITCH_STATE_GPIO_PULL      GPIO_PULLUP

/* Rotary encoder push button, active LOW */
#define ENC_SW_GPIO_PORT            GPIOB
#define ENC_SW_GPIO_PIN             GPIO_PIN_1
#define ENC_SW_GPIO_PULL            GPIO_PULLUP

/* Enable internal pull-ups on the encoder A/B timer pins (PA8/PA1).
 * Set to 0 if the PCB has external pull-ups. */
#define ENC_AB_INTERNAL_PULLUPS     1

/* ----------------------------------------------------
 * Encoder behaviour
 * ----------------------------------------------------
 * TIM2 runs in encoder mode TI1 (counts both edges of channel A only),
 * which gives 2 counts per full quadrature cycle. A PEC12R-style encoder
 * produces one full cycle (4 transitions) per detent -> 2 counts/detent.
 */
#define ENCODER_COUNTS_PER_DETENT   2
/* The original firmware reversed the raw direction so that clockwise
 * increases the value. Set to 0 if your encoder turns the wrong way. */
#define ENCODER_REVERSE_DIRECTION   1

#define BUTTON_DEBOUNCE_MS          40u

/* ----------------------------------------------------
 * SSD1306 OLED (0.91", 128x32) settings
 * ---------------------------------------------------- */
#define OLED_I2C_ADDR               0x3C  /* 7-bit address */
#define OLED_WIDTH                  128
#define OLED_HEIGHT                 32

/* ----------------------------------------------------
 * TPS55289 buck-boost converter settings
 * ---------------------------------------------------- */
#define TPS_ADDR_1                  0x74
#define TPS_ADDR_2                  0x75

/* 0x31 = longest OCP response delay (~12.3 ms), 2.5 mV/us VOUT slew */
#define TPS_VOUT_SR_VALUE           0x31
/* 0x82 = output enabled, forced-PWM enabled */
#define TPS_MODE_VALUE              0x82

/* ----------------------------------------------------
 * INA226 current monitor settings
 * ---------------------------------------------------- */
#define INA_BATTERY_ADDR            0x40  /* 7-bit, binary 1000000 */
#define INA_OUTPUT_ADDR             0x41  /* 7-bit, binary 1000001 */

/* Shunt resistor values: 10 milliohms = 0.010 ohms */
#define BATTERY_SHUNT_OHMS          0.01f
#define OUTPUT_SHUNT_OHMS           0.01f

#define INVERT_BATTERY_CURRENT      0
#define INVERT_OUTPUT_CURRENT       0

#define SHOW_DRAW_AS_POSITIVE       1

/* ----------------------------------------------------
 * Charging display settings
 * ----------------------------------------------------
 * If enabled, a "CHG" indicator is shown while net current flows
 * into the battery. */
#define SHOW_CHARGING_STATUS        1
#define BATTERY_CHARGING_CURRENT_IS_POSITIVE  0
/* Ignore small currents/noise around zero so the display does not flicker */
#define BATTERY_CHARGING_CURRENT_THRESHOLD_A  0.050f  /* 50 mA */

/* ----------------------------------------------------
 * Output voltage settings
 * ---------------------------------------------------- */
/* Initial startup voltage - the device powers on with this value (in mV) */
#define STARTUP_MV                  5000

/* Remember the last voltage set before power-off?
 * 0 = default to STARTUP_MV
 * 1 = save the selected voltage to the L031 data EEPROM on power-off
 *     and restore it on startup */
#define REMEMBER_OUTPUT_VOLTAGE_ON_POWER_OFF  0

/* Regulator limits: TPS55289 range is 800 mV .. 22000 mV */
#define MIN_MV                      1000
#define MAX_MV                      20000
#define STEP_MV                     100

/* Default current-limit setting (the TPS55289 uses 50 mA steps) */
#define DEFAULT_CURRENT_LIMIT_MA    5000
#define MIN_CURRENT_LIMIT_MA        100
/* Theoretical max is 6350 mA, but that is likely more than the board can handle */
#define MAX_CURRENT_LIMIT_MA        5000
#define CURRENT_LIMIT_STEP_MA       50

/* ----------------------------------------------------
 * Display timing
 * ---------------------------------------------------- */
/* How long the big "set value" screen stays after touching the knob */
#define KNOB_DISPLAY_TIMEOUT_MS     1500u
/* Dashboard measurement refresh interval */
#define DASHBOARD_REFRESH_MS        250u
/* TPS55289 fault status poll interval */
#define TPS_STATUS_CHECK_INTERVAL_MS 20u

#endif /* APP_CONFIG_H */
