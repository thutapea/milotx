/**
  ******************************************************************************
  * @file    si5351.h
  * @brief   Minimal Si5351A/B/C driver (STM32 HAL, blocking I2C).
  *
  *          Programming order follows Si5351 datasheet Figure 10, register
  *          encoding follows AN619.
  ******************************************************************************
  */
#ifndef __SI5351_H
#define __SI5351_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include <stdbool.h>
#include <stdint.h>

/* Board configuration -------------------------------------------------------*/
/* 7-bit address 0x60, shifted left for HAL (write 0xC0 / read 0xC1). */
#define SI5351_I2C_ADDR        (0x60U << 1)

/* PLL reference: 1 = CMOS clock on CLKIN (milotx: 25 MHz TCXO U6 via R2),
 * 0 = crystal on XA/XB. */
#ifndef SI5351_REF_CLKIN
#define SI5351_REF_CLKIN       1
#endif

/* Reference frequency. CLKIN must be 10-40 MHz (no CLKIN divider used). */
#ifndef SI5351_REF_FREQ
#define SI5351_REF_FREQ        25000000UL
#endif

/* Crystal only. Register 183 load: 0xD2 = 10 pF, 0x92 = 8 pF, 0x52 = 6 pF. */
#ifndef SI5351_XTAL_LOAD
#define SI5351_XTAL_LOAD       0xD2U
#endif

/* OEB pin (active low output enable). Uses the CubeMX user label
 * "SI5351_OEB" if one is set on the pin, otherwise PA0. */
#ifndef SI5351_OEB_GPIO_Port
#define SI5351_OEB_GPIO_Port   GPIOA
#define SI5351_OEB_Pin         GPIO_PIN_0
#endif

#define SI5351_I2C_TIMEOUT_MS  10U
#define SI5351_LOCK_TIMEOUT_MS 100U

/* Valid output range without the >150 MHz DIVBY4 mode. */
#define SI5351_FREQ_MIN        2500UL
#define SI5351_FREQ_MAX        112500000UL

/* Register 0 (Device Status) bits */
#define SI5351_STATUS_SYS_INIT 0x80U
#define SI5351_STATUS_LOL_B    0x40U
#define SI5351_STATUS_LOL_A    0x20U
#define SI5351_STATUS_LOS_CLKIN 0x10U
#define SI5351_STATUS_LOS_XTAL 0x08U

typedef enum
{
  SI5351_CLK0 = 0,
  SI5351_CLK1,
  SI5351_CLK2,
  SI5351_CLK3,
  SI5351_CLK4,
  SI5351_CLK5
} si5351_clk_t;

typedef enum
{
  SI5351_PLLA = 0,
  SI5351_PLLB
} si5351_pll_t;

typedef enum
{
  SI5351_DRIVE_2MA = 0,
  SI5351_DRIVE_4MA,
  SI5351_DRIVE_6MA,
  SI5351_DRIVE_8MA
} si5351_drive_t;

/* Waits for SYS_INIT, disables and powers down all outputs, selects the
 * CLKIN or the crystal as the PLL reference. Holds OEB high (outputs off). */
HAL_StatusTypeDef si5351_init(I2C_HandleTypeDef *hi2c);

/* Programs `pll` and the clock's MultiSynth for `freq_hz`, resets the PLL and
 * waits for lock. The output stays disabled until si5351_output_enable().
 * Each clock owns its PLL's VCO, so two unrelated frequencies need
 * different PLLs. */
HAL_StatusTypeDef si5351_set_freq(si5351_clk_t clk, si5351_pll_t pll,
                                  uint32_t freq_hz, si5351_drive_t drive);

/* Enables/disables a single output in register 3. */
HAL_StatusTypeDef si5351_output_enable(si5351_clk_t clk, bool enable);

/* Drives the OEB pin: true = pin low = outputs enabled. */
void si5351_oeb(bool enable);

HAL_StatusTypeDef si5351_read_status(uint8_t *status);

HAL_StatusTypeDef si5351_write_reg(uint8_t reg, uint8_t value);
HAL_StatusTypeDef si5351_read_reg(uint8_t reg, uint8_t *value);

#ifdef __cplusplus
}
#endif

#endif /* __SI5351_H */
