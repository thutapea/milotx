/**
  ******************************************************************************
  * @file    si5351.c
  * @brief   Minimal Si5351A/B/C driver (STM32 HAL, blocking I2C).
  ******************************************************************************
  */
#include "si5351.h"

/* Register map (AN619) */
#define REG_DEVICE_STATUS   0U
#define REG_INT_MASK        2U
#define REG_OUTPUT_ENABLE   3U
#define REG_PLL_INPUT_SRC   15U
#define REG_CLK0_CTRL       16U
#define REG_CLK6_CTRL       22U  /* bit 6 = FBA_INT */
#define REG_CLK7_CTRL       23U  /* bit 6 = FBB_INT */
#define REG_PLLA_BASE       26U
#define REG_PLLB_BASE       34U
#define REG_MS0_BASE        42U
#define REG_PLL_RESET       177U
#define REG_XTAL_LOAD       183U

#define CLK_CTRL_PDN        0x80U
#define CLK_CTRL_INT        0x40U
#define CLK_CTRL_SRC_PLLB   0x20U
#define CLK_CTRL_SRC_MS     0x0CU
#define FB_INT              0x40U

#define PLL_RESET_A         0x20U
#define PLL_RESET_B         0x80U

#define VCO_MIN             600000000ULL
#define VCO_MAX             900000000ULL
#define FRAC_DENOM_MAX      1048575UL

#if SI5351_REF_CLKIN
#define PLL_SRC_SEL         0x0CU  /* reg 15: PLLA_SRC | PLLB_SRC = CLKIN, CLKIN_DIV /1 */
#define LOS_REF             SI5351_STATUS_LOS_CLKIN
#define LOS_UNUSED          SI5351_STATUS_LOS_XTAL
#else
#define PLL_SRC_SEL         0x00U  /* reg 15: both PLLs from XTAL */
#define LOS_REF             SI5351_STATUS_LOS_XTAL
#define LOS_UNUSED          SI5351_STATUS_LOS_CLKIN
#endif

static I2C_HandleTypeDef *si5351_i2c;
static uint8_t output_enable_mask = 0xFFU;  /* register 3, 1 = disabled */

HAL_StatusTypeDef si5351_write_reg(uint8_t reg, uint8_t value)
{
  return HAL_I2C_Mem_Write(si5351_i2c, SI5351_I2C_ADDR, reg, I2C_MEMADD_SIZE_8BIT,
                           &value, 1U, SI5351_I2C_TIMEOUT_MS);
}

HAL_StatusTypeDef si5351_read_reg(uint8_t reg, uint8_t *value)
{
  return HAL_I2C_Mem_Read(si5351_i2c, SI5351_I2C_ADDR, reg, I2C_MEMADD_SIZE_8BIT,
                          value, 1U, SI5351_I2C_TIMEOUT_MS);
}

static HAL_StatusTypeDef write_burst(uint8_t reg, uint8_t *data, uint16_t len)
{
  return HAL_I2C_Mem_Write(si5351_i2c, SI5351_I2C_ADDR, reg, I2C_MEMADD_SIZE_8BIT,
                           data, len, SI5351_I2C_TIMEOUT_MS);
}

HAL_StatusTypeDef si5351_read_status(uint8_t *status)
{
  return si5351_read_reg(REG_DEVICE_STATUS, status);
}

void si5351_oeb(bool enable)
{
  HAL_GPIO_WritePin(SI5351_OEB_GPIO_Port, SI5351_OEB_Pin,
                    enable ? GPIO_PIN_RESET : GPIO_PIN_SET);
}

static uint32_t gcd(uint32_t a, uint32_t b)
{
  while (b != 0U)
  {
    uint32_t t = a % b;
    a = b;
    b = t;
  }
  return a;
}

/* Packs divider a + b/c into the 8-byte P1/P2/P3 block (AN619 §3.2, §4.1).
 * `r_div` and `divby4` only apply to MultiSynth blocks (byte 3 upper bits). */
static void encode_divider(uint8_t out[8], uint32_t a, uint32_t b, uint32_t c,
                           uint8_t r_div)
{
  uint32_t floor_term = (uint32_t)((128ULL * b) / c);
  uint32_t p1 = 128U * a + floor_term - 512U;
  uint32_t p2 = 128U * b - c * floor_term;
  uint32_t p3 = c;

  out[0] = (uint8_t)(p3 >> 8);
  out[1] = (uint8_t)p3;
  out[2] = (uint8_t)((r_div << 4) | ((p1 >> 16) & 0x03U));
  out[3] = (uint8_t)(p1 >> 8);
  out[4] = (uint8_t)p1;
  out[5] = (uint8_t)(((p3 >> 12) & 0xF0U) | ((p2 >> 16) & 0x0FU));
  out[6] = (uint8_t)(p2 >> 8);
  out[7] = (uint8_t)p2;
}

static HAL_StatusTypeDef wait_lock(si5351_pll_t pll)
{
  uint8_t lol = (pll == SI5351_PLLA) ? SI5351_STATUS_LOL_A : SI5351_STATUS_LOL_B;
  uint32_t start = HAL_GetTick();
  uint8_t status;

  do
  {
    if (si5351_read_status(&status) != HAL_OK)
    {
      return HAL_ERROR;
    }
    if ((status & (lol | LOS_REF | SI5351_STATUS_SYS_INIT)) == 0U)
    {
      return HAL_OK;
    }
  } while ((HAL_GetTick() - start) < SI5351_LOCK_TIMEOUT_MS);

  return HAL_TIMEOUT;
}

HAL_StatusTypeDef si5351_init(I2C_HandleTypeDef *hi2c)
{
  uint8_t status;
  uint8_t clk_off[8];
  uint32_t start;

  si5351_i2c = hi2c;
  si5351_oeb(false);

  if (HAL_I2C_IsDeviceReady(si5351_i2c, SI5351_I2C_ADDR, 3U, SI5351_I2C_TIMEOUT_MS) != HAL_OK)
  {
    return HAL_ERROR;
  }

  /* Step 1: wait until SYS_INIT clears. */
  start = HAL_GetTick();
  do
  {
    if (si5351_read_status(&status) != HAL_OK)
    {
      return HAL_ERROR;
    }
    if ((HAL_GetTick() - start) >= SI5351_LOCK_TIMEOUT_MS)
    {
      return HAL_TIMEOUT;
    }
  } while ((status & SI5351_STATUS_SYS_INIT) != 0U);

  /* Step 2: disable all outputs. */
  output_enable_mask = 0xFFU;
  if (si5351_write_reg(REG_OUTPUT_ENABLE, output_enable_mask) != HAL_OK)
  {
    return HAL_ERROR;
  }

  /* Step 3: power down all output drivers (CLK0..CLK7). */
  for (uint8_t i = 0U; i < sizeof(clk_off); i++)
  {
    clk_off[i] = CLK_CTRL_PDN;
  }
  if (write_burst(REG_CLK0_CTRL, clk_off, sizeof(clk_off)) != HAL_OK)
  {
    return HAL_ERROR;
  }

  /* Step 4: only the used reference and PLL lock matter on INTR; mask the
   * unused input's LOS so it doesn't hold INTR low. */
  if (si5351_write_reg(REG_INT_MASK, LOS_UNUSED) != HAL_OK)
  {
    return HAL_ERROR;
  }

  if (si5351_write_reg(REG_PLL_INPUT_SRC, PLL_SRC_SEL) != HAL_OK)
  {
    return HAL_ERROR;
  }

#if SI5351_REF_CLKIN
  return HAL_OK;
#else
  return si5351_write_reg(REG_XTAL_LOAD, SI5351_XTAL_LOAD);
#endif
}

HAL_StatusTypeDef si5351_set_freq(si5351_clk_t clk, si5351_pll_t pll,
                                  uint32_t freq_hz, si5351_drive_t drive)
{
  uint8_t regs[8];
  uint8_t r_div = 0U;
  uint32_t ms_freq = freq_hz;
  uint32_t ms_div = 0U;
  uint32_t d_min;
  uint32_t d_max;
  uint64_t vco;
  uint32_t fb_a, fb_b, fb_c, g;
  uint8_t ctrl;
  uint8_t fb_reg;

  if ((clk > SI5351_CLK5) || (freq_hz < SI5351_FREQ_MIN) || (freq_hz > SI5351_FREQ_MAX))
  {
    return HAL_ERROR;
  }

  /* Use the R divider (/1../128) to bring low frequencies into MultiSynth range. */
  while (((uint64_t)ms_freq * 2048U < VCO_MIN) && (r_div < 7U))
  {
    ms_freq *= 2U;
    r_div++;
  }

  /* Even-integer output divider keeping the VCO in 600-900 MHz, searched from
   * the top of the VCO range down; prefer one that also makes the feedback
   * divider an integer (lowest jitter). */
  d_min = (uint32_t)((VCO_MIN + ms_freq - 1U) / ms_freq);
  d_max = (uint32_t)(VCO_MAX / ms_freq);
  if (d_min < 8U)
  {
    d_min = 8U;
  }
  if (d_max > 2048U)
  {
    d_max = 2048U;
  }
  for (uint32_t d = d_max & ~1U; d >= d_min; d -= 2U)
  {
    if (ms_div == 0U)
    {
      ms_div = d;
    }
    if (((uint64_t)ms_freq * d) % SI5351_REF_FREQ == 0U)
    {
      ms_div = d;
      break;
    }
  }
  if (ms_div == 0U)
  {
    return HAL_ERROR;
  }

  /* Feedback divider a + b/c = VCO / reference (valid 15..90). */
  vco = (uint64_t)ms_freq * ms_div;
  fb_a = (uint32_t)(vco / SI5351_REF_FREQ);
  fb_b = (uint32_t)(vco % SI5351_REF_FREQ);
  fb_c = SI5351_REF_FREQ;
  if (fb_b == 0U)
  {
    fb_c = 1U;
  }
  else
  {
    g = gcd(fb_b, fb_c);
    fb_b /= g;
    fb_c /= g;
    if (fb_c > FRAC_DENOM_MAX)
    {
      fb_b = (uint32_t)(((uint64_t)fb_b * FRAC_DENOM_MAX) / fb_c);
      fb_c = FRAC_DENOM_MAX;
    }
  }
  if ((fb_a < 15U) || (fb_a > 90U))
  {
    return HAL_ERROR;
  }

  /* PLL feedback MultiSynth */
  encode_divider(regs, fb_a, fb_b, fb_c, 0U);
  if (write_burst((pll == SI5351_PLLA) ? REG_PLLA_BASE : REG_PLLB_BASE,
                  regs, sizeof(regs)) != HAL_OK)
  {
    return HAL_ERROR;
  }

  /* FBA_INT / FBB_INT live in the CLK6 / CLK7 control registers. */
  fb_reg = (pll == SI5351_PLLA) ? REG_CLK6_CTRL : REG_CLK7_CTRL;
  if (si5351_read_reg(fb_reg, &ctrl) != HAL_OK)
  {
    return HAL_ERROR;
  }
  ctrl = (fb_b == 0U) ? (uint8_t)(ctrl | FB_INT) : (uint8_t)(ctrl & ~FB_INT);
  if (si5351_write_reg(fb_reg, ctrl) != HAL_OK)
  {
    return HAL_ERROR;
  }

  /* Output MultiSynth (always an integer here) + R divider */
  encode_divider(regs, ms_div, 0U, 1U, r_div);
  if (write_burst((uint8_t)(REG_MS0_BASE + 8U * clk), regs, sizeof(regs)) != HAL_OK)
  {
    return HAL_ERROR;
  }

  /* Power up the driver: integer mode, PLL source, MultiSynth source, drive. */
  ctrl = (uint8_t)(CLK_CTRL_INT | CLK_CTRL_SRC_MS | (uint8_t)drive);
  if (pll == SI5351_PLLB)
  {
    ctrl |= CLK_CTRL_SRC_PLLB;
  }
  if (si5351_write_reg((uint8_t)(REG_CLK0_CTRL + clk), ctrl) != HAL_OK)
  {
    return HAL_ERROR;
  }

  if (si5351_write_reg(REG_PLL_RESET,
                       (pll == SI5351_PLLA) ? PLL_RESET_A : PLL_RESET_B) != HAL_OK)
  {
    return HAL_ERROR;
  }

  return wait_lock(pll);
}

HAL_StatusTypeDef si5351_output_enable(si5351_clk_t clk, bool enable)
{
  if (enable)
  {
    output_enable_mask &= (uint8_t)~(1U << clk);
  }
  else
  {
    output_enable_mask |= (uint8_t)(1U << clk);
  }
  return si5351_write_reg(REG_OUTPUT_ENABLE, output_enable_mask);
}
