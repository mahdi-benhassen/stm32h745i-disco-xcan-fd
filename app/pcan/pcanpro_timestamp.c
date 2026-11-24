#include "at32f45x.h"
#include "at32f45x_int.h"
#include "pcanpro_timestamp.h"

void pcan_timestamp_init(void)
{
  uint32_t pclk1_hz = system_core_clock;
  uint32_t prescaler = (pclk1_hz / 1000000U);

  if (prescaler > 0U)
  {
    prescaler -= 1U;
  }

  crm_periph_clock_enable(CRM_TMR2_PERIPH_CLOCK, TRUE);
  tmr_reset(TMR2);
  tmr_32_bit_function_enable(TMR2, TRUE);
  tmr_base_init(TMR2, 0xFFFFFFFFU, (uint16_t)prescaler);
  tmr_cnt_dir_set(TMR2, TMR_COUNT_UP);
  tmr_counter_enable(TMR2, TRUE);

  app_ensure_tick_nonzero();
}

uint32_t pcan_timestamp_millis(void)
{
  return app_get_tick_ms();
}

uint32_t pcan_timestamp_us(void)
{
  return tmr_counter_value_get(TMR2);
}

uint64_t pcan_timestamp_us64(void)
{
  static uint32_t last_us = 0U;
  static uint32_t hi_us = 0U;

  uint32_t lo = tmr_counter_value_get(TMR2);
  if (lo < last_us)
  {
    ++hi_us;
  }
  last_us = lo;

  return (((uint64_t)hi_us) << 32U) | (uint64_t)lo;
}
