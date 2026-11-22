#include "tim.h"
#include "pcanpro_timestamp.h"

void pcan_timestamp_init(void)
{
  if (HAL_TIM_Base_Start(&htim2) != HAL_OK)
  {
    Error_Handler();
  }

  /* Keep uwTick non-zero so initial LED event timestamps evaluate as armed. */
  if (HAL_GetTick() == 0U)
  {
    HAL_IncTick();
  }
}

uint32_t pcan_timestamp_millis(void)
{
  return HAL_GetTick();
}

uint32_t pcan_timestamp_us(void)
{
  return __HAL_TIM_GET_COUNTER(&htim2);
}

uint64_t pcan_timestamp_us64(void)
{
  static uint32_t last_us = 0U;
  static uint32_t hi_us = 0U;

  uint32_t lo = __HAL_TIM_GET_COUNTER(&htim2);
  if (lo < last_us)
  {
    ++hi_us;
  }
  last_us = lo;

  return (((uint64_t)hi_us) << 32) | (uint64_t)lo;
}
