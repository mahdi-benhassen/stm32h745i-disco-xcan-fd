#include <stdint.h>
#include "hpm_soc.h"
#include "hpm_csr_drv.h"
#include "hpm_clock_drv.h"
#include "pcanpro_timestamp.h"

static uint32_t s_cpu_freq_hz = 0U;

void pcan_timestamp_init(void)
{
  s_cpu_freq_hz = clock_get_frequency(clock_cpu0);
}

uint32_t pcan_timestamp_millis(void)
{
  if (s_cpu_freq_hz == 0U)
  {
    return 1U;
  }

  uint32_t ms = (uint32_t)(hpm_csr_get_core_mcycle() / (s_cpu_freq_hz / 1000U));
  return (ms == 0U) ? 1U : ms;
}

uint32_t pcan_timestamp_us(void)
{
  return (uint32_t)pcan_timestamp_us64();
}

uint64_t pcan_timestamp_us64(void)
{
  if (s_cpu_freq_hz == 0U)
  {
    return 0ULL;
  }

  uint64_t cycle = hpm_csr_get_core_mcycle();
  uint64_t seconds = cycle / s_cpu_freq_hz;
  uint64_t remainder = cycle % s_cpu_freq_hz;

  return (seconds * 1000000ULL) + ((remainder * 1000000ULL) / s_cpu_freq_hz);
}
