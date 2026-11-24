#include "at32f45x_int.h"

static volatile uint32_t g_systick_ms = 0U;

void NMI_Handler(void)
{
}

void HardFault_Handler(void)
{
  while (1)
  {
  }
}

void MemManage_Handler(void)
{
  while (1)
  {
  }
}

void BusFault_Handler(void)
{
  while (1)
  {
  }
}

void UsageFault_Handler(void)
{
  while (1)
  {
  }
}

void SVC_Handler(void)
{
}

void DebugMon_Handler(void)
{
}

void PendSV_Handler(void)
{
}

void SysTick_Handler(void)
{
  g_systick_ms++;
}

uint32_t app_get_tick_ms(void)
{
  return g_systick_ms;
}

void app_ensure_tick_nonzero(void)
{
  if (g_systick_ms == 0U)
  {
    g_systick_ms = 1U;
  }
}
