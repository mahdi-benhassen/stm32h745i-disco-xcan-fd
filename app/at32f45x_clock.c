#include "at32f45x_clock.h"

/**
 * Configure system clock tree for AT-START-F457 (8 MHz HEXT):
 * - PLL VCO = (8 MHz * 120) / 1 = 960 MHz
 * - SCLK    = 960 MHz / 6 = 160 MHz (AHB = 160 MHz, APB1 = 160 MHz, APB2 = 160 MHz, APB3 = 80 MHz)
 * - PLLU    = 960 MHz / 20 = 48.0 MHz exact for OTGFS1 USB device
 * - CAN1    = PCLK (160 MHz = 2 * 80 MHz PCAN reference clock)
 */
void system_clock_config(void)
{
  crm_reset();

  flash_psr_set(FLASH_WAIT_CYCLE_5);

  crm_periph_clock_enable(CRM_PWC_PERIPH_CLOCK, TRUE);
  pwc_ldo_output_voltage_set(PWC_LDO_OUTPUT_1V3);

  crm_clock_source_enable(CRM_CLOCK_SOURCE_HEXT, TRUE);
  while (crm_hext_stable_wait() == ERROR)
  {
  }

  crm_pll_config(CRM_PLL_SOURCE_HEXT, 120U, 1U, CRM_PLL_FP_6);
  crm_pllu_div_set(CRM_PLL_FU_20);
  crm_pllu_output_set(TRUE);

  crm_clock_source_enable(CRM_CLOCK_SOURCE_PLL, TRUE);
  while (crm_flag_get(CRM_PLL_STABLE_FLAG) != SET)
  {
  }

  crm_ahb_div_set(CRM_AHB_DIV_1);
  crm_apb3_div_set(CRM_APB3_DIV_2);
  crm_apb2_div_set(CRM_APB2_DIV_1);
  crm_apb1_div_set(CRM_APB1_DIV_1);

  crm_auto_step_mode_enable(TRUE);
  crm_sysclk_switch(CRM_SCLK_PLL);
  while (crm_sysclk_switch_status_get() != CRM_SCLK_PLL)
  {
  }
  crm_auto_step_mode_enable(FALSE);

  system_core_clock_update();
}
