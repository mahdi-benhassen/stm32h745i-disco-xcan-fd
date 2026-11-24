#ifndef AT32F45X_INT_H
#define AT32F45X_INT_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "at32f45x.h"

void NMI_Handler(void);
void HardFault_Handler(void);
void MemManage_Handler(void);
void BusFault_Handler(void);
void UsageFault_Handler(void);
void SVC_Handler(void);
void DebugMon_Handler(void);
void PendSV_Handler(void);
void SysTick_Handler(void);

uint32_t app_get_tick_ms(void);
void app_ensure_tick_nonzero(void);

#ifdef __cplusplus
}
#endif

#endif /* AT32F45X_INT_H */
