/* SPDX-License-Identifier: MIT */
#include "can_node_time.h"

#include "tim.h"

static volatile uint32_t s_time_hi_us;
static volatile uint8_t s_time_ready;

void can_node_time_init(void)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();

    if (s_time_ready == 0U) {
        s_time_hi_us = 0U;

        /* HAL_TIM_Base_Init() triggers EGR.UG, which latches SR.UIF. Clear it before arming UIE
         * so the first IRQ only fires when CNT truly wraps past 0xFFFFFFFF (71.58 minutes). */
        TIM2->SR = ~((uint32_t)TIM_SR_UIF);
        (void)TIM2->SR;

        TIM2->DIER |= TIM_DIER_UIE;
        if ((TIM2->CR1 & TIM_CR1_CEN) == 0U) {
            TIM2->CR1 |= TIM_CR1_CEN;
        }
        s_time_ready = 1U;
    }

    __set_PRIMASK(primask);

    HAL_NVIC_SetPriority(TIM2_IRQn, 2U, 0U);
    HAL_NVIC_ClearPendingIRQ(TIM2_IRQn);
    HAL_NVIC_EnableIRQ(TIM2_IRQn);
}

uint64_t can_node_time_us64(void)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();

    uint32_t hi = s_time_hi_us;
    uint32_t lo = TIM2->CNT;
    if (((TIM2->SR & TIM_SR_UIF) != 0U) && (lo < 0x80000000U)) {
        hi++;
    }

    __set_PRIMASK(primask);
    return (((uint64_t)hi) << 32U) | (uint64_t)lo;
}

uint32_t can_node_time_us32(void)
{
    return TIM2->CNT;
}

/* Overrides the weak startup vector in startup_stm32h745xx_CM7.s. */
void TIM2_IRQHandler(void)
{
    if ((TIM2->SR & TIM_SR_UIF) != 0U) {
        TIM2->SR = ~((uint32_t)TIM_SR_UIF);
        s_time_hi_us++;
        /* Flush APB1 write buffer before exiting ISR on Cortex-M7 to avoid tail-chained
         * spurious re-entry. */
        (void)TIM2->SR;
        __DSB();
    }
}
