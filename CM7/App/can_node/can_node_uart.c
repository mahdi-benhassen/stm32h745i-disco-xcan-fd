/* SPDX-License-Identifier: MIT */
#include "can_node_uart.h"

#include <stdarg.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>

#include "usart.h"

#define UART_RING_SIZE  1024U
#define UART_PRINTF_BUF 192U

static uint8_t s_uart_ring[UART_RING_SIZE];
static uint32_t s_uart_head;
static uint32_t s_uart_tail;
static uint32_t s_uart_dropped;

static uint32_t ring_used(void)
{
    if (s_uart_head >= s_uart_tail) {
        return s_uart_head - s_uart_tail;
    }
    return UART_RING_SIZE - (s_uart_tail - s_uart_head);
}

static bool ring_push(uint8_t ch)
{
    uint32_t next = (s_uart_head + 1U) % UART_RING_SIZE;
    if (next == s_uart_tail) {
        s_uart_dropped++;
        return false;
    }
    s_uart_ring[s_uart_head] = ch;
    s_uart_head = next;
    return true;
}

void can_node_uart_init(void)
{
    (void)memset(s_uart_ring, 0, sizeof(s_uart_ring));
    s_uart_head = 0U;
    s_uart_tail = 0U;
    s_uart_dropped = 0U;
}

void can_node_uart_write_str(const char *str)
{
    if (str == NULL) {
        return;
    }
    for (uint32_t i = 0U; str[i] != '\0'; i++) {
        if (!ring_push((uint8_t)str[i])) {
            break;
        }
    }
}

void can_node_uart_printf(const char *fmt, ...)
{
    char buf[UART_PRINTF_BUF];
    va_list args;

    if (fmt == NULL) {
        return;
    }

    va_start(args, fmt);
    int written = vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);

    if (written > 0) {
        can_node_uart_write_str(buf);
    }
}

void can_node_uart_poll(void)
{
    if ((huart3.Instance != NULL) && (s_uart_tail != s_uart_head) &&
        ((huart3.Instance->ISR & USART_ISR_TXE_TXFNF) != 0U)) {
        huart3.Instance->TDR = s_uart_ring[s_uart_tail];
        s_uart_tail = (s_uart_tail + 1U) % UART_RING_SIZE;
    }
}

void can_node_uart_flush(uint32_t timeout_ms)
{
    uint32_t start_ms = HAL_GetTick();

    while ((ring_used() > 0U) && ((uint32_t)(HAL_GetTick() - start_ms) < timeout_ms)) {
        can_node_uart_poll();
    }

    if (huart3.Instance != NULL) {
        while (((huart3.Instance->ISR & USART_ISR_TC) == 0U) &&
               ((uint32_t)(HAL_GetTick() - start_ms) < timeout_ms)) {
        }
    }
}
