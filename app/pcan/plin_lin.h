#pragma once
#include <stdint.h>

/* HPM5300EVK LIN physical channel 0 on HPM_UART3:
 * PA14 = UART3_RXD, PA15 = UART3_TXD, PA13 = LIN3_SLP_N, PA24 = LIN 12V_EN.
 */

#define PLIN_LIN_MODE_NONE   0U
#define PLIN_LIN_MODE_SLAVE  1U
#define PLIN_LIN_MODE_MASTER 2U

void plin_lin_init(void);
void plin_lin_poll(void);
void plin_lin_set_config(uint8_t channel, uint16_t baudrate, uint8_t mode);
void plin_lin_set_active(uint8_t channel, uint8_t active);
void plin_lin_send_wakeup(uint8_t channel);
void plin_lin_write(uint8_t channel, uint8_t pid, uint8_t length, uint8_t direction,
                    uint8_t checksum_type, const uint8_t *data);
