#pragma once
#include <stdint.h>

/* PLIN-USB (MI_01) endpoints matching PCAN-USB Pro FD descriptors */
#define PLIN_EP_CMDOUT        0x04U
#define PLIN_EP_CMDIN         0x84U
#define PLIN_EP_DATA0OUT      0x05U
#define PLIN_EP_DATA0IN       0x85U
#define PLIN_EP_DATA1OUT      0x06U
#define PLIN_EP_DATA1IN       0x86U

#define PLIN_EP_SIZE          64U
#define PLIN_CHANNELS         2U
#define PLIN_RX_MSG_SIZE      24U
#define PLIN_WRITE_MSG_SIZE   36U

#define PLIN_MAX_FRAMES       64U
#define PLIN_MAX_SLOTS        8U
#define PLIN_MAX_SCHEDULES    2U
#define PLIN_FRAME_ENTRY_SIZE 16U
#define PLIN_SLOT_SIZE        20U
#define PLIN_RX_QUEUE_LEN     32U

void plin_protocol_init(void);
void plin_protocol_poll(void);
void plin_protocol_process_command(const uint8_t *buf, uint16_t size);
void plin_protocol_process_data(uint8_t channel, const uint8_t *buf, uint16_t size);
void plin_protocol_on_tx_done(uint8_t channel);
void plin_protocol_on_driver_state(uint8_t loaded);
void plin_protocol_report_msg(uint8_t channel, const uint8_t *msg, uint16_t size);
void plin_protocol_set_frame_data(uint8_t channel, uint8_t id, const uint8_t *data, uint8_t length);

uint8_t plin_protocol_get_device_no(void);
uint8_t plin_protocol_get_client_no(void);
uint8_t plin_protocol_get_break_len(uint8_t channel);

int plin_protocol_get_frame(uint8_t channel, uint8_t id, uint8_t *entry);
int plin_protocol_get_slot(uint8_t channel, uint8_t index, uint8_t *slot);
uint8_t plin_protocol_get_slot_count(uint8_t channel);
void plin_protocol_set_schedule_active(uint8_t channel, uint8_t active);
