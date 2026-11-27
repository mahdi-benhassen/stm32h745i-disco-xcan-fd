/* SPDX-License-Identifier: MIT */
#include "can_node_backend.h"

#include <stddef.h>
#include <string.h>

#include "can_node_config.h"
#include "can_node_time.h"
#include "crc/crc.h"

/* Place the 64 KB virtual flash slot in AXI-SRAM D1 so DTCMRAM stays nearly empty. */
__attribute__((section(".ram_d1_bss"),
               aligned(32))) static uint8_t s_dl_slot[CAN_NODE_DL_SLOT_SIZE];

static const dtc_event_config_t k_dtc_table[DTC_EVENTS_MAX] = {
    /* 0..5: confirmed (threshold 1) -> status 0x2B (CONFIRMED | TF | TFTOC | PDTC | TFSLC). */
    {0xC10001U, 1, -1, 1U, 1U, 1U, false, true},
    {0xC10102U, 1, -1, 1U, 1U, 1U, false, true},
    {0xC12100U, 1, -1, 1U, 1U, 1U, false, false},
    {0xD00116U, 1, -1, 1U, 1U, 1U, false, false},
    {0x056216U, 1, -1, 1U, 1U, 1U, false, false}, /* SAE J2012 P0562-16 */
    {0xC10087U, 1, -1, 1U, 1U, 1U, false, true},  /* SAE J2012 U0100-87 */
    /* 6..9: pending only (threshold 2, 1 failure) -> status 0x27 (PENDING | TF). */
    {0xC20001U, 1, -1, 1U, 1U, 2U, false, false},
    {0xC20002U, 1, -1, 1U, 1U, 2U, false, false},
    {0xC20003U, 1, -1, 1U, 1U, 2U, false, false},
    {0xC20004U, 1, -1, 1U, 1U, 2U, false, false},
    /* 10..31: passed DTCs -> full 0x19 0x0A reportSupportedDTC = 3 + 32*4 = 131 bytes. */
    {0xE00101U, 1, -1, 1U, 1U, 1U, false, false},
    {0xE00102U, 1, -1, 1U, 1U, 1U, false, false},
    {0xE00103U, 1, -1, 1U, 1U, 1U, false, false},
    {0xE00104U, 1, -1, 1U, 1U, 1U, false, false},
    {0xE00105U, 1, -1, 1U, 1U, 1U, false, false},
    {0xE00106U, 1, -1, 1U, 1U, 1U, false, false},
    {0xE00107U, 1, -1, 1U, 1U, 1U, false, false},
    {0xE00108U, 1, -1, 1U, 1U, 1U, false, false},
    {0xE00109U, 1, -1, 1U, 1U, 1U, false, false},
    {0xE0010AU, 1, -1, 1U, 1U, 1U, false, false},
    {0xE0010BU, 1, -1, 1U, 1U, 1U, false, false},
    {0xE0010CU, 1, -1, 1U, 1U, 1U, false, false},
    {0xE0010DU, 1, -1, 1U, 1U, 1U, false, false},
    {0xE0010EU, 1, -1, 1U, 1U, 1U, false, false},
    {0xE0010FU, 1, -1, 1U, 1U, 1U, false, false},
    {0xE00110U, 1, -1, 1U, 1U, 1U, false, false},
    {0xE00111U, 1, -1, 1U, 1U, 1U, false, false},
    {0xE00112U, 1, -1, 1U, 1U, 1U, false, false},
    {0xE00113U, 1, -1, 1U, 1U, 1U, false, false},
    {0xE00114U, 1, -1, 1U, 1U, 1U, false, false},
    {0xE00115U, 1, -1, 1U, 1U, 1U, false, false},
    {0xE00116U, 1, -1, 1U, 1U, 1U, false, false},
};

static uint32_t packed_version(void)
{
    return (((uint32_t)CAN_NODE_SW_VER_MAJOR & 0xFFU) << 24U) |
           (((uint32_t)CAN_NODE_SW_VER_MINOR & 0xFFU) << 16U) |
           ((uint32_t)CAN_NODE_SW_VER_PATCH & 0xFFFFU);
}

static uds_be_t be_ecu_reset(void *ctx, uds_call_t call, uint8_t reset_type, uint8_t *nrc)
{
    can_node_backend_t *be = (can_node_backend_t *)ctx;
    (void)call;

    if ((be == NULL) || (reset_type != UDS_RESET_HARD)) {
        *nrc = UDS_NRC_SUBFUNCTION_UNSUPPORTED;
    } else {
        be->reset_requested = true;
        *nrc = UDS_NRC_NONE;
    }
    return UDS_BE_DONE;
}

static uds_be_t be_boot_info(void *ctx, uds_call_t call, uds_boot_info_t *info, uint8_t *nrc)
{
    const can_node_backend_t *be = (const can_node_backend_t *)ctx;
    (void)call;

    if ((be == NULL) || (info == NULL)) {
        *nrc = UDS_NRC_GENERAL_REJECT;
        return UDS_BE_DONE;
    }

    (void)memset(info, 0, sizeof(*info));
    info->handoff = true;
    info->slot = be->active_slot;
    info->mode = (be->confirm_state == 3U) ? 1U : 2U; /* 1 CONFIRMED, 2 TRIAL */
    info->attempt = (be->confirm_state == 3U) ? 0U : 1U;
    info->event = (uint8_t)BOOT_EVENT_COMMIT;
    info->event_slot = be->active_slot;
    info->confirm_state = be->confirm_state;
    info->image_version = packed_version();
    *nrc = UDS_NRC_NONE;
    return UDS_BE_DONE;
}

static uds_be_t be_dtc_snapshot(void *ctx, uds_call_t call, const dtc_state_t **state, uint8_t *nrc)
{
    const can_node_backend_t *be = (const can_node_backend_t *)ctx;
    (void)call;

    if ((be == NULL) || (state == NULL)) {
        *nrc = UDS_NRC_GENERAL_REJECT;
        return UDS_BE_DONE;
    }

    *state = &be->dtc;
    *nrc = UDS_NRC_NONE;
    return UDS_BE_DONE;
}

static uds_be_t be_dtc_clear(void *ctx, uds_call_t call, uint32_t group, uint8_t *nrc)
{
    can_node_backend_t *be = (can_node_backend_t *)ctx;
    (void)call;

    if ((be == NULL) || (group != UDS_DTC_GROUP_ALL)) {
        *nrc = UDS_NRC_REQUEST_OUT_OF_RANGE;
        return UDS_BE_DONE;
    }

    (void)dtc_clear_all(&be->dtc);
    *nrc = UDS_NRC_NONE;
    return UDS_BE_DONE;
}

static uds_be_t be_erase(void *ctx, uds_call_t call, uint32_t address, uint32_t size,
                         boot_status_t *status)
{
    can_node_backend_t *be = (can_node_backend_t *)ctx;

    if (be == NULL) {
        *status = BOOT_E_ARG;
        return UDS_BE_DONE;
    }

    if (call == UDS_CALL_START) {
        if ((address != CAN_NODE_DL_SLOT_ADDR) || (size == 0U) || (size > CAN_NODE_DL_SLOT_SIZE)) {
            *status = BOOT_E_SIZE;
            return UDS_BE_DONE;
        }
        be->erase_size = size;
        be->written_bytes = 0U;
        be->running_crc = CRC32_INIT;
        be->slot_erased = false;
        be->erase_start_us = can_node_time_us32();
        be->erasing = (CAN_NODE_ERASE_SIM_US > 0U);
    }

    if (be->erasing) {
        const uint32_t elapsed = can_node_time_us32() - be->erase_start_us;
        if (elapsed < CAN_NODE_ERASE_SIM_US) {
            return UDS_BE_PENDING;
        }
        be->erasing = false;
    }

    (void)memset(s_dl_slot, 0xFF, be->erase_size);
    be->slot_erased = true;
    *status = BOOT_OK;
    return UDS_BE_DONE;
}

static uds_be_t be_write(void *ctx, uds_call_t call, uint32_t offset, const uint8_t *data,
                         uint32_t len, boot_status_t *status)
{
    can_node_backend_t *be = (can_node_backend_t *)ctx;
    (void)call;

    if ((be == NULL) || (data == NULL) || !be->slot_erased) {
        *status = BOOT_E_SEQUENCE;
        return UDS_BE_DONE;
    }
    if ((offset > CAN_NODE_DL_SLOT_SIZE) || (len > (CAN_NODE_DL_SLOT_SIZE - offset))) {
        *status = BOOT_E_SIZE;
        return UDS_BE_DONE;
    }

    (void)memcpy(&s_dl_slot[offset], data, len);
    be->running_crc = crc32_update(be->running_crc, data, len);
    be->written_bytes = offset + len;
    *status = BOOT_OK;
    return UDS_BE_DONE;
}

static uds_be_t be_transfer_exit(void *ctx, uds_call_t call, boot_status_t *status)
{
    can_node_backend_t *be = (can_node_backend_t *)ctx;
    (void)call;

    if ((be == NULL) || !be->slot_erased) {
        *status = BOOT_E_SEQUENCE;
        return UDS_BE_DONE;
    }

    be->last_crc32 = be->running_crc ^ CRC32_XOROUT;
    be->last_dl_size = be->written_bytes;
    *status = BOOT_BUSY;
    return UDS_BE_DONE;
}

static uds_be_t be_check(void *ctx, uds_call_t call, boot_status_t *status)
{
    can_node_backend_t *be = (can_node_backend_t *)ctx;
    (void)call;

    if ((be == NULL) || (be->last_dl_size == 0U)) {
        *status = BOOT_E_SEQUENCE;
        return UDS_BE_DONE;
    }

    const uint32_t full_crc = crc32_compute(s_dl_slot, be->last_dl_size);
    if (full_crc != be->last_crc32) {
        *status = BOOT_E_INVALID;
        return UDS_BE_DONE;
    }

    be->active_slot = 1U;
    be->confirm_state = 1U; /* WAITING confirmation via 0x31 01 F0 00 */
    be->download_count++;
    *status = BOOT_OK;
    return UDS_BE_DONE;
}

static uds_be_t be_confirm(void *ctx, uds_call_t call, boot_status_t *status)
{
    can_node_backend_t *be = (can_node_backend_t *)ctx;
    (void)call;

    if (be == NULL) {
        *status = BOOT_E_ARG;
        return UDS_BE_DONE;
    }

    if (be->confirm_state != 1U) {
        *status = BOOT_E_STATE;
        return UDS_BE_DONE;
    }

    be->confirm_state = 3U; /* CONFIRMED */
    *status = BOOT_OK;
    return UDS_BE_DONE;
}

bool can_node_backend_init(can_node_backend_t *be, uds_backend_t *out_cbs)
{
    if ((be == NULL) || (out_cbs == NULL)) {
        return false;
    }

    (void)memset(be, 0, sizeof(*be));
    (void)memset(s_dl_slot, 0xFF, sizeof(s_dl_slot));
    be->active_slot = 0U;
    be->confirm_state = 3U; /* Start CONFIRMED; 0x31 01 FF 01 transitions to WAITING (TRIAL). */

    if (!dtc_init(&be->dtc, k_dtc_table, (uint16_t)DTC_EVENTS_MAX)) {
        return false;
    }

    for (uint16_t i = 0U; i < 10U; i++) {
        (void)dtc_report(&be->dtc, i, DTC_REPORT_FAILED);
    }
    for (uint16_t i = 10U; i < (uint16_t)DTC_EVENTS_MAX; i++) {
        (void)dtc_report(&be->dtc, i, DTC_REPORT_PASSED);
    }

    out_cbs->ctx = be;
    out_cbs->ecu_reset = be_ecu_reset;
    out_cbs->boot_info = be_boot_info;
    out_cbs->dtc_snapshot = be_dtc_snapshot;
    out_cbs->dtc_clear = be_dtc_clear;
    out_cbs->erase = be_erase;
    out_cbs->write = be_write;
    out_cbs->transfer_exit = be_transfer_exit;
    out_cbs->check = be_check;
    out_cbs->confirm = be_confirm;
    return true;
}
