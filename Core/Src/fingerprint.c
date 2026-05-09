/**
  ******************************************************************************
  * @file    fingerprint.c
  * @brief   AS608 Optical Fingerprint Sensor driver - implementation.
  *
  *  Reference: AS608 / R30X / R503 instruction set, "GT-511C series" style
  *  packet protocol used by all common ZhianTec fingerprint modules.
  ******************************************************************************
  */

#include "fingerprint.h"
#include <string.h>

/* ------------------------------------------------------------------ */
/*  Module state                                                       */
/* ------------------------------------------------------------------ */
static UART_HandleTypeDef *as608_uart   = NULL;
static uint32_t            as608_addr   = AS608_DEFAULT_ADDRESS;

/* Working RX buffer - large enough for any ack packet we expect.      */
#define AS608_RX_BUF_SIZE   32
static uint8_t as608_rx_buf[AS608_RX_BUF_SIZE];

/* ------------------------------------------------------------------ */
/*  Low-level helpers                                                  */
/* ------------------------------------------------------------------ */

/**
 * @brief  Send one command packet.
 * @param  data    Pointer to the instruction code + parameters.
 * @param  length  Number of bytes pointed to by `data`.
 * @retval HAL status (HAL_OK on success).
 *
 * Frame layout sent on the wire:
 *   EF 01 | <addr 4B BE> | 01 (PID command) | <len 2B BE> | data... | <chk 2B BE>
 *   where len = length(data) + 2 (the checksum bytes themselves).
 */
static HAL_StatusTypeDef AS608_SendCommand(const uint8_t *data, uint16_t length)
{
    if (as608_uart == NULL) return HAL_ERROR;

    uint8_t  packet[64];                       /* 11 header + up to data */
    uint16_t pkt_len = (uint16_t)(length + 2); /* +2 for checksum        */
    uint16_t idx     = 0;
    uint16_t checksum;

    /* Header */
    packet[idx++] = AS608_HEADER_HI;
    packet[idx++] = AS608_HEADER_LO;

    /* Address (big-endian) */
    packet[idx++] = (uint8_t)(as608_addr >> 24);
    packet[idx++] = (uint8_t)(as608_addr >> 16);
    packet[idx++] = (uint8_t)(as608_addr >>  8);
    packet[idx++] = (uint8_t)(as608_addr      );

    /* PID + Length */
    packet[idx++] = AS608_PID_COMMAND;
    packet[idx++] = (uint8_t)(pkt_len >> 8);
    packet[idx++] = (uint8_t)(pkt_len     );

    /* Payload */
    if (length > 0 && ((uint32_t)(idx + length + 2)) <= sizeof(packet)) {
        memcpy(&packet[idx], data, length);
        idx = (uint16_t)(idx + length);
    } else if (length > 0) {
        return HAL_ERROR;   /* would overflow packet buffer */
    }

    /* Checksum: sum of (PID + length-bytes + data), low 16 bits */
    checksum = 0;
    /* indices 6..(idx-1) cover PID, length high, length low, payload */
    for (uint16_t i = 6; i < idx; i++) {
        checksum = (uint16_t)(checksum + packet[i]);
    }
    packet[idx++] = (uint8_t)(checksum >> 8);
    packet[idx++] = (uint8_t)(checksum     );

    return HAL_UART_Transmit(as608_uart, packet, idx, 500);
}

/**
 * @brief  Receive an ack packet from the sensor.
 * @param  payload      Output buffer for data after the confirmation code.
 *                      May be NULL if `payload_max` is 0.
 * @param  payload_max  Capacity of `payload`.
 * @param  timeout_ms   Per-byte read timeout in milliseconds.
 * @retval The confirmation code (first byte of ack data) on success,
 *         or 0xFE on framing/timeout error (distinct from valid codes).
 *
 * The standard ack frame is:
 *   EF 01 | addr 4B | 07 (PID ack) | len 2B | confirm 1B [+payload] | chk 2B
 */
static uint8_t AS608_ReceiveAck(uint8_t *payload, uint16_t payload_max,
                                uint16_t timeout_ms)
{
    if (as608_uart == NULL) return 0xFE;

    /* Read fixed 9-byte header (header + addr + PID + len). */
    if (HAL_UART_Receive(as608_uart, as608_rx_buf, 9, timeout_ms) != HAL_OK)
        return 0xFE;

    if (as608_rx_buf[0] != AS608_HEADER_HI ||
        as608_rx_buf[1] != AS608_HEADER_LO)
        return 0xFE;

    if (as608_rx_buf[6] != AS608_PID_ACK)
        return 0xFE;

    uint16_t pkt_len = (uint16_t)((as608_rx_buf[7] << 8) | as608_rx_buf[8]);
    if (pkt_len < 3 || pkt_len > AS608_RX_BUF_SIZE)
        return 0xFE;

    /* Read the rest: pkt_len bytes (confirmation + payload + 2 checksum). */
    if (HAL_UART_Receive(as608_uart, as608_rx_buf, pkt_len, timeout_ms) != HAL_OK)
        return 0xFE;

    uint8_t  confirm  = as608_rx_buf[0];
    uint16_t data_len = (uint16_t)(pkt_len - 3); /* exclude confirm + chk */

    /* Copy out any extra payload bytes that follow the confirmation code. */
    if (payload != NULL && payload_max > 0 && data_len > 0) {
        uint16_t to_copy = (data_len > payload_max) ? payload_max : data_len;
        memcpy(payload, &as608_rx_buf[1], to_copy);
    }

    return confirm;
}

/* ------------------------------------------------------------------ */
/*  Public API                                                         */
/* ------------------------------------------------------------------ */

uint8_t AS608_Init(UART_HandleTypeDef *huart)
{
    if (huart == NULL) return 0xFE;
    as608_uart = huart;
    as608_addr = AS608_DEFAULT_ADDRESS;

    /* Drain anything left in the RX FIFO by overrun / boot noise. */
    __HAL_UART_CLEAR_OREFLAG(as608_uart);

    /* Many AS608 modules need ~200ms after power-up before responding. */
    HAL_Delay(250);

    return AS608_VerifyPassword(AS608_DEFAULT_PASSWORD);
}

uint8_t AS608_VerifyPassword(uint32_t password)
{
    uint8_t cmd[5];
    cmd[0] = AS608_CMD_VERIFY_PWD;
    cmd[1] = (uint8_t)(password >> 24);
    cmd[2] = (uint8_t)(password >> 16);
    cmd[3] = (uint8_t)(password >>  8);
    cmd[4] = (uint8_t)(password      );

    if (AS608_SendCommand(cmd, sizeof(cmd)) != HAL_OK) return 0xFE;
    return AS608_ReceiveAck(NULL, 0, AS608_TIMEOUT_DEFAULT);
}

uint8_t AS608_GetImage(void)
{
    uint8_t cmd = AS608_CMD_GET_IMAGE;
    if (AS608_SendCommand(&cmd, 1) != HAL_OK) return 0xFE;
    return AS608_ReceiveAck(NULL, 0, AS608_TIMEOUT_DEFAULT);
}

uint8_t AS608_GenerateChar(uint8_t bufferID)
{
    if (bufferID != 1 && bufferID != 2) return 0xFE;
    uint8_t cmd[2] = { AS608_CMD_IMG_TO_TZ, bufferID };
    if (AS608_SendCommand(cmd, sizeof(cmd)) != HAL_OK) return 0xFE;
    return AS608_ReceiveAck(NULL, 0, AS608_TIMEOUT_DEFAULT);
}

uint8_t AS608_SearchFinger(AS608_SearchResult_t *result)
{
    if (result == NULL) return 0xFE;

    /* Search across the full library (start page 0, count 0xFFFF). */
    uint8_t cmd[6];
    cmd[0] = AS608_CMD_SEARCH;
    cmd[1] = 0x01;          /* search using CharBuffer1                  */
    cmd[2] = 0x00; cmd[3] = 0x00;   /* start page = 0                    */
    cmd[4] = 0xFF; cmd[5] = 0xFF;   /* count = 65535 (entire library)    */

    if (AS608_SendCommand(cmd, sizeof(cmd)) != HAL_OK) return 0xFE;

    uint8_t payload[4] = {0};   /* pageID(2) + matchScore(2) */
    uint8_t confirm = AS608_ReceiveAck(payload, sizeof(payload),
                                       AS608_TIMEOUT_DEFAULT);

    if (confirm == AS608_OK) {
        result->pageID     = (uint16_t)((payload[0] << 8) | payload[1]);
        result->matchScore = (uint16_t)((payload[2] << 8) | payload[3]);
    } else {
        result->pageID     = 0;
        result->matchScore = 0;
    }
    return confirm;
}

uint8_t AS608_GenerateTemplate(void)
{
    uint8_t cmd = AS608_CMD_REG_MODEL;
    if (AS608_SendCommand(&cmd, 1) != HAL_OK) return 0xFE;
    return AS608_ReceiveAck(NULL, 0, AS608_TIMEOUT_DEFAULT);
}

uint8_t AS608_StoreTemplate(uint16_t pageID)
{
    uint8_t cmd[4];
    cmd[0] = AS608_CMD_STORE;
    cmd[1] = 0x01;                      /* source CharBuffer1            */
    cmd[2] = (uint8_t)(pageID >> 8);
    cmd[3] = (uint8_t)(pageID     );
    if (AS608_SendCommand(cmd, sizeof(cmd)) != HAL_OK) return 0xFE;
    return AS608_ReceiveAck(NULL, 0, AS608_TIMEOUT_DEFAULT);
}

uint8_t AS608_DeleteTemplate(uint16_t pageID)
{
    uint8_t cmd[5];
    cmd[0] = AS608_CMD_DELETE;
    cmd[1] = (uint8_t)(pageID >> 8);
    cmd[2] = (uint8_t)(pageID     );
    cmd[3] = 0x00; cmd[4] = 0x01;       /* count = 1 template            */
    if (AS608_SendCommand(cmd, sizeof(cmd)) != HAL_OK) return 0xFE;
    return AS608_ReceiveAck(NULL, 0, AS608_TIMEOUT_DEFAULT);
}

uint8_t AS608_EmptyDatabase(void)
{
    uint8_t cmd = AS608_CMD_EMPTY;
    if (AS608_SendCommand(&cmd, 1) != HAL_OK) return 0xFE;
    return AS608_ReceiveAck(NULL, 0, AS608_TIMEOUT_DEFAULT);
}

uint8_t AS608_GetTemplateCount(uint16_t *count)
{
    if (count == NULL) return 0xFE;

    uint8_t cmd = AS608_CMD_GET_TEMPLATE_NUM;
    if (AS608_SendCommand(&cmd, 1) != HAL_OK) return 0xFE;

    uint8_t payload[2] = {0};
    uint8_t confirm = AS608_ReceiveAck(payload, sizeof(payload),
                                       AS608_TIMEOUT_DEFAULT);
    if (confirm == AS608_OK) {
        *count = (uint16_t)((payload[0] << 8) | payload[1]);
    } else {
        *count = 0;
    }
    return confirm;
}

/* ------------------------------------------------------------------ */
/*  Convenience high-level routines                                    */
/* ------------------------------------------------------------------ */

/* Block until the sensor returns AS608_OK from GetImage, or until
 * `timeout_ms` elapses. Returns AS608_OK on capture, AS608_NO_FINGER on
 * timeout, or any other error code. Polls every 100 ms.                */
static uint8_t AS608_WaitForFinger(uint32_t timeout_ms)
{
    uint32_t start = HAL_GetTick();
    while ((HAL_GetTick() - start) < timeout_ms) {
        uint8_t r = AS608_GetImage();
        if (r == AS608_OK)        return AS608_OK;
        if (r == AS608_NO_FINGER) { HAL_Delay(100); continue; }
        return r;   /* any other error - bail out */
    }
    return AS608_NO_FINGER;
}

/* Block until the user removes their finger, or until timeout_ms.      */
static void AS608_WaitForRemoval(uint32_t timeout_ms)
{
    uint32_t start = HAL_GetTick();
    while ((HAL_GetTick() - start) < timeout_ms) {
        if (AS608_GetImage() == AS608_NO_FINGER) return;
        HAL_Delay(100);
    }
}

uint8_t AS608_IdentifyFinger(AS608_SearchResult_t *result,
                             uint32_t finger_timeout_ms)
{
    if (result == NULL) return 0xFE;

    uint8_t r = AS608_WaitForFinger(finger_timeout_ms);
    if (r != AS608_OK) return r;

    r = AS608_GenerateChar(1);
    if (r != AS608_OK) return r;

    return AS608_SearchFinger(result);
}

uint8_t AS608_EnrollFinger(uint16_t pageID, AS608_StatusCallback cb)
{
    uint8_t r;

    if (cb) cb("Place finger");
    r = AS608_WaitForFinger(AS608_TIMEOUT_IMAGE);
    if (r != AS608_OK) return r;

    r = AS608_GenerateChar(1);
    if (r != AS608_OK) return r;

    if (cb) cb("Remove finger");
    AS608_WaitForRemoval(AS608_TIMEOUT_IMAGE);
    HAL_Delay(500);

    if (cb) cb("Place again");
    r = AS608_WaitForFinger(AS608_TIMEOUT_IMAGE);
    if (r != AS608_OK) return r;

    r = AS608_GenerateChar(2);
    if (r != AS608_OK) return r;

    if (cb) cb("Combining");
    r = AS608_GenerateTemplate();
    if (r != AS608_OK) return r;

    if (cb) cb("Storing");
    return AS608_StoreTemplate(pageID);
}
