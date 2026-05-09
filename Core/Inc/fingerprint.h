/**
  ******************************************************************************
  * @file    fingerprint.h
  * @brief   AS608 Optical Fingerprint Sensor Driver (UART, 57600 baud default)
  *          For STM32 HAL on STM32F1xx (tested with USART1).
  *
  *  Wiring:
  *      AS608 VCC  -> 3.3V
  *      AS608 GND  -> GND
  *      AS608 TX   -> MCU USART_RX (e.g. PB7)
  *      AS608 RX   -> MCU USART_TX (e.g. PB6)
  *
  *  Default module address  : 0xFFFFFFFF
  *  Default module password : 0x00000000
  *  Default baud rate       : 57600
  *
  *  Packet format (all multi-byte fields MSB first):
  *      [Header 0xEF 0x01][Address 4B][PID 1B][Length 2B][Data N B][Checksum 2B]
  ******************************************************************************
  */

#ifndef __FINGERPRINT_H
#define __FINGERPRINT_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f1xx_hal.h"
#include <stdint.h>

/* ------------------------------------------------------------------ */
/*  Protocol constants                                                */
/* ------------------------------------------------------------------ */
#define AS608_HEADER_HI         0xEF
#define AS608_HEADER_LO         0x01

#define AS608_PID_COMMAND       0x01
#define AS608_PID_DATA          0x02
#define AS608_PID_ACK           0x07
#define AS608_PID_END_DATA      0x08

/* Instruction codes */
#define AS608_CMD_GET_IMAGE     0x01    /* Capture finger image           */
#define AS608_CMD_IMG_TO_TZ     0x02    /* Generate char file from image  */
#define AS608_CMD_MATCH         0x03    /* Compare two char files         */
#define AS608_CMD_SEARCH        0x04    /* Search the database            */
#define AS608_CMD_REG_MODEL     0x05    /* Combine two char files into 1  */
#define AS608_CMD_STORE         0x06    /* Store template in flash        */
#define AS608_CMD_LOAD_CHAR     0x07    /* Load template from flash       */
#define AS608_CMD_DELETE        0x0C    /* Delete one or more templates   */
#define AS608_CMD_EMPTY         0x0D    /* Empty entire fingerprint DB    */
#define AS608_CMD_VERIFY_PWD    0x13    /* Verify module password         */
#define AS608_CMD_GET_TEMPLATE_NUM 0x1D /* Read number of stored templates*/

/* Confirmation (return) codes from AS608 */
#define AS608_OK                0x00    /* Command executed successfully    */
#define AS608_ERR_PACKET        0x01    /* Error receiving data package     */
#define AS608_NO_FINGER         0x02    /* No finger on the sensor          */
#define AS608_ENROLL_FAIL       0x03    /* Failed to enroll the finger      */
#define AS608_IMAGE_DISORDERLY  0x06    /* Image too messy to extract feat. */
#define AS608_IMAGE_FEW_FEAT    0x07    /* Too few feature points           */
#define AS608_NOT_MATCH         0x08    /* Two finger templates do not match*/
#define AS608_NOT_FOUND         0x09    /* No matching finger in DB         */
#define AS608_FAIL_COMBINE      0x0A    /* Failed to combine char files     */
#define AS608_BAD_PAGE_ID       0x0B    /* Addressing PageID is out of range*/
#define AS608_FLASH_ERR         0x18    /* Error writing flash              */

/* Generic timeouts / sizes */
#define AS608_DEFAULT_ADDRESS   0xFFFFFFFFUL
#define AS608_DEFAULT_PASSWORD  0x00000000UL
#define AS608_TIMEOUT_DEFAULT   1500U   /* ms - default ack wait            */
#define AS608_TIMEOUT_IMAGE     5000U   /* ms - waiting for finger placement*/

/* ------------------------------------------------------------------ */
/*  Result struct returned by AS608_SearchFinger                       */
/* ------------------------------------------------------------------ */
typedef struct {
    uint16_t pageID;        /* Template ID where the match was found       */
    uint16_t matchScore;    /* Confidence score (higher = better match)    */
} AS608_SearchResult_t;


/* ------------------------------------------------------------------ */
/*  Public API                                                         */
/* ------------------------------------------------------------------ */

/**
 * @brief  Bind the driver to a HAL UART handle and verify communication.
 * @param  huart  Pointer to an already-initialized UART handle (57600 8N1).
 * @retval AS608_OK on success, error code otherwise.
 */
uint8_t AS608_Init(UART_HandleTypeDef *huart);

/**
 * @brief  Verify the module's 4-byte password.
 * @param  password  32-bit password (default 0x00000000).
 * @retval AS608_OK if password accepted.
 */
uint8_t AS608_VerifyPassword(uint32_t password);

/**
 * @brief  Tell the sensor to capture a fingerprint image.
 * @retval AS608_OK if image captured, AS608_NO_FINGER if no finger, etc.
 */
uint8_t AS608_GetImage(void);

/**
 * @brief  Convert the captured image into a character file in buffer.
 * @param  bufferID  1 or 2 - which character buffer to fill.
 */
uint8_t AS608_GenerateChar(uint8_t bufferID);

/**
 * @brief  Search the entire database for a finger matching CharBuffer1.
 * @param  result  Output: matched pageID and score (only valid on AS608_OK).
 * @retval AS608_OK on match, AS608_NOT_FOUND if no match, error otherwise.
 */
uint8_t AS608_SearchFinger(AS608_SearchResult_t *result);

/**
 * @brief  Combine CharBuffer1 and CharBuffer2 into a single template.
 *         Both buffers must already contain char files of the SAME finger.
 */
uint8_t AS608_GenerateTemplate(void);

/**
 * @brief  Store the template in CharBuffer1 to flash slot pageID.
 * @param  pageID  Slot in the on-sensor library (0..library_size-1).
 */
uint8_t AS608_StoreTemplate(uint16_t pageID);

/**
 * @brief  Delete one template at pageID.
 */
uint8_t AS608_DeleteTemplate(uint16_t pageID);

/**
 * @brief  Erase the entire fingerprint database on the sensor.
 */
uint8_t AS608_EmptyDatabase(void);

/**
 * @brief  Read how many templates are currently stored in the sensor.
 * @param  count  Output pointer.
 */
uint8_t AS608_GetTemplateCount(uint16_t *count);

/**
 * @brief  Convenience: full enrollment pipeline for one finger.
 *         Caller is expected to display prompts ("place finger", "remove",
 *         "place again") between callbacks. This function blocks until the
 *         user complies or the per-step timeout expires.
 *
 *         Step 1: wait for finger -> capture -> char file 1
 *         Step 2: wait for removal
 *         Step 3: wait for finger -> capture -> char file 2
 *         Step 4: combine into template
 *         Step 5: store at pageID
 *
 * @param  pageID            Slot to store at.
 * @param  on_state_change   Optional callback (may be NULL) invoked with a
 *                           short status string at each user-visible step.
 * @retval AS608_OK on success or any error code from the underlying call.
 */
typedef void (*AS608_StatusCallback)(const char *status);
uint8_t AS608_EnrollFinger(uint16_t pageID, AS608_StatusCallback on_state_change);

/**
 * @brief  Convenience: capture finger, search DB, return match.
 * @param  result            Output: matched pageID + score.
 * @param  finger_timeout_ms How long to wait for a finger to be placed.
 * @retval AS608_OK on match, AS608_NO_FINGER on timeout, AS608_NOT_FOUND etc.
 */
uint8_t AS608_IdentifyFinger(AS608_SearchResult_t *result,
                             uint32_t finger_timeout_ms);

#ifdef __cplusplus
}
#endif
#endif /* __FINGERPRINT_H */
