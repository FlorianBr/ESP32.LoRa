/**
 ******************************************************************************
 *  file           : com.h
 *  brief          : LoRa communication
 ******************************************************************************
 *  Copyright (C) 2024 Florian Brandner
 */

#ifndef __COM_H_
#define __COM_H_

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/

#include <stdbool.h>

#include "lora_frame.h"

/* Private includes ----------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/
typedef struct {
  uint64_t id;
  uint16_t type;
  uint32_t uptime;
  uint8_t vmaj;
  uint8_t vmin;
} com_devicedata_t;

/* Exported constants --------------------------------------------------------*/

/* Exported macro ------------------------------------------------------------*/

/* Exported functions prototypes ---------------------------------------------*/

/**
 * @brief Initialise LoRa communication
 */
void com_init();

/**
 * @brief Check the Header of a received frame
 *
 * @param pHeader Pointer to the Header
 * @return true if valid, false otherwise
 */
bool com_checkHeader(const lora_frameheader_t* pHeader);

/**
 * @brief Parse a received lifesign message
 *
 * @param res Pointer to the ID response frame
 * @param pDvData Detected Device Data
 * @return true if ID detected, false on errors
 */
bool com_parse_msg_lifesign(const lora_id_response_t* res, com_devicedata_t* pDvData);

/**
 * @brief Transmits a lifesign request message
 *
 * @return true if send successfully
 */
bool com_tx_lifesign_req();

/**
 * @brief Wait for received data with timeout
 *
 * @param timeout Timeout in ms or 0 for no timeout
 * @param buffer Pointer to a rx data buffer
 * @param maxsize Max buffer size
 * @return uint16_t Size of Data received or 0 for timeout
 */
uint16_t com_wait4rx(uint32_t timeout, uint8_t* buffer, const uint16_t maxsize);

/**
 * @brief Aborts any blocking wait loop
 */
void com_waitabort();

#ifdef __cplusplus
}
#endif

#endif // __COM_H_