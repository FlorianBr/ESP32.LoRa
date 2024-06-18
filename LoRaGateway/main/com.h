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
 * @param res Pointer to the frame
 */
void com_parse_msg_lifesign(lora_id_response_t* res);

/**
 * @brief Transmits a lifesign request message
 *
 * @return true if send successfully
 */
bool com_tx_lifesign_req();

/**
 * @brief Wait for received data with timeout
 *
 * @param timeout Timeout in ms
 * @param buffer Pointer to a rx data buffer
 * @param maxsize Max buffer size
 * @return uint16_t Size of Data received or 0 for timeout
 */
uint16_t com_wait4rx(uint32_t timeout, const uint8_t* buffer, const uint16_t maxsize);

#ifdef __cplusplus
}
#endif

#endif // __COM_H_