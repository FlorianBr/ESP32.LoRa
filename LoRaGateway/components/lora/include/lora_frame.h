/**
 ******************************************************************************
 *  file           : lora_frame.h
 *  brief          : The frames for LoRa transmissions
 ******************************************************************************
 *  Copyright (C) 2024 Florian Brandner
 */

#ifndef __LORA_FRAMES_H__
#define __LORA_FRAMES_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>

/* Private includes ----------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/

/* Exported constants --------------------------------------------------------*/

#define FRAME_VERSION 1   // Version of the frame protocol
#define LORA_MAX_SIZE 256 // Max. size for transmissions

#define DEVTYPE_WPAPER 0x0010 // Device Type: Heltec Wireless Paper

/* Exported macro ------------------------------------------------------------*/

/* Exported functions prototypes ---------------------------------------------*/

/**
 * @brief Type of frame
 */
typedef enum __attribute__((packed)) {
  TYPE_NONE     = 0,  // None
  TYPE_ID_REQ   = 10, // Request
  TYPE_ID_RES   = 20, // Response
  TYPE_ID_BCAST = 30, // Broadcast
} lora_frame_type_t;

/**
 * @brief Device types
 */
typedef enum __attribute__((packed)) {
  DEV_TYPE_NONE = 0, // None/unused
  DEV_TYPE_GATEWAY,  // Gateway to other worlds
  DEV_TYPE_ENDDEV    // End device
} lora_dev_type_t;

/**
 * @brief Device types
 */
typedef enum __attribute__((packed)) {
  DEV_CMD_NONE     = 0,  // None/unused
  DEV_CMD_LIFESIGN = 1,  // Lifesign
  DEV_CMD_DATAACC  = 10, // Data Access
  DEV_CMD_STATUS   = 30, // Status
} lora_command_t;

/**
 * @brief LoRa telegram frame header
 */
typedef struct __attribute__((packed)) {
  uint8_t version;         // Protocol version number
  lora_frame_type_t ftype; // Type of the frame
  lora_dev_type_t dtype;   // Generic device type
  lora_command_t cmd;      // Command
  uint64_t id;             // Unique ID
  uint16_t payloadlen;     // Length of payload
  uint8_t crc;             // Header-CRC (without payload and CRC)
} lora_frameheader_t;

/**
 * @brief A LoRa ID Response or Broadcast
 */
typedef struct __attribute__((packed)) {
  lora_frameheader_t header;
  uint16_t devtype; // Device Type
  uint8_t vmajor;   // Version: Major
  uint8_t vminor;   // Version: Minor
  uint32_t uptime;  // [s] Uptime
  uint8_t crc;      // CRC
} lora_id_response_t;

/**
 * @brief A LoRa data read/write request/response
 */
typedef struct __attribute__((packed)) {
  lora_frameheader_t header;
  uint8_t endpoint; // The data endpoint to access
  uint8_t cmd;      // The command for the endpoint
} lora_data_access_t;

#ifdef __cplusplus
}
#endif

#endif // __LORA_FRAMES_H__
