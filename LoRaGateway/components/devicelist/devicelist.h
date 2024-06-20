/**
 ******************************************************************************
 *  file           : devicelist.h
 *  brief          : Device list management
 ******************************************************************************
 *  Copyright (C) 2024 Florian Brandner
 */

#ifndef DEVICELIST_H_
#define DEVICELIST_H_

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/

/* Private includes ----------------------------------------------------------*/
#include <stdint.h>

/* Exported types ------------------------------------------------------------*/

/* Exported constants --------------------------------------------------------*/

/* Exported macro ------------------------------------------------------------*/

/* Exported functions prototypes ---------------------------------------------*/

/**
 * @brief Add/Update a device in the list
 *
 * @param id The devices ID
 * @return true if added, false on error
 */
bool devlist_adddevice(const uint64_t id);

/**
 * @brief Init the device list module
 *
 */
void devlist_init();

/**
 * @brief Returns the list of known devices
 *
 * @return uint8_t
 */
uint8_t devlist_known();

#ifdef __cplusplus
}
#endif

#endif // DEVICELIST_H_