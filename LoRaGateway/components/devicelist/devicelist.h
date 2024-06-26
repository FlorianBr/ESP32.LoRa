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

// Callback for new or removed devices
typedef void (*dev_cb_t)(const uint64_t id);

// The callbacks for new or deleted devices
typedef struct {
  dev_cb_t new_cb;
  dev_cb_t del_cd;
} device_cbs_t;

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

/**
 * @brief Gets the ID of a specific entry
 *
 * @param entry Number of the entry
 * @param Id The devices ID
 * @return true if valid, false otherwise
 */
bool devlist_getEntryId(const uint8_t entry, uint64_t* Id);

/**
 * @brief Set the callbacks when a device is added or deleted from the list
 *
 * @param cbs The callback structure
 */
void devlist_setcbs(const device_cbs_t cbs);

#ifdef __cplusplus
}
#endif

#endif // DEVICELIST_H_