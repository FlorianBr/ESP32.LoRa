/**
 ******************************************************************************
 *  file           : devicelist.c
 *  brief          : Device list management
 ******************************************************************************
 *  Copyright (C) 2024 Florian Brandner
 */

/* Includes ------------------------------------------------------------------*/
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "sdkconfig.h"

/* Private includes ----------------------------------------------------------*/
#include "devicelist.h"

/* Private typedef -----------------------------------------------------------*/

typedef struct {
  uint64_t id;
  int timestamp;
  bool isAlive;
} devicelist_entry_t;

/* Private define ------------------------------------------------------------*/

#define MAX_DEVICELIST 20               // Max. number of devices in the list
#define DEV_FORGET_TIME (5 * 60 * 1000) // [ms] Time after devices are forgotten

/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

TaskHandle_t xHdlDevListWorker = NULL;
static devicelist_entry_t devicelist[MAX_DEVICELIST];

static const char* TAG = "DLST";

/* Private function prototypes -----------------------------------------------*/

void worker(void* pvParameters);
void clearEntry(size_t entry);

/* Private user code ---------------------------------------------------------*/

// Sets a entry to defaults
void clearEntry(size_t entry) {
  if (entry < MAX_DEVICELIST) {
    devicelist[entry].id        = 0;
    devicelist[entry].timestamp = 0;
    devicelist[entry].isAlive   = false;
  }
}

// Manages the devicelist
void worker(void* pvParameters) {
  while (true) {
    vTaskDelay(pdMS_TO_TICKS(30000));

    for (size_t i = 0; i < MAX_DEVICELIST; i++) {
      // Seen too long ago?
      if (devicelist[i].isAlive && (xTaskGetTickCount() - devicelist[i].timestamp > pdMS_TO_TICKS(DEV_FORGET_TIME))) {
        ESP_LOGI(TAG, "Forgetting device %d with ID %llx", i, devicelist[i].id);
        clearEntry(i);
      }
      // Number of known devices
    }

    // Display list of devices
    ESP_LOGI(TAG, "Known Devices:%d", devlist_known());
    for (size_t i = 0; i < MAX_DEVICELIST; i++) {
      if (devicelist[i].isAlive) {
        int seen = xTaskGetTickCount() - devicelist[i].timestamp;
        seen     = seen * configTICK_RATE_HZ; // Tick to ms
        ESP_LOGI(TAG, "   %02d %llx %d", i, devicelist[i].id, (seen / 1000));
      }
    }
  }
}

/* Public user code ----------------------------------------------------------*/

bool devlist_adddevice(const uint64_t id) {
  size_t dlpos      = MAX_DEVICELIST;
  size_t dlfirstpos = MAX_DEVICELIST;
  bool newDevice    = true;

  // Already in the devicelist?
  for (size_t i = 0; i < MAX_DEVICELIST; i++) {
    // Check if already known
    if (devicelist[i].id == id) {
      dlpos     = i;
      newDevice = false;
      break;
    }
    // Remember first unused entry
    if ((!devicelist[i].isAlive) && (i < dlfirstpos)) {
      dlfirstpos = i;
    }
  }
  if (dlpos == MAX_DEVICELIST) {       // Not found?
    if (dlfirstpos < MAX_DEVICELIST) { // and unused position?
      dlpos = dlfirstpos;
    } else {
      ESP_LOGW(TAG, "Unable to add device %llx, list is full!", id);
      return false;
    }
  }

  // Update entry
  devicelist[dlpos].id        = id;
  devicelist[dlpos].isAlive   = true;
  devicelist[dlpos].timestamp = xTaskGetTickCount();
  ESP_LOGI(TAG, "Device %llx %s at pos %d", id, (newDevice ? "added" : "updated"), dlpos);

  return true;
}

void devlist_init() {
  // Clear the devicelist
  for (size_t i = 0; i < MAX_DEVICELIST; i++) {
    clearEntry(i);
  }

  // Start the DeviceList worker
  xTaskCreate(worker, "DevList", 2048, NULL, tskIDLE_PRIORITY, &xHdlDevListWorker);
  configASSERT(xHdlDevListWorker);
}

uint8_t devlist_known() {
  uint8_t known = 0;
  for (size_t i = 0; i < MAX_DEVICELIST; i++) {
    if (devicelist[i].isAlive) {
      known++;
    }
  }
  return known;
}
