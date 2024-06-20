/**
 ******************************************************************************
 *  file           : main.c
 *  brief          : MQTT Connector for LoRa Devices
 ******************************************************************************
 *  Copyright (C) 2024 Florian Brandner
 */

/* Includes ------------------------------------------------------------------*/
#include <stdio.h>

/* Private includes ----------------------------------------------------------*/
#include "com.h"
#include "devicelist.h"
#include "esp_chip_info.h"
#include "esp_log.h"
#include "esp_ota_ops.h"
#include "esp_partition.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "lcd.h"
#include "nvs_flash.h"
#include "sdkconfig.h"

/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/

#define LIFESIGN_CYCLE (60 * 1000) // [ms] Cycle time for lifesign request broadcasts
#define MAX_FRAME_SIZE 512         // Max. Size of frame we can handle
#define SEMA_TIMEOUT 1000          // [ms] Timeout when waiting for semaphores

/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

static const char* TAG = "MAIN";
static esp_partition_t* part_info;
static esp_ota_img_states_t ota_state;

TaskHandle_t xHdlRXWorker = NULL;
TaskHandle_t xHdlTXWorker = NULL;

SemaphoreHandle_t xSemaCom = NULL; // Semaphore for communication resource

/* Private function prototypes -----------------------------------------------*/

void rx_worker(void* pvParameters);
void tx_worker(void* pvParameters);

/* Private user code ---------------------------------------------------------*/

// Handle RX
void rx_worker(void* pvParameters) {
  uint8_t Rxbuffer[MAX_FRAME_SIZE];

  while (true) {
    if (xSemaphoreTake(xSemaCom, pdMS_TO_TICKS(SEMA_TIMEOUT)) == pdTRUE) {
      const uint16_t rxlen = com_wait4rx(0, &Rxbuffer[0], MAX_FRAME_SIZE); // Wait for received data (BLOCKING!)
      xSemaphoreGive(xSemaCom);

      if (rxlen > 0) {
        ESP_LOGI(TAG, "Received %d byte", rxlen);
        if (rxlen < sizeof(lora_frameheader_t)) {
          ESP_LOGW(TAG, "Received size too small");
        } else {
          const lora_frameheader_t* pHeader = (lora_frameheader_t*)&Rxbuffer;
          if (com_checkHeader(pHeader)) {
            if ((pHeader->ftype == TYPE_ID_RES) ||   // Lifesign response
                (pHeader->ftype == TYPE_ID_BCAST)) { // Lifesign broadcast
              com_devicedata_t DevData;
              if (com_parse_msg_lifesign((const lora_id_response_t*)Rxbuffer, &DevData)) {
                if (devlist_adddevice(DevData.id)) {
                  // TODO: Publish device data to MQTT
                }
              } else {
                ESP_LOGW(TAG, "Unable to parse lifesign message");
              }
            } else {
              ESP_LOGW(TAG, "No Parser for this frame available");
            }
          } else {
            ESP_LOGW(TAG, "Header invalid");
          }
        }
      }
    } else {
      ESP_LOGW(TAG, "Unable to get semaphore!");
      vTaskDelay(pdMS_TO_TICKS(250));
    }

    vTaskDelay(10);
  }
}

// Handle TX
void tx_worker(void* pvParameters) {
  vTaskDelay(pdMS_TO_TICKS(1000)); // 1s start-up delay
  while (true) {
    ESP_LOGI(TAG, "Sending lifesign");

    com_waitabort();
    if (xSemaphoreTake(xSemaCom, pdMS_TO_TICKS(SEMA_TIMEOUT)) == pdTRUE) {
      com_tx_lifesign_req(); // Send lifesign request
      xSemaphoreGive(xSemaCom);
    } else {
      ESP_LOGW(TAG, "Unable to get semaphore!");
    }

    vTaskDelay(pdMS_TO_TICKS(LIFESIGN_CYCLE));
  }
}

/* Public user code ----------------------------------------------------------*/

void app_main(void) {
  esp_err_t ret = ESP_OK;

  // Get chip info
  esp_chip_info_t chip_info;
  esp_chip_info(&chip_info);

  // Core info
  ESP_LOGW(TAG, "-------------------------------------");
  ESP_LOGW(TAG, "System Info:");
  ESP_LOGW(TAG, "%s chip with %d CPU cores, WiFi%s%s%s%s%s%s, ", CONFIG_IDF_TARGET, chip_info.cores,
           (chip_info.features & CHIP_FEATURE_BT) ? "/BT" : "", (chip_info.features & CHIP_FEATURE_BLE) ? "/BLE" : "",
           (chip_info.features & CHIP_FEATURE_EMB_FLASH) ? "/FLASH" : "",
           (chip_info.features & CHIP_FEATURE_WIFI_BGN) ? "/WiFi" : "",
           (chip_info.features & CHIP_FEATURE_IEEE802154) ? "/WPAN" : "",
           (chip_info.features & CHIP_FEATURE_EMB_PSRAM) ? "/PSRAM" : "");
  ESP_LOGW(TAG, "Heap: %lu", esp_get_free_heap_size());
  ESP_LOGW(TAG, "Reset reason: %d", esp_reset_reason());
  ESP_LOGW(TAG, "-------------------------------------");

  // Partition info
  part_info = (esp_partition_t*)esp_ota_get_boot_partition();
  esp_ota_get_state_partition(part_info, &ota_state);
  if (NULL != part_info) {
    ESP_LOGW(TAG, "Current partition:");
    ESP_LOGW(TAG, "    Label = %s, state = %d", part_info->label, ota_state);
    ESP_LOGW(TAG, "    Address=0x%lx, size=0x%lx", part_info->address, part_info->size);
  }
  ESP_LOGW(TAG, "-------------------------------------");

  // Initialize NVS, format it if necessary
  ret = nvs_flash_init();
  if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    ESP_LOGW(TAG, "Erasing NVS!");
    ESP_ERROR_CHECK(nvs_flash_erase());
    ret = nvs_flash_init();
  }
  ESP_ERROR_CHECK(ret);
  ESP_LOGI(TAG, "NVS init returned %d", ret);

  // Print out NVS statistics
  nvs_stats_t nvs_stats;
  nvs_get_stats("nvs", &nvs_stats);
  ESP_LOGW(TAG, "-------------------------------------");
  ESP_LOGW(TAG, "NVS Statistics:");
  ESP_LOGW(TAG, "NVS Used = %d", nvs_stats.used_entries);
  ESP_LOGW(TAG, "NVS Free = %d", nvs_stats.free_entries);
  ESP_LOGW(TAG, "NVS All = %d", nvs_stats.total_entries);

  nvs_iterator_t iter = NULL;
  esp_err_t res       = nvs_entry_find("nvs", NULL, NVS_TYPE_ANY, &iter);
  while (res == ESP_OK) {
    nvs_entry_info_t info;
    nvs_entry_info(iter, &info);
    ESP_LOGW(TAG, "Key '%s', Type '%d'", info.key, info.type);
    res = nvs_entry_next(&iter);
  }
  nvs_release_iterator(iter);
  ESP_LOGW(TAG, "-------------------------------------");

  lcd_init();
  lcd_settext1("Starting");
  com_init();

  vSemaphoreCreateBinary(xSemaCom);
  configASSERT(xSemaCom);

  devlist_init();

  // Start the RX worker
  xTaskCreate(rx_worker, "RX", 4096, NULL, tskIDLE_PRIORITY, &xHdlRXWorker);
  configASSERT(xHdlRXWorker);

  // Start the TX worker
  xTaskCreate(tx_worker, "TX", 4096, NULL, tskIDLE_PRIORITY, &xHdlTXWorker);
  configASSERT(xHdlTXWorker);

  lcd_settext1("Running");
  while (1) {
    lcd_settext2("Devices: ", devlist_known());
    vTaskDelay(1000);
  }
}
