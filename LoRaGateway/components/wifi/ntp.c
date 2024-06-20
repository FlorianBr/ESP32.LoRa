/**
 ******************************************************************************
 *  file           : ntp.c
 *  brief          : Getting the time from a NTP Server
 ******************************************************************************
 *  Copyright (C) 2024 Florian Brandner
 */

/* Includes ------------------------------------------------------------------*/
#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_netif_sntp.h"
#include "esp_sntp.h"
#include "esp_system.h"

/* Private includes ----------------------------------------------------------*/

/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/

#define SYNC_INTERVAL (15 * 60 * 1000) //< Sync Interval in ms (15 Minutes)
#define NTP_SERVER "192.168.178.1"

/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

static const char* TAG = "NTP";

/* Private function prototypes -----------------------------------------------*/

/* Private user code ---------------------------------------------------------*/

void ntp_cb(struct timeval* tv) {
  time_t now;
  char strftime_buf[64];
  struct tm timeinfo;
  time(&now);
  localtime_r(&now, &timeinfo);
  strftime(strftime_buf, sizeof(strftime_buf), "%c", &timeinfo);
  ESP_LOGI(TAG, "NTP-Time received: %s", strftime_buf);
}

/* Public user code ----------------------------------------------------------*/

/**
 * @brief Get the time from a NTP Server and starts the NTP Client in the background
 *
 * @return esp_err_t
 */
esp_err_t NTP_Init(void) {
  const esp_sntp_config_t config = {
      .smooth_sync                = false,
      .server_from_dhcp           = false,
      .wait_for_sync              = true,
      .start                      = true,
      .sync_cb                    = ntp_cb,
      .renew_servers_after_new_IP = false,
      .ip_event_to_renew          = IP_EVENT_STA_GOT_IP,
      .index_of_first_server      = 0,
      .num_of_servers             = 1,
      .servers                    = ESP_SNTP_SERVER_LIST(NTP_SERVER),
  };
  esp_netif_sntp_init(&config);

  setenv("TZ", "CET-1CEST,M3.5.0,M10.5.0/3", 1);
  tzset();

  return (ESP_OK);
}