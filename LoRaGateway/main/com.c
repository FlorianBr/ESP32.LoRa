/**
 ******************************************************************************
 *  file           : com.c
 *  brief          : LoRa communication
 ******************************************************************************
 *  Copyright (C) 2024 Florian Brandner
 */

/* Includes ------------------------------------------------------------------*/
#include "com.h"

#include <stdio.h>
#include <string.h>

#include "lora.h"
#include "sdkconfig.h"

/* Private includes ----------------------------------------------------------*/

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

static const char* TAG = "COM";
static bool abort_wait = false;

/* Private function prototypes -----------------------------------------------*/

uint8_t crc8(const uint8_t* data, const size_t len);

/* Private user code ---------------------------------------------------------*/

// Calculate CRC8 with start value 0xFF
uint8_t crc8(const uint8_t* data, const size_t len) {
  uint8_t crc = 0xff;
  for (size_t i = 0; i < len; i++) {
    crc ^= data[i];
    for (size_t j = 0; j < 8; j++) {
      if ((crc & 0x80) != 0)
        crc = (uint8_t)((crc << 1) ^ 0x31);
      else
        crc <<= 1;
    }
  }
  return crc;
}

/* Public user code ----------------------------------------------------------*/

void com_init() {
  if (lora_init() == 0) {
    ESP_LOGE(TAG, "LoRa module not found!");
    return;
  }
  lora_set_frequency(866e6); // 866MHz
  lora_enable_crc();

  int cr = 5; // Coding Rate 4:5
  int bw = 7; // Bandwidth 125kHz
  int sf = 7; // Spreading Factor

  lora_set_coding_rate(cr);
  lora_set_bandwidth(bw);
  lora_set_spreading_factor(sf);
}

bool com_checkHeader(const lora_frameheader_t* pHeader) {
  ESP_LOGD(TAG, "Checking Header");

  if (pHeader->version != FRAME_VERSION) {
    ESP_LOGW(TAG, "Wrong header version");
    return false;
  }
  switch (pHeader->ftype) {
    case TYPE_ID_REQ:
      ESP_LOGD(TAG, "   Frame Type = %d (Request)", pHeader->ftype);
      break;
    case TYPE_ID_RES:
      ESP_LOGD(TAG, "   Frame Type = %d (Response)", pHeader->ftype);
      break;
    case TYPE_ID_BCAST:
      ESP_LOGD(TAG, "   Frame Type = %d (Broadcast)", pHeader->ftype);
      break;
    default:
      ESP_LOGW(TAG, "   Frame Type = %d (Unknown)", pHeader->ftype);
      return false;
      break;
  }

  switch (pHeader->dtype) {
    case DEV_TYPE_GATEWAY:
      ESP_LOGD(TAG, "  Device Type = %d (Gateway)", pHeader->dtype);
      break;
    case DEV_TYPE_ENDDEV:
      ESP_LOGD(TAG, "  Device Type = %d (End Device)", pHeader->dtype);
      break;
    default:
      ESP_LOGW(TAG, "  Device Type = %d (Unknown)", pHeader->dtype);
      return false;
      break;
  }

  switch (pHeader->cmd) {
    case DEV_CMD_LIFESIGN:
      ESP_LOGD(TAG, "      Command = %d (Lifesign)", pHeader->cmd);
      break;
    case DEV_CMD_DATAACC:
      ESP_LOGD(TAG, "      Command = %d (Read Data)", pHeader->cmd);
      break;
    default:
      ESP_LOGW(TAG, "      Command = %d (Unknown)", pHeader->cmd);
      return false;
      break;
  }
  ESP_LOGD(TAG, "   Payloadlen = %d", pHeader->payloadlen);

  // ID 0 is ignored
  if (pHeader->id == 0) {
    return false;
  }
  ESP_LOGD(TAG, "   SystemID = 0x%llx", pHeader->id);

  // TODO: Check CRC
  ESP_LOGD(TAG, "   Header CRC = %d", pHeader->crc);

  return true;
}

bool com_parse_msg_lifesign(const lora_id_response_t* res, com_devicedata_t* pDvData) {
  ESP_LOGD(TAG, "Parsing LifeSign");

  if (pDvData == NULL) {
    ESP_LOGW(TAG, "NULL pointer for device data!");
    return false;
  }

  if (res->header.payloadlen != (sizeof(lora_id_response_t) - sizeof(lora_frameheader_t))) {
    ESP_LOGW(TAG, "Illegal payload length %d", res->header.payloadlen);
    return false;
  }

  // TODO: Check CRC

  ESP_LOGD(TAG, "      Type = %x", res->devtype);
  ESP_LOGD(TAG, "   Version = %d.%d", res->vmajor, res->vminor);
  ESP_LOGD(TAG, "    Uptime = %ld", res->uptime);

  pDvData->id     = res->header.id;
  pDvData->type   = res->devtype;
  pDvData->uptime = res->uptime;
  pDvData->vmaj   = res->vmajor;
  pDvData->vmin   = res->vminor;
  return true;
}

bool com_tx_lifesign_req() {
  lora_frameheader_t frame;

  memset((uint8_t*)&frame, 0, sizeof(lora_frameheader_t));

  frame.version    = FRAME_VERSION;
  frame.ftype      = TYPE_ID_REQ;
  frame.dtype      = DEV_TYPE_GATEWAY;
  frame.cmd        = DEV_CMD_LIFESIGN;
  frame.id         = 0;
  frame.payloadlen = 0;
  frame.crc        = crc8((uint8_t*)&frame, sizeof(lora_frameheader_t) - 1);

  lora_set_tx_power(2);
  lora_send_packet((uint8_t*)&frame, sizeof(lora_frameheader_t));
  ESP_LOGD(TAG, "Transmitted Lifesign Request, %d packets lost", lora_packet_lost());

  return true;
}

uint16_t com_wait4rx(uint32_t timeout, uint8_t* buffer, const uint16_t maxsize) {
  const uint32_t aborttime = xTaskGetTickCount() + pdMS_TO_TICKS(timeout);

  lora_receive();
  while (lora_received() == 0) {
    if ((timeout > 0) && (xTaskGetTickCount() >= aborttime)) {
      ESP_LOGD(TAG, "com_wait4rx: Timeout waiting for RX");
      return 0;
    }
    if (abort_wait) {
      abort_wait = false;
      ESP_LOGD(TAG, "com_wait4rx: Wait abort requested");
      return 0;
    }
    vTaskDelay(10); // Called from the main loop, a yield would not trigger the WDT!
  }

  int rxLen = lora_receive_packet(buffer, maxsize);

  ESP_LOGD(TAG, "com_wait4rx: %d bytes of data received", rxLen);
  ESP_LOG_BUFFER_HEXDUMP(TAG, buffer, rxLen, ESP_LOG_DEBUG);

  return rxLen;
}

void com_waitabort() {
  abort_wait = true;
}

bool com_tx_cmd(const uint64_t id, const uint8_t cmd, const uint8_t endpoint, const uint8_t payloadsize,
                const char* payload) {
  lora_data_access_t* pFrame;
  char txbuffer[LORA_MAX_SIZE];
  size_t txlength = 0;

  ESP_LOGI(TAG, "Transmitting Command 0x%x to Endpoint %d for 0x%llx", cmd, endpoint, id);

  pFrame = (lora_data_access_t*)&txbuffer[0];

  pFrame->header.version    = FRAME_VERSION;
  pFrame->header.ftype      = TYPE_ID_REQ;
  pFrame->header.dtype      = DEV_TYPE_GATEWAY;
  pFrame->header.cmd        = DEV_CMD_DATAACC;
  pFrame->header.id         = id;
  pFrame->header.payloadlen = (sizeof(lora_data_access_t) - sizeof(lora_frameheader_t)) + payloadsize;
  pFrame->header.crc        = crc8((uint8_t*)pFrame, sizeof(lora_frameheader_t) - 1);
  pFrame->endpoint          = endpoint;
  pFrame->cmd               = cmd;

  txlength = sizeof(lora_data_access_t);

  if ((payloadsize > 0) && (NULL != payload)) {
    ESP_LOGI(TAG, "Transmitting additional %d byte of payload", payloadsize);
    memcpy(&txbuffer[txlength], payload, payloadsize);
    txlength += payloadsize;
  }

  if (txlength > LORA_MAX_SIZE) {
    ESP_LOGE(TAG, "Transmission size too big!");
    return false;
  } else {
    lora_set_tx_power(2);
    lora_send_packet((uint8_t*)&txbuffer[0], txlength);
    ESP_LOGI(TAG, "Transmitted Command Frame with %d byte, %d packets lost", txlength, lora_packet_lost());
  }
  return true;
}