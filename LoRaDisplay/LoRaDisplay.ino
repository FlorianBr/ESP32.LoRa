/**
 ******************************************************************************
 *  file           : LoRaDisplay.ino
 *  brief          : Arduino Firmware for Heltec Wireless Paper
 ******************************************************************************
 *  Copyright (C) 2024 Florian Brandner
 */

/* Includes ------------------------------------------------------------------*/
// #include <BH1750.h>
// #include <BMP180.h>
// #include <ESP32_LoRaWan_102.h>
// #include <ESP32_Mcu.h>
// #include <GXHTC.h>
// #include <HT_Display.h>
// #include <HT_DisplayFonts.h>
// #include <HT_DisplayUi.h>
// #include <HT_QYEG0213RWS800_BWR.h>
// #include <HT_SH1107Wire.h>
// #include <HT_SSD1306Spi.h>
// #include <HT_SSD1306Wire.h>
// #include <HT_TinyGPS++.h>
// #include <HT_lCMEN2R13EFC1.h>
// #include <HT_st7735.h>
// #include <HT_st7735_fonts.h>
// #include <HT_st7736.h>
// #include <da217.h>
// #include <esp_clk_internal.h>
// #include <heltec.h>
#include <esp32-hal-log.h>

/* Private includes ----------------------------------------------------------*/

#include "Arduino.h"
#include "CRC8.h"
#include "LoRaWan_APP.h"
#include "lora_frame.h"

/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/

// Enable ESP logging
#ifdef CORE_DEBUG_LEVEL
  #undef CORE_DEBUG_LEVEL
#endif
#ifdef LOG_LOCAL_LEVEL
  #undef LOG_LOCAL_LEVEL
#endif
#define CORE_DEBUG_LEVEL 4
#define LOG_LOCAL_LEVEL CORE_DEBUG_LEVEL
#define CORE_DEBUG_LEVEL 3
#define LOG_LOCAL_LEVEL ESP_LOG_DEBUG

// Generic settings
#define VERSION_MAJOR 0
#define VERSION_MINOR 1

// LoRa Settings
#define RF_FREQUENCY 866000000         // Hz
#define TX_OUTPUT_POWER 5              // dBm
#define LORA_BANDWIDTH 0               // 125 kHz,
#define LORA_SPREADING_FACTOR 7        // SF7
#define LORA_CODINGRATE 1              // 4/5
#define LORA_PREAMBLE_LENGTH 8         // Same for Tx and Rx
#define LORA_ID_CYCLE (15 * 60 * 1000) // [ms] Cycle time for ID transmission

#define LIFESIGN_DELAY_MIN 100  // [ms] Min delay for a lifesign response
#define LIFESIGN_DELAY_MAX 5000 // [ms] Max delay for a lifesign response
#define LORA_LOOP_TIME 125      // [ms] Loop time of the LoRa Worker Task

/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

static const char* TAG = "MAIN";
static RadioEvents_t RadioEvents;          // Possible events of the LoRa radio
static uint64_t systemId          = 0;     // Unique system ID
static TaskHandle_t LoRaWorkerHdl = NULL;  // Handle to the RX/TX worker task
volatile bool lora_idle           = false; // LoRa Transceiver semaphore TODO: Make atomic

/* Private function prototypes -----------------------------------------------*/

void lora_OnTxDone(void);
void lora_OnRxDone(void);
void lora_OnTxTimeout(void);
void lora_OnRxTimeout(void);
void lora_OnRxError(void);
void lora_init();
void lora_Worker(void* pvParameters);
bool lora_tx_id_frame(const bool wasRequested);
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

/**
 * @brief Init the LoRa Transceiver
 */
void lora_init() {
  Mcu.begin(HELTEC_BOARD, SLOW_CLK_TPYE);

  RadioEvents.TxDone    = lora_OnTxDone;
  RadioEvents.TxTimeout = lora_OnTxTimeout;
  RadioEvents.RxDone    = lora_OnRxDone;
  RadioEvents.RxTimeout = lora_OnRxTimeout;
  RadioEvents.RxError   = lora_OnRxError;

  Radio.Init(&RadioEvents);
  Radio.SetChannel(RF_FREQUENCY);
  Radio.SetTxConfig(MODEM_LORA, TX_OUTPUT_POWER, 0, LORA_BANDWIDTH, LORA_SPREADING_FACTOR, LORA_CODINGRATE,
                    LORA_PREAMBLE_LENGTH, false, true, 0, 0, false, 3000);

  lora_idle = true;
}

/**
 * @brief Transmit a LoRa ID Frame
 *
 * @param wasRequested Response to a ID request?
 */
bool lora_tx_id_frame(const bool wasRequested) {
  if (!lora_idle) {
    ESP_LOGE(TAG, "LoRa is NOT idle!");
    return false;
  }
  lora_idle = false;

  lora_id_response_t framedata;
  framedata.header.version    = FRAME_VERSION;
  framedata.header.ftype      = (wasRequested ? TYPE_ID_RES : TYPE_ID_BCAST);
  framedata.header.dtype      = DEV_TYPE_ENDDEV;
  framedata.header.cmd        = DEV_CMD_LIFESIGN;
  framedata.header.payloadlen = sizeof(lora_id_response_t) - sizeof(lora_frameheader_t);
  framedata.header.crc        = 0xFF;
  framedata.id                = systemId;
  framedata.devtype           = DEVTYPE_WPAPER;
  framedata.vmajor            = VERSION_MAJOR;
  framedata.vminor            = VERSION_MINOR;
  framedata.uptime            = (xTaskGetTickCount() * configTICK_RATE_HZ) / 1000000;
  framedata.crc               = 0xFF;

  // Header CRC
  CRC8 crc;
  crc.add((uint8_t*)&framedata, sizeof(lora_frameheader_t) - 1);
  framedata.header.crc = crc.calc();

  // Frame CRC
  crc.reset();
  crc.add((uint8_t*)&framedata, sizeof(lora_id_response_t) - 1);
  framedata.crc = crc.calc();

  ESP_LOGI(TAG, "Transmitting ID Frame with %d byte", sizeof(lora_id_response_t));
  Radio.Send((uint8_t*)&framedata, sizeof(lora_id_response_t));

  return true;
}

// LoRa Callback: TX done
void lora_OnTxDone() {
  ESP_LOGD(TAG, "TX done");
  lora_idle = true;
}

// LoRa Callback: TX Timeout
void lora_OnTxTimeout() {
  Radio.Sleep();
  ESP_LOGW(TAG, "TX timeout");
  lora_idle = true;
}

// LoRa Callback: RX Timeout
void lora_OnRxTimeout() {
  Radio.Sleep();
  ESP_LOGW(TAG, "RX timeout");
  lora_idle = true;
}

// LoRa Callback: RX Error
void lora_OnRxError() {
  Radio.Sleep();
  ESP_LOGW(TAG, "RX Error");
  lora_idle = true;
}

// LoRa Callback: RX
void lora_OnRxDone(uint8_t* payload, uint16_t size, int16_t rssi, int8_t snr) {
  Radio.Sleep();
  lora_idle = true;

  ESP_LOGI(TAG, "RX: rssi %d, len %d, snr %d", rssi, size, snr);
  const lora_frameheader_t* pHeader = (lora_frameheader_t*)payload;

  if (size < sizeof(lora_frameheader_t)) {
    ESP_LOGE(TAG, "Received data shorter than possible");
    return;
  }
  if (pHeader->version != FRAME_VERSION) {
    ESP_LOGW(TAG, "Frame version not supported");
    return;
  }
  if (pHeader->dtype == DEV_TYPE_ENDDEV) {
    ESP_LOGW(TAG, "Device to Device communication not supported ");
    return;
  }
#if 0 // TODO: Enable CRC check
  uint8_t crc = crc8((uint8_t*)pHeader, sizeof(lora_frameheader_t) - 1);
  if (pHeader->crc != crc) {
    ESP_LOGE(TAG, "CRC error, expected %x but got %x", crc, pHeader->crc);
    return;
  }
#endif

  ESP_LOGI(TAG, "Received Header:");
  ESP_LOGI(TAG, "   Version: %d", pHeader->version);
  ESP_LOGI(TAG, "     FType: %d", pHeader->ftype);
  ESP_LOGI(TAG, "     DType: %d", pHeader->dtype);
  ESP_LOGI(TAG, "       CMD: %d", pHeader->cmd);
  ESP_LOGI(TAG, "   Payload: %d", pHeader->payloadlen);
  ESP_LOGI(TAG, "       CRC: %d", pHeader->crc);

  if (pHeader->ftype == TYPE_ID_REQ) {
    if (pHeader->cmd == DEV_CMD_LIFESIGN) {
      ESP_LOGI(TAG, "Lifesign request received");

      // TODO: Instead of a delay this should be enqueued while continue listening
      // TODO: Alternative approach: Check if channel is free before sending
      const uint16_t delaytime = random(LIFESIGN_DELAY_MIN, LIFESIGN_DELAY_MAX);
      ESP_LOGI(TAG, "Delaying %d ms", delaytime);
      vTaskDelay(pdMS_TO_TICKS(delaytime));
      lora_tx_id_frame(true);
    } else if (pHeader->cmd == DEV_CMD_RDATA) {
      ESP_LOGI(TAG, "Read Data request received");
      // TODO: Implement me!
    } else if (pHeader->cmd == DEV_CMD_WDATA) {
      ESP_LOGI(TAG, "Write Data request received");
      // TODO: Implement me!
    } else if (pHeader->cmd == DEV_CMD_STATUS) {
      ESP_LOGI(TAG, "Status request received");
      // TODO: Implement me!
    } else {
      ESP_LOGW(TAG, "Command %d not supported", pHeader->cmd);
    }
  } else {
    ESP_LOGW(TAG, "Frame type %d not supported", pHeader->ftype);
  }
}

// Task for LoRa RX/TX
void lora_Worker(void* pvParameters) {
  uint32_t runtime = LORA_ID_CYCLE; // Trigger a RX right at the start
  ESP_LOGI(TAG, "LoRa Worker started");
  while (1) {
    // Every LORA_ID_CYCLE: Abort RX and send cyclic ID
    if (runtime >= LORA_ID_CYCLE) {
      runtime = 0;
      Radio.Standby();
      lora_idle = true;
      if (!lora_tx_id_frame(false)) {
        ESP_LOGW(TAG, "lora_tx_id_frame FAILED, retrying!");
      }
    }

    // Go into RX mode
    if (Radio.GetStatus() == RF_IDLE) {
      ESP_LOGI(TAG, "Radio set to RX");
      lora_idle = false;
      Radio.Rx(0);
    }

    Radio.IrqProcess();

    vTaskDelay(LORA_LOOP_TIME / portTICK_PERIOD_MS);
    runtime += LORA_LOOP_TIME;
  }
}

/* Public user code ----------------------------------------------------------*/

// Arduino SETUP
void setup() {
  Mcu.begin(HELTEC_BOARD, SLOW_CLK_TPYE);
  lora_init();

  systemId = ESP.getEfuseMac(); // use the mac-address as system ID

  delay(2000); // Delay to enable activation of serial monitor

  ESP_LOGI(TAG, "Chip type: %s Rev %d, %d cores", ESP.getChipModel(), ESP.getChipRevision(), ESP.getChipCores());
  ESP_LOGI(TAG, "Chip ID: 0x%x", systemId);

  xTaskCreate(lora_Worker, "LoRa", 4096, NULL, tskIDLE_PRIORITY, &LoRaWorkerHdl);
  configASSERT(LoRaWorkerHdl);
}

// Arduino LOOP
void loop() {
  vTaskDelay(500 / portTICK_PERIOD_MS);
}
