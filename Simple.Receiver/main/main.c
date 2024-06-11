#include <stdio.h>

#include "driver/i2c_master.h"
#include "esp_chip_info.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_panel_vendor.h"
#include "esp_log.h"
#include "esp_lvgl_port.h"
#include "esp_ota_ops.h"
#include "esp_partition.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "lora.h"
#include "lvgl.h"
#include "nvs_flash.h"
#include "sdkconfig.h"

#define I2C_BUS_PORT 0
#define LCD_PIXEL_CLOCK_HZ (400 * 1000)
#define LCD_NUM_SDA 21
#define LCD_NUM_SCL 22
#define LCD_HW_ADDR 0x3C
#define LCD_H_RES 128
#define LCD_V_RES 64

#define LCD_CMD_BITS 8
#define LCD_PARAM_BITS 8

static const char* TAG = "MAIN";

static esp_partition_t* part_info;
static esp_ota_img_states_t ota_state;
static lv_disp_t* display;

void lcd_init() {
  // I2C for the LCD
  i2c_master_bus_handle_t i2c_bus    = NULL;
  i2c_master_bus_config_t bus_config = {
      .clk_source                   = I2C_CLK_SRC_DEFAULT,
      .glitch_ignore_cnt            = 7,
      .i2c_port                     = I2C_BUS_PORT,
      .sda_io_num                   = LCD_NUM_SDA,
      .scl_io_num                   = LCD_NUM_SCL,
      .flags.enable_internal_pullup = true,
  };
  ESP_ERROR_CHECK(i2c_new_master_bus(&bus_config, &i2c_bus));

  // LCD IO Handle
  esp_lcd_panel_io_handle_t io_handle     = NULL;
  esp_lcd_panel_io_i2c_config_t io_config = {
      .dev_addr            = LCD_HW_ADDR,
      .scl_speed_hz        = LCD_PIXEL_CLOCK_HZ,
      .control_phase_bytes = 1,
      .lcd_cmd_bits        = LCD_CMD_BITS,
      .lcd_param_bits      = LCD_CMD_BITS,
      .dc_bit_offset       = 6,
  };
  ESP_ERROR_CHECK(esp_lcd_new_panel_io_i2c(i2c_bus, &io_config, &io_handle));

  // Panel Driver
  esp_lcd_panel_handle_t panel_handle     = NULL;
  esp_lcd_panel_dev_config_t panel_config = {
      .bits_per_pixel = 1,
      .reset_gpio_num = -1,
  };
  ESP_ERROR_CHECK(esp_lcd_new_panel_ssd1306(io_handle, &panel_config, &panel_handle));

  ESP_ERROR_CHECK(esp_lcd_panel_reset(panel_handle));
  ESP_ERROR_CHECK(esp_lcd_panel_init(panel_handle));
  ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(panel_handle, true));

  // LVGL
  const lvgl_port_cfg_t lvgl_cfg = ESP_LVGL_PORT_INIT_CONFIG();
  lvgl_port_init(&lvgl_cfg);
  const lvgl_port_display_cfg_t disp_cfg = {.io_handle     = io_handle,
                                            .panel_handle  = panel_handle,
                                            .buffer_size   = LCD_H_RES * LCD_V_RES,
                                            .double_buffer = true,
                                            .hres          = LCD_H_RES,
                                            .vres          = LCD_V_RES,
                                            .monochrome    = true,
                                            .rotation      = {
                                                     .swap_xy  = false,
                                                     .mirror_x = false,
                                                     .mirror_y = false,
                                            }};
  display                                = lvgl_port_add_disp(&disp_cfg);

  lv_disp_set_rotation(display, LV_DISP_ROT_180);
}

void lcd_settext(const char* pText) {
  static lv_obj_t* label = NULL;
  if (lvgl_port_lock(0)) {
    lv_obj_t* scr = lv_disp_get_scr_act(display);
    if (NULL == label)
      label = lv_label_create(scr);
    lv_label_set_long_mode(label, LV_LABEL_LONG_SCROLL_CIRCULAR);
    lv_label_set_text(label, pText);
    lv_obj_set_width(label, display->driver->hor_res);
    lv_obj_align(label, LV_ALIGN_TOP_MID, 0, 0);
    lvgl_port_unlock();
  }
}

void lcd_settext2(const uint16_t counter) {
  static lv_obj_t* label = NULL;
  if (lvgl_port_lock(0)) {
    lv_obj_t* scr = lv_disp_get_scr_act(display);
    if (NULL == label)
      label = lv_label_create(scr);
    lv_label_set_long_mode(label, LV_LABEL_LONG_SCROLL_CIRCULAR);
    lv_label_set_text_fmt(label, "Rx: %d", counter);
    lv_obj_set_width(label, display->driver->hor_res);
    lv_obj_align(label, LV_ALIGN_TOP_MID, 0, 12);
    lvgl_port_unlock();
  }
}

void lra_init() {
  if (lora_init() == 0) {
    ESP_LOGE(TAG, "LoRa module not found!");
    return;
  }
  lora_set_frequency(866e6); // 866MHz
  lora_enable_crc();

  int cr = 1; // Coding Rate
  int bw = 7; // Bandwith
  int sf = 7; // Spreadong Factor

  lora_set_coding_rate(cr);
  lora_set_bandwidth(bw);
  lora_set_spreading_factor(sf);
}

void lra_rx(void* pvParameters) {
  uint8_t buf[256];
  uint16_t rxcnt = 0;

  lcd_settext("Receiving");
  lcd_settext2(rxcnt);

  while (1) {
    lora_receive();
    if (lora_received()) {
      int rxLen = lora_receive_packet(buf, sizeof(buf));
      ESP_LOGI(TAG, "%d byte packet received:[%.*s]", rxLen, rxLen, buf);
      rxcnt++;

      lcd_settext2(rxcnt);
    }
    vTaskDelay(1);
  }
}

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
  lcd_settext("Booting...");

  lra_init();

  lcd_settext("Booted!");

  xTaskCreate(&lra_rx, "RX", 1024 * 3, NULL, 5, NULL);

  while (1) {
    vTaskDelay(100);
  }
}
