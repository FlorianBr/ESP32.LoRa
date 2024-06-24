/**
 ******************************************************************************
 *  file           : lcd.c
 *  brief          : The LCD
 ******************************************************************************
 *  Copyright (C) 2024 Florian Brandner
 */

/* Includes ------------------------------------------------------------------*/
#include "lcd.h"

#include "driver/i2c_master.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_panel_vendor.h"
#include "esp_log.h"
#include "esp_lvgl_port.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "lvgl.h"
#include "sdkconfig.h"

/* Private includes ----------------------------------------------------------*/

/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/
#define I2C_BUS_PORT 0                         // I2C port to use
#define LCD_PIXEL_CLOCK_HZ (400 * 1000)        // 400kHz
#define LCD_NUM_SDA 21                         // SDA Pin number
#define LCD_NUM_SCL 22                         // SCL  Pin number
#define LCD_HW_ADDR 0x3C                       // LCD address
#define LCD_H_RES 128                          // Horizontal resolution
#define LCD_V_RES 64                           // Vertical resolution
#define LCD_CMD_BITS 8                         // 1 Byte command
#define LCD_PARAM_BITS 8                       // 1 Byte param
#define LINE_HEIGHT 12                         // Height of a line in pixels
#define LINE_MAX_NUM (LCD_V_RES / LINE_HEIGHT) // Max. Number of lines

/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

static lv_disp_t* display;
static const char* TAG = "LCD";

/* Private function prototypes -----------------------------------------------*/

/* Private user code ---------------------------------------------------------*/

/* Public user code ----------------------------------------------------------*/

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

  lv_disp_set_rotation(display, LV_DISP_ROT_NONE);
}

void lcd_settext(const uint8_t line, const char* pText) {
  static lv_obj_t* label[LINE_MAX_NUM] = {NULL};

  if (line >= LINE_MAX_NUM) {
    ESP_LOGE(TAG, "Line %d is invalid, max is %d", line, LINE_MAX_NUM);
    return;
  }

  if (lvgl_port_lock(0)) {
    lv_obj_t* scr = lv_disp_get_scr_act(display);
    if (label[line] == NULL) {
      label[line] = lv_label_create(scr);
    }
    lv_label_set_long_mode(label[line], LV_LABEL_LONG_SCROLL_CIRCULAR);
    lv_label_set_text(label[line], pText);
    lv_obj_set_width(label[line], display->driver->hor_res);
    lv_obj_align(label[line], LV_ALIGN_TOP_MID, 0, line * LINE_HEIGHT);
    lv_obj_set_style_text_align(label[line], LV_TEXT_ALIGN_CENTER, 0);
    lvgl_port_unlock();
  }
}

void lcd_statusbar(const bool wifi, const bool mqtt) {
  lv_obj_t* scr = lv_disp_get_scr_act(display);
  if (lvgl_port_lock(0)) {
    char cBuffer[100];

    static lv_obj_t* label = NULL;
    if (label == NULL) {
      label = lv_label_create(scr);
    }
    lv_label_set_long_mode(label, LV_LABEL_LONG_SCROLL_CIRCULAR);

    snprintf(&cBuffer[0], sizeof(cBuffer), "WiFi:%s MQTT:%s", (wifi ? LV_SYMBOL_OK : LV_SYMBOL_CLOSE),
             (mqtt ? LV_SYMBOL_OK : LV_SYMBOL_CLOSE));
    lv_label_set_text(label, cBuffer);
    lv_obj_set_width(label, display->driver->hor_res);
    lv_obj_align(label, LV_ALIGN_TOP_MID, 0, 0);
    lv_obj_set_style_text_align(label, LV_TEXT_ALIGN_CENTER, 0);
    lvgl_port_unlock();
  }
}