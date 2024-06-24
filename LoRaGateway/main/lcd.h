/**
 ******************************************************************************
 *  file           : lcd.h
 *  brief          : The LCD
 ******************************************************************************
 *  Copyright (C) 2024 Florian Brandner
 */

#ifndef __LCD_H_
#define __LCD_H_

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdbool.h>
#include <stdint.h>

/* Private includes ----------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/

/* Exported constants --------------------------------------------------------*/

/* Exported macro ------------------------------------------------------------*/

/* Exported functions prototypes ---------------------------------------------*/

void lcd_init();
void lcd_settext(const uint8_t line, const char* pText);

void lcd_statusbar(const bool wifi, const bool mqtt);

#ifdef __cplusplus
}
#endif

#endif // __LCD_H_
