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
#include <stdint.h>

/* Private includes ----------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/

/* Exported constants --------------------------------------------------------*/

/* Exported macro ------------------------------------------------------------*/

/* Exported functions prototypes ---------------------------------------------*/

void lcd_init();
void lcd_settext1(const char* pText);
void lcd_settext2(const uint16_t counter);
void lcd_settext3(const uint16_t counter);

#ifdef __cplusplus
}
#endif

#endif // __LCD_H_
