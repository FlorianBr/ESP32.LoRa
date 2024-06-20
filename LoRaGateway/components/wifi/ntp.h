/**
 ******************************************************************************
 *  file           : ntp.h
 *  brief          : Getting the time from a NTP Server
 ******************************************************************************
 *  Copyright (C) 2024 Florian Brandner
 */

#ifndef NTP_H_
#define NTP_H_

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/

/* Private includes ----------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/

/* Exported constants --------------------------------------------------------*/

/* Exported macro ------------------------------------------------------------*/

/* Exported functions prototypes ---------------------------------------------*/

esp_err_t NTP_Init(void);

#ifdef __cplusplus
}
#endif

#endif // NTP_H_