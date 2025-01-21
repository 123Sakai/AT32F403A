/* add user code begin Header */
/**
  **************************************************************************
  * @file     at32f403a_407_int.h
  * @brief    header file of main interrupt service routines.
  **************************************************************************
  *                       Copyright notice & Disclaimer
  *
  * The software Board Support Package (BSP) that is made available to
  * download from Artery official website is the copyrighted work of Artery.
  * Artery authorizes customers to use, copy, and distribute the BSP
  * software and its related documentation for the purpose of design and
  * development in conjunction with Artery microcontrollers. Use of the
  * software is governed by this copyright notice and the following disclaimer.
  *
  * THIS SOFTWARE IS PROVIDED ON "AS IS" BASIS WITHOUT WARRANTIES,
  * GUARANTEES OR REPRESENTATIONS OF ANY KIND. ARTERY EXPRESSLY DISCLAIMS,
  * TO THE FULLEST EXTENT PERMITTED BY LAW, ALL EXPRESS, IMPLIED OR
  * STATUTORY OR OTHER WARRANTIES, GUARANTEES OR REPRESENTATIONS,
  * INCLUDING BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY,
  * FITNESS FOR A PARTICULAR PURPOSE, OR NON-INFRINGEMENT.
  *
  **************************************************************************
  */
/* add user code end Header */

/* define to prevent recursive inclusion -------------------------------------*/
#ifndef __AT32F403A_407_INT_H
#define __AT32F403A_407_INT_H

#ifdef __cplusplus
extern "C" {
#endif

/* includes ------------------------------------------------------------------*/
#include "at32f403a_407.h"

/* private includes ----------------------------------------------------------*/
/* add user code begin private includes */
#include "DELAY_ONE.h"
#include "at32f403A_407_wk_config.h"
/* add user code end private includes */

/* exported types ------------------------------------------------------------*/
/* add user code begin exported types */
#define uart4Buffer 1024           //可以接收的最大字符个数 
typedef struct Muart 
{
	uint8_t RecvBuf[uart4Buffer];
	uint8_t sendBuf[uart4Buffer];
	uint8_t recvBufSize;
	uint8_t sendBufSize;
	uint8_t recvState;
}muart_t;
extern muart_t Muartnum;
/* add user code end exported types */

/* exported constants --------------------------------------------------------*/
/* add user code begin exported constants */

/* add user code end exported constants */

/* exported macro ------------------------------------------------------------*/
/* add user code begin exported macro */

/* add user code end exported macro */

/* exported functions ------------------------------------------------------- */
void NMI_Handler(void);
void HardFault_Handler(void);
void MemManage_Handler(void);
void BusFault_Handler(void);
void UsageFault_Handler(void);
void SVC_Handler(void);
void DebugMon_Handler(void);
void PendSV_Handler(void);
void SysTick_Handler(void);

void CRM_IRQHandler(void);
void UART4_IRQHandler(void);
/* add user code begin exported functions */

/* add user code end exported functions */

#ifdef __cplusplus
}
#endif

#endif
