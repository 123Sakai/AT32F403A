/* add user code begin Header */
/**
  **************************************************************************
  * @file     at32f403a_407_int.c
  * @brief    main interrupt service routines.
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

/* includes ------------------------------------------------------------------*/
#include "at32f403a_407_int.h"

/* private includes ----------------------------------------------------------*/
/* add user code begin private includes */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "universal_peripherals.h"
#include "mpu6050.h"

/* add user code end private includes */

/* private typedef -----------------------------------------------------------*/
/* add user code begin private typedef */
MPU6050_Info_t *mpu6050_t;
/* add user code end private typedef */

/* private define ------------------------------------------------------------*/
/* add user code begin private define */

/* add user code end private define */

/* private macro -------------------------------------------------------------*/
/* add user code begin private macro */

/* add user code end private macro */

/* private variables ---------------------------------------------------------*/
/* add user code begin private variables */

/* add user code end private variables */

/* private function prototypes --------------------------------------------*/
/* add user code begin function prototypes */

/* add user code end function prototypes */

/* private user code ---------------------------------------------------------*/
/* add user code begin 0 */

/* add user code end 0 */

/* external variables ---------------------------------------------------------*/
/* add user code begin external variables */

/* add user code end external variables */

/**
  * @brief  this function handles nmi exception.
  * @param  none
  * @retval none
  */
void NMI_Handler(void)
{
  /* add user code begin NonMaskableInt_IRQ 0 */

  /* add user code end NonMaskableInt_IRQ 0 */

  /* add user code begin NonMaskableInt_IRQ 1 */

  /* add user code end NonMaskableInt_IRQ 1 */
}

/**
  * @brief  this function handles hard fault exception.
  * @param  none
  * @retval none
  */
void HardFault_Handler(void)
{
  /* add user code begin HardFault_IRQ 0 */

  /* add user code end HardFault_IRQ 0 */
  /* go to infinite loop when hard fault exception occurs */
  while (1)
  {
    /* add user code begin W1_HardFault_IRQ 0 */

    /* add user code end W1_HardFault_IRQ 0 */
  }
}

/**
  * @brief  this function handles memory manage exception.
  * @param  none
  * @retval none
  */
void MemManage_Handler(void)
{
  /* add user code begin MemoryManagement_IRQ 0 */

  /* add user code end MemoryManagement_IRQ 0 */
  /* go to infinite loop when memory manage exception occurs */
  while (1)
  {
    /* add user code begin W1_MemoryManagement_IRQ 0 */

    /* add user code end W1_MemoryManagement_IRQ 0 */
  }
}

/**
  * @brief  this function handles bus fault exception.
  * @param  none
  * @retval none
  */
void BusFault_Handler(void)
{
  /* add user code begin BusFault_IRQ 0 */

  /* add user code end BusFault_IRQ 0 */
  /* go to infinite loop when bus fault exception occurs */
  while (1)
  {
    /* add user code begin W1_BusFault_IRQ 0 */

    /* add user code end W1_BusFault_IRQ 0 */
  }
}

/**
  * @brief  this function handles usage fault exception.
  * @param  none
  * @retval none
  */
void UsageFault_Handler(void)
{
  /* add user code begin UsageFault_IRQ 0 */

  /* add user code end UsageFault_IRQ 0 */
  /* go to infinite loop when usage fault exception occurs */
  while (1)
  {
    /* add user code begin W1_UsageFault_IRQ 0 */

    /* add user code end W1_UsageFault_IRQ 0 */
  }
}

/**
  * @brief  this function handles svcall exception.
  * @param  none
  * @retval none
  */
void SVC_Handler(void)
{
  /* add user code begin SVCall_IRQ 0 */

  /* add user code end SVCall_IRQ 0 */
  /* add user code begin SVCall_IRQ 1 */

  /* add user code end SVCall_IRQ 1 */
}

/**
  * @brief  this function handles debug monitor exception.
  * @param  none
  * @retval none
  */
void DebugMon_Handler(void)
{
  /* add user code begin DebugMonitor_IRQ 0 */

  /* add user code end DebugMonitor_IRQ 0 */
  /* add user code begin DebugMonitor_IRQ 1 */

  /* add user code end DebugMonitor_IRQ 1 */
}

/**
  * @brief  this function handles pendsv_handler exception.
  * @param  none
  * @retval none
  */
void PendSV_Handler(void)
{
  /* add user code begin PendSV_IRQ 0 */

  /* add user code end PendSV_IRQ 0 */
  /* add user code begin PendSV_IRQ 1 */

  /* add user code end PendSV_IRQ 1 */
}

/**
  * @brief  this function handles systick handler.
  * @param  none
  * @retval none
  */
void SysTick_Handler(void)
{
  /* add user code begin SysTick_IRQ 0 */

  /* add user code end SysTick_IRQ 0 */
  /* add user code begin SysTick_IRQ 1 */

  /* add user code end SysTick_IRQ 1 */
}

/**
  * @brief  this function handles CRM handler.
  * @param  none
  * @retval none
  */
void CRM_IRQHandler(void)
{
  /* add user code begin CRM_IRQ 0 */

  /* add user code end CRM_IRQ 0 */
  /* add user code begin CRM_IRQ 1 */

  /* add user code end CRM_IRQ 1 */
}

/**
  * @brief  this function handles EXINT Line 0 handler.
  * @param  none
  * @retval none
  */
void EXINT0_IRQHandler(void)
{
  /* add user code begin EXINT0_IRQ 0 */
  if(exint_interrupt_flag_get(EXINT_LINE_0) != RESET)
  {
		free(mpu6050_t->hi2cx);
		mpu6050_t->hi2cx = NULL;

		free(mpu6050_t);
		mpu6050_t = NULL;

		mpu6050_struct_init();
    at32_led_toggle(LED4);
    exint_flag_clear(EXINT_LINE_0);
  }
  /* add user code end EXINT0_IRQ 0 */
  /* add user code begin EXINT0_IRQ 1 */

  /* add user code end EXINT0_IRQ 1 */
}

/**
  * @brief  this function handles DMA1 Channel 1 handler.
  * @param  none
  * @retval none
  */
void DMA1_Channel1_IRQHandler(void)
{
  /* add user code begin DMA1_Channel1_IRQ 0 */

  /* add user code end DMA1_Channel1_IRQ 0 */
  /* add user code begin DMA1_Channel1_IRQ 1 */

  /* add user code end DMA1_Channel1_IRQ 1 */
}

/**
  * @brief  this function handles DMA1 Channel 2 handler.
  * @param  none
  * @retval none
  */
void DMA1_Channel2_IRQHandler(void)
{
  /* add user code begin DMA1_Channel2_IRQ 0 */

  /* add user code end DMA1_Channel2_IRQ 0 */
  /* add user code begin DMA1_Channel2_IRQ 1 */

  /* add user code end DMA1_Channel2_IRQ 1 */
}

/**
  * @brief  this function handles I2C1 Event handler.
  * @param  none
  * @retval none
  */
void I2C1_EVT_IRQHandler(void)
{
  /* add user code begin I2C1_EVT_IRQ 0 */

  /* add user code end I2C1_EVT_IRQ 0 */
  /* add user code begin I2C1_EVT_IRQ 1 */

  /* add user code end I2C1_EVT_IRQ 1 */
}

/**
  * @brief  this function handles I2C1 Error handler.
  * @param  none
  * @retval none
  */
void I2C1_ERR_IRQHandler(void)
{
  /* add user code begin I2C1_ERR_IRQ 0 */

  /* add user code end I2C1_ERR_IRQ 0 */
  /* add user code begin I2C1_ERR_IRQ 1 */

  /* add user code end I2C1_ERR_IRQ 1 */
}

/**
  * @brief  this function handles UART4 handler.
  * @param  none
  * @retval none
  */
void UART4_IRQHandler(void)
{
  /* add user code begin UART4_IRQ 0 */
	uint8_t clear;
	if(usart_interrupt_flag_get(UART4, USART_RDBF_FLAG) != RESET)//usart_interrupt_flag_get中断接收用这个，usart_flag_get
	{
		Muartnum.RecvBuf[Muartnum.recvBufSize++] = UART4->dt;
	}
	if(usart_interrupt_flag_get(UART4, USART_IDLEF_FLAG) != RESET)
	{
		clear = UART4->sts;
		clear = UART4->dt;
		clear &= 0;
		Muartnum.recvState = 1;
	}
  /* add user code end UART4_IRQ 0 */
  /* add user code begin UART4_IRQ 1 */

  /* add user code end UART4_IRQ 1 */
}

/* add user code begin 1 */

/* add user code end 1 */
