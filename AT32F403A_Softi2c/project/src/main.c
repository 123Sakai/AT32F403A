/* add user code begin Header */
/**
  **************************************************************************
  * @file     main.c
  * @brief    main program
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

/* Includes ------------------------------------------------------------------*/
#include "at32f403a_407_wk_config.h"

/* private includes ----------------------------------------------------------*/
/* add user code begin private includes */
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include "mpu6050.h"
#include "DELAY_ONE.h"
#include "at32f403a_407_int.h"
/* add user code end private includes */

/* private typedef -----------------------------------------------------------*/
/* add user code begin private typedef */

/* add user code end private typedef */

/* private define ------------------------------------------------------------*/
/* add user code begin private define */

/* add user code end private define */

/* private macro -------------------------------------------------------------*/
/* add user code begin private macro */

/* add user code end private macro */

/* private variables ---------------------------------------------------------*/
/* add user code begin private variables */
uint8_t mpu6050_id=0;
float mpu6050_accel_value[3]={0};
float mpu6050_gyro_value[3]={0};
float mpu6050_temp_value = 0;
muart_t Muartnum;
#define COUNTOF(a)                       (sizeof(a) / sizeof(*(a)))
#define UART4_TX_BUFFER_SIZE            (COUNTOF(uart4_tx_buffer) - 1)

uint8_t uart4_tx_buffer[] = "uart transfer by interrupt: uart4 -> ";
uint8_t uart4_rx_buffer[UART4_TX_BUFFER_SIZE];
volatile uint8_t uart4_tx_dma_status = 0;
volatile uint8_t uart4_rx_dma_status = 0;

#define BUFFERSIZE_uart4 1024           //可以接收的最大字符个数       
uint8_t ReceiveBuff_uart4[BUFFERSIZE_uart4]; //接收缓冲区
uint8_t recv_end_flag_uart4 = 0,Rx_len_uart4;//接收完成中断标志，接收到字符长度
/* add user code end private variables */

/* private function prototypes --------------------------------------------*/
/* add user code begin function prototypes */

/* add user code end function prototypes */

/* private user code ---------------------------------------------------------*/
/* add user code begin 0 */
#if (__ARMCC_VERSION > 6000000)
  __asm (".global __use_no_semihosting\n\t");
  void _sys_exit(int x)
  {
    x = x;
  }
  /* __use_no_semihosting was requested, but _ttywrch was */
  void _ttywrch(int ch)
  {
    ch = ch;
  }
  FILE __stdout;
#else
 #ifdef __CC_ARM
  #pragma import(__use_no_semihosting)
  struct __FILE
  {
    int handle;
  };
  FILE __stdout;
  void _sys_exit(int x)
  {
    x = x;
  }
  /* __use_no_semihosting was requested, but _ttywrch was */
  void _ttywrch(int ch)
  {
    ch = ch;
  }
 #endif
#endif
	
int fputc(int ch, FILE *f)
{
  while(usart_flag_get(UART4, USART_TDBE_FLAG) == RESET);
  usart_data_transmit(UART4, (uint16_t)ch);
  while(usart_flag_get(UART4, USART_TDC_FLAG) == RESET);
  return ch;
}

void uart_Txdatas(usart_type* usart_x,uint16_t *SendUARTData,uint16_t len)
{
	uint8_t i=0;
	for(i=0;i<len;i++)
	{
		while(usart_flag_get(usart_x , USART_TDBE_FLAG) == RESET);
		usart_data_transmit(usart_x, SendUARTData[i]);
		while(usart_flag_get(usart_x, USART_TDC_FLAG) == RESET);
	}
}
void mpu6050_init_struct(void)
{
	int i=0;
	float temp_tmp = 0.0f;
	mpu6050_id = MPU6050_GetID();
	for(i=0;i<10;i++)
	{
		temp_tmp += MPU6050_GetTemp_Value();
		delay_ms(1000);
	}
	if(mpu6050_id != 0x68)
	{
		printf("cann't read MPU6050,check it.\r\n");
		//while(1);
	}
	else
	{
		printf("get mpu6050_id : 0x%X\n",mpu6050_id);
	}
	mpu6050_temp_value = temp_tmp / i;
	printf("mpu6050_temp_value = %.2f C \n",mpu6050_temp_value);
}
void mpu6050_capture_data(void)
{
	int i=0;
		MPU6050_GetAccel_Value(mpu6050_accel_value);
		MPU6050_GetGyro_Value(mpu6050_gyro_value);
		printf("mpu6050_accel_value:");
		for(i=0;i<3;i++)
		{
			printf("%.2f ",mpu6050_accel_value[i]);
		}
		printf("\r\nmpu6050_gyro_value:");
		for(i=0;i<3;i++)
		{
			printf("%.2f ",mpu6050_gyro_value[i]);
		}
		printf("\r\n");
		delay_ms(1000);
}
/* add user code end 0 */

/**
  * @brief main function.
  * @param  none
  * @retval none
  */
int main(void)
{
  /* add user code begin 1 */

  /* add user code end 1 */

  /* system clock config. */
  wk_system_clock_config();

  /* config periph clock. */
  wk_periph_clock_config();

  /* config systick clock source */
  systick_clock_source_config(SYSTICK_CLOCK_SOURCE_AHBCLK_DIV8);
  /* system tick config */
  /**
   * use systick as time base source and configure 1ms tick.
   * users need add interrupt handler code into the below function in the at32f403a_407_int.c file.
   *  --void SystTick_IRQHandler(void)
   */
  systick_interrupt_config(system_core_clock / 8 / 1000);

  /* nvic config. */
  wk_nvic_config();

  /* init uart4 function. */
  wk_uart4_init();

  /* init gpio function. */
  wk_gpio_config();

  /* add user code begin 2 */
	delay_init();
	MPU6050_Init();
	mpu6050_init_struct();
//	while(1)
//	{
//		mpu6050_id = MPU6050_GetID();
//		//delay_ms(500);
//	}

  /* add user code end 2 */

  while(1)
  {
    /* add user code begin 3 */
	mpu6050_capture_data();
    /* add user code end 3 */
  }
}
