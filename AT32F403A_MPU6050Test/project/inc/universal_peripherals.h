#ifndef __UNIVERSAL_PERIPHRTALS_H
#define __UNIVERSAL_PERIPHRTALS_H

#ifdef __cplusplus
extern "C" {
#endif
#include "at32f403a_407.h"
#include "at32f403a_407_wk_config.h"
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include "mpu6050.h"
#include "DELAY_ONE.h"

typedef enum
{
  LED2                                   = 0,
  LED3                                   = 1,
  LED4                                   = 2
} led_type;

#define LED_NUM                          3

#define LED2_PIN                         GPIO_PINS_13
#define LED2_GPIO                        GPIOD
#define LED2_GPIO_CRM_CLK                CRM_GPIOD_PERIPH_CLOCK

#define LED3_PIN                         GPIO_PINS_14
#define LED3_GPIO                        GPIOD
#define LED3_GPIO_CRM_CLK                CRM_GPIOD_PERIPH_CLOCK

#define LED4_PIN                         GPIO_PINS_15
#define LED4_GPIO                        GPIOD
#define LED4_GPIO_CRM_CLK                CRM_GPIOD_PERIPH_CLOCK

typedef enum
{
  USER_BUTTON                            = 0,
  NO_BUTTON                              = 1
} button_type;

#define MY_BTN_PIN                  		 GPIO_PINS_0
#define MY_BTN_GPIO_PORT                 GPIOA
#define USER_BUTTON_CRM_CLK              CRM_GPIOA_PERIPH_CLOCK

void Led_and_Button_init(void);

void at32_led_init(led_type led);
void at32_led_on(led_type led);
void at32_led_off(led_type led);
void at32_led_toggle(led_type led);
void at32_button_init(void);
uint8_t at32_button_state(void);
button_type at32_button_press();


#ifdef __cplusplus
extern }
#endif

#endif


