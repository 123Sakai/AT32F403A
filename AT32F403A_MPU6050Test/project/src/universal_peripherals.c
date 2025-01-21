#include "universal_peripherals.h"


gpio_type *led_gpio_port[LED_NUM]        = {LED2_GPIO, LED3_GPIO, LED4_GPIO};
uint16_t led_gpio_pin[LED_NUM]           = {LED2_PIN, LED3_PIN, LED4_PIN};
crm_periph_clock_type led_gpio_crm_clk[LED_NUM] = {LED2_GPIO_CRM_CLK, LED3_GPIO_CRM_CLK, LED4_GPIO_CRM_CLK};


void at32_led_init(led_type led)
{
  gpio_init_type gpio_init_struct;

  /* enable the led clock */
  crm_periph_clock_enable(led_gpio_crm_clk[led], TRUE);

  /* set default parameter */
  gpio_default_para_init(&gpio_init_struct);

  /* configure the led gpio */
  gpio_init_struct.gpio_drive_strength = GPIO_DRIVE_STRENGTH_STRONGER;
  gpio_init_struct.gpio_out_type  = GPIO_OUTPUT_PUSH_PULL;
  gpio_init_struct.gpio_mode = GPIO_MODE_OUTPUT;
  gpio_init_struct.gpio_pins = led_gpio_pin[led];
  gpio_init_struct.gpio_pull = GPIO_PULL_NONE;
  gpio_init(led_gpio_port[led], &gpio_init_struct);
}

void at32_led_on(led_type led)
{
  if(led > (LED_NUM - 1))
    return;
  if(led_gpio_pin[led])
    led_gpio_port[led]->clr = led_gpio_pin[led];
}

void at32_led_off(led_type led)
{
  if(led > (LED_NUM - 1))
    return;
  if(led_gpio_pin[led])
    led_gpio_port[led]->scr = led_gpio_pin[led];
}

void at32_led_toggle(led_type led)
{
  if(led > (LED_NUM - 1))
    return;
  if(led_gpio_pin[led])
    led_gpio_port[led]->odt ^= led_gpio_pin[led];
}

void at32_button_init(void)
{
  gpio_init_type gpio_init_struct;

  /* enable the button clock */
  crm_periph_clock_enable(USER_BUTTON_CRM_CLK, TRUE);

  /* set default parameter */
  gpio_default_para_init(&gpio_init_struct);

  /* configure button pin as input with pull-up/pull-down */
  gpio_init_struct.gpio_drive_strength = GPIO_DRIVE_STRENGTH_STRONGER;
  gpio_init_struct.gpio_out_type  = GPIO_OUTPUT_PUSH_PULL;
  gpio_init_struct.gpio_mode = GPIO_MODE_INPUT;
  gpio_init_struct.gpio_pins = MY_BTN_PIN;
  gpio_init_struct.gpio_pull = GPIO_PULL_DOWN;
  gpio_init(MY_BTN_GPIO_PORT, &gpio_init_struct);
}

uint8_t at32_button_state(void)
{
  return gpio_input_data_bit_read(MY_BTN_GPIO_PORT, MY_BTN_PIN);
}

button_type at32_button_press()
{
  static uint8_t pressed = 1;
  /* get button state in at_start board */
  if((pressed == 1) && (at32_button_state() != RESET))
  {
    /* debounce */
    pressed = 0;
    delay_ms(10);
    if(at32_button_state() != RESET)
      return USER_BUTTON;
  }
  else if(at32_button_state() == RESET)
  {
    pressed = 1;
  }
  return NO_BUTTON;
}

//init this before must init delay
void Led_and_Button_init(void)
{
	//delay_init();
	
  at32_led_init(LED2);
  at32_led_init(LED3);
  at32_led_init(LED4);
  at32_led_off(LED2);
  at32_led_off(LED3);
  at32_led_off(LED4);

  at32_button_init();
}
