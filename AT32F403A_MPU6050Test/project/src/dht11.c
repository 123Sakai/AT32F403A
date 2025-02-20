#include "dht11.h"

void dht11_io_in(void)
{
	gpio_init_type gpio_init_struct;
  gpio_default_para_init(&gpio_init_struct);

  gpio_bits_reset(DHT11_SDA_GPIO_PORT, DHT11_SDA_PIN);
  gpio_init_struct.gpio_mode = GPIO_MODE_INPUT;
  gpio_init_struct.gpio_pins = DHT11_SDA_PIN;
  gpio_init_struct.gpio_pull = GPIO_PULL_UP;
  gpio_init(DHT11_SDA_GPIO_PORT, &gpio_init_struct);
}
 
void dht11_io_out(void)
{
  gpio_init_type gpio_init_struct;
  gpio_default_para_init(&gpio_init_struct);

  gpio_bits_reset(DHT11_SDA_GPIO_PORT, DHT11_SDA_PIN);

  gpio_init_struct.gpio_drive_strength = GPIO_DRIVE_STRENGTH_MODERATE;
  gpio_init_struct.gpio_out_type = GPIO_OUTPUT_PUSH_PULL;
  gpio_init_struct.gpio_mode = GPIO_MODE_OUTPUT;
  gpio_init_struct.gpio_pins = DHT11_SDA_PIN;
  gpio_init_struct.gpio_pull = GPIO_PULL_UP;
  gpio_init(DHT11_SDA_GPIO_PORT, &gpio_init_struct);
}


void dht11_ready_read(void)
{
	dht11_io_out();
	DHT11_SDA_RESET;
	delay_ms(20);
	DHT11_SDA_SET;//release
	delay_us(30);
}

uint8_t dht11_check(void)
{
	uint8_t dht11_state=0;//1:online 0:underline
	uint8_t wait=0;
	dht11_ready_read();
	dht11_io_in();
	while(DHT11_READ_DATA == 1 && wait<100)
	{
		wait++;
		delay_us(1);
	}
	if(wait>=100) dht11_state=1;//underline
	else wait=0;
	while(DHT11_READ_DATA == 0 && wait<100)
	{
		wait++;
		delay_us(1);
	}
	if(wait>=100) dht11_state=1;
	return dht11_state;
}

uint8_t dht11_bit_read(void)
{
	uint8_t wait = 0;
	while(DHT11_READ_DATA == 1 && wait < 100)
	{
		wait++;
		delay_us(1);
	}
	wait = 0;
	while(DHT11_READ_DATA == 0 && wait < 100)
	{
		wait++;
		delay_us(1);
	}
	delay_us(40);
	if(DHT11_READ_DATA) return 1;
	else return 0;
}

uint8_t dht11_gpio_read(void)
{
	uint8_t i = 0;
	uint8_t byte = 0;
	for(i=0;i<8;i++)
	{
		byte <<= 1;
		byte |= dht11_bit_read();
	}
	return byte;
}

void dht11_finish_read(void)//useless
{
	dht11_io_out();
	DHT11_SDA_RESET;//idle
	delay_us(54);
	DHT11_SDA_SET;
}

uint8_t dht11_read_raw_data(DHT11_Data_t *dht11_t)
{
	uint8_t ret = 0,i = 0;
	uint8_t raw_data[5]={0};
	
	dht11_check();
	for(i=0;i<5;i++)
	{
		raw_data[i] = dht11_gpio_read();
	}
	if(raw_data[0] + raw_data[1] + raw_data[2] + raw_data[3] == raw_data[4])
	{
		dht11_t->dht11_humi = (raw_data[0]<<8) | raw_data[1];
		dht11_t->dht11_temp = (raw_data[2]<<8) | raw_data[3];
		ret = 1;
	}
	return ret;
}

 
