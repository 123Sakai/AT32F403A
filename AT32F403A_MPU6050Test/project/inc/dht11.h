#ifndef __DHT11_H
#define __DHT11_H
#include <stdio.h> 
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include "at32f403a_407_wk_config.h"
#include "universal_peripherals.h"

typedef struct
{
	uint16_t dht11_temp;
	uint16_t dht11_humi;
	uint8_t dht11_calibration;
}DHT11_Data_t;

uint8_t dht11_read_raw_data(DHT11_Data_t *dht11_t);
#endif


