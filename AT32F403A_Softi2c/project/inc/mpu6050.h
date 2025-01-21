#ifndef __MPU6050_H
#define __MPU6050_H
#include <stdio.h> 
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include "at32f403a_407_i2c.h"
#include "at32f403a_407_wk_config.h"


#define MPU6050_ADDRESS		    0xD0     //MPU6050 I2C slave address,add 1 as read
 
#define	MPU6050_SMPLRT_DIV		0x19
#define	MPU6050_CONFIG				0x1A 
#define	MPU6050_GYRO_CONFIG		0x1B
#define	MPU6050_ACCEL_CONFIG	0x1C
 
#define	MPU6050_ACCEL_XOUT_H	0x3B
#define	MPU6050_ACCEL_XOUT_L	0x3C
#define	MPU6050_ACCEL_YOUT_H	0x3D
#define	MPU6050_ACCEL_YOUT_L	0x3E
#define	MPU6050_ACCEL_ZOUT_H	0x3F
#define	MPU6050_ACCEL_ZOUT_L	0x40

#define	MPU6050_TEMP_OUT_H		0x41     
#define	MPU6050_TEMP_OUT_L		0x42

#define	MPU6050_GYRO_XOUT_H		0x43
#define	MPU6050_GYRO_XOUT_L		0x44
#define	MPU6050_GYRO_YOUT_H		0x45
#define	MPU6050_GYRO_YOUT_L		0x46
#define	MPU6050_GYRO_ZOUT_H		0x47
#define	MPU6050_GYRO_ZOUT_L		0x48
 
#define	MPU6050_PWR_MGMT_1		0x6B     //power manager 1
#define	MPU6050_PWR_MGMT_2		0x6C     //power manager 2
#define	MPU6050_WHO_AM_I			0x75     //device ID's addr
 
 
 
void MPU6050_Init(void);
uint8_t MPU6050_GetID(void);
 
void MPU6050_GetACCEL(int16_t *ACCEL_Array);
void MPU6050_GetGYRO(int16_t *GYRO_Array);
void MPU6050_GetTemp(int16_t *Temp_Array);

void MPU6050_GetAccel_Value(float *Accel_Value);
void MPU6050_GetGyro_Value(float *Gyro_Value);
float MPU6050_GetTemp_Value(void);
#endif 


