#ifndef __MPU6050_H
#define __MPU6050_H
#include <stdio.h> 
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include "at32f403a_407_i2c.h"
#include "at32f403a_407_wk_config.h"
#include "i2c_application.h"
#include "universal_peripherals.h"

#define MPU6050_ADDRESS		    0xD0     //MPU6050 I2C slave address,add 1 as read
 
#define	MPU6050_SMPLRT_DIV		0x19
#define	MPU6050_DLPF_CONFIG				0x1A
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


/*AFS_SEL*/
typedef enum {
    ACCEL_FS_SEL_2G = 0,    // ±2g
    ACCEL_FS_SEL_4G,        // ±4g
    ACCEL_FS_SEL_8G,        // ±8g
    ACCEL_FS_SEL_16G        // ±16g
} MPU6050_ACCEL_FS_SEL_t;

/*FS_SEL*/
typedef enum {
    GYRO_FS_SEL_250DPS = 0,  // ±250dps
    GYRO_FS_SEL_500DPS,      // ±500dps
    GYRO_FS_SEL_1000DPS,     // ±1000dps
    GYRO_FS_SEL_2000DPS      // ±2000dps
} MPU6050_GYRO_FS_SEL_t;

/*DLPF_CFG*/
typedef enum {
    LPF_CUTOFF_FREQ_5HZ = 0,    //截止频率5HZ   初始采样率1KHZ
    LPF_CUTOFF_FREQ_10HZ,       //截止频率10HZ   初始采样率1KHZ
    LPF_CUTOFF_FREQ_21HZ,       //截止频率21HZ   初始采样率1KHZ
    LPF_CUTOFF_FREQ_44HZ,       //截止频率44HZ   初始采样率1KHZ
    LPF_CUTOFF_FREQ_94HZ,       //截止频率94HZ   初始采样率1KHZ
    LPF_CUTOFF_FREQ_184HZ,      //截止频率184HZ   初始采样率1KHZ
    LPF_CUTOFF_FREQ_260HZ,      //截止频率260HZ   初始采样率8KHZ
    LPF_CUTOFF_FREQ_3600HZ      //禁用低通滤波器   初始采样率8KHZ
} MPU6050_LPF_CUTOFF_FREQ_t;

/*SMPLRT_DIV*/
typedef enum {
    SAMPLE_RATE_DIV0 = 0,       //0分频
    SAMPLE_RATE_DIV2,           //2分频
    SAMPLE_RATE_DIV4,           //4分频
    SAMPLE_RATE_DIV8,           //8分频
    SAMPLE_RATE_DIV16,          //16分频
    SAMPLE_RATE_DIV32,          //32分频
    SAMPLE_RATE_DIV64,          //64分频
    SAMPLE_RATE_DIV128          //128分频
} MPU6050_SAMPLE_RATE_t;

typedef struct 
{
	i2c_handle_type *hi2cx;											// i2c handle
	i2c_status_type *i2c_status;								// i2c status
	MPU6050_LPF_CUTOFF_FREQ_t lpf_cutoff_freq;  // DLPF_CFG
	MPU6050_SAMPLE_RATE_t sample_rate;          // SMPLRT_DIV
	MPU6050_ACCEL_FS_SEL_t accel_fs_sel;        // AFS_SEL
	MPU6050_GYRO_FS_SEL_t gyro_fs_sel;          // FS_SEL
	uint8_t init_flag;													// is init?
}MPU6050_Info_t;
 
void MPU6050_Init(MPU6050_Info_t *mpu6050_t);
uint8_t MPU6050_GetID(MPU6050_Info_t *mpu6050_t);
 
void MPU6050_GetACCEL(MPU6050_Info_t *mpu6050_t,int16_t *ACCEL_Array);
void MPU6050_GetGYRO(MPU6050_Info_t *mpu6050_t,int16_t *GYRO_Array);
void MPU6050_GetTemp(MPU6050_Info_t *mpu6050_t,int16_t *Temp_Array);

void MPU6050_GetAccel_Value(MPU6050_Info_t *mpu6050_t,float *Accel_Value);
void MPU6050_GetGyro_Value(MPU6050_Info_t *mpu6050_t,float *Gyro_Value);
float MPU6050_GetTemp_Value(MPU6050_Info_t *mpu6050_t);
#endif 


