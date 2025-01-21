#include "mpu6050.h"
#include "DELAY_ONE.h"

void error_handler(uint32_t error_code)
{
  while(1)
  {
		printf("err:0x%x.\n",error_code);
    delay_ms(1000);
  }
}

i2c_status_type MPU6050_WriteReg(MPU6050_Info_t *mpu6050_t, uint8_t Reg_Addr, uint8_t Data)
{
	i2c_status_type write_flag;
	i2c_memory_write(mpu6050_t->hi2cx,I2C_MEM_ADDR_WIDIH_8,MPU6050_ADDRESS,Reg_Addr,&Data,sizeof(Data),I2C_TIMEOUT);
	delay_ms(10);
	return write_flag;
	//return i2c_memory_write(mpu6050_t->hi2cx,I2C_MEM_ADDR_WIDIH_8,MPU6050_ADDRESS,Reg_Addr,&Data,sizeof(Data),I2C_TIMEOUT);
}

i2c_status_type MPU6050_ReadReg(MPU6050_Info_t *mpu6050_t,uint8_t Reg_Addr,uint8_t *pData,uint16_t d_size)
{
	i2c_status_type read_flag;
	read_flag = i2c_memory_read(mpu6050_t->hi2cx,I2C_MEM_ADDR_WIDIH_8,MPU6050_ADDRESS,Reg_Addr,pData,d_size,I2C_TIMEOUT);
	delay_ms(10);
	return read_flag;
}

int16_t MPU6050_Byte_to_HalfWord(uint8_t DataL, uint8_t DataH)
{
    int16_t Data;
    Data = (DataH << 8) | DataL;
    return Data;
}

void MPU6050_SetLPF(MPU6050_Info_t *mpu6050_t, MPU6050_LPF_CUTOFF_FREQ_t lpf_cutoff_freq)
{
    uint8_t data;
    switch (lpf_cutoff_freq)
    {
        case LPF_CUTOFF_FREQ_5HZ://<<
					data = 0x06;
					break;
        case LPF_CUTOFF_FREQ_10HZ:
            data = 0x05;
            break;
        case LPF_CUTOFF_FREQ_21HZ:
            data = 0x04;
            break;
        case LPF_CUTOFF_FREQ_44HZ:
            data = 0x03;
            break;
        case LPF_CUTOFF_FREQ_94HZ:
            data = 0x02;
            break;
        case LPF_CUTOFF_FREQ_184HZ:
            data = 0x01;
            break;
        case LPF_CUTOFF_FREQ_260HZ:
            data = 0x00;
            break;
        case LPF_CUTOFF_FREQ_3600HZ:
            data = 0x07;
            break;
        default:
            return;
    }
    MPU6050_WriteReg(mpu6050_t,MPU6050_DLPF_CONFIG, data); //DLPF_CFG
    mpu6050_t->lpf_cutoff_freq = lpf_cutoff_freq;
}

void MPU6050_SetSampleRate(MPU6050_Info_t *mpu6050_t, MPU6050_SAMPLE_RATE_t sample_rate)
{
    uint8_t data;
    switch (sample_rate)
    {
        case SAMPLE_RATE_DIV0:  //0
					data = 0x00;
					break;
        case SAMPLE_RATE_DIV2:  //2
            data = 0x01;
            break;
        case SAMPLE_RATE_DIV4:  //4
            data = 0x03;
            break;
        case SAMPLE_RATE_DIV8:  //8<<
            data = 0x07;
            break;
        case SAMPLE_RATE_DIV16: //16
            data = 0x0F;
            break;
        case SAMPLE_RATE_DIV32: //32
            data = 0x1F;
            break;
        case SAMPLE_RATE_DIV64: //64
            data = 0x3F;
            break;
        case SAMPLE_RATE_DIV128: //128
            data = 0x7F;
            break;
        default:
            return;
    }
    MPU6050_WriteReg(mpu6050_t,MPU6050_SMPLRT_DIV, data); //sample rate
    mpu6050_t->sample_rate = sample_rate;
}

void MPU6050_SetAccelFS(MPU6050_Info_t *mpu6050_t, MPU6050_ACCEL_FS_SEL_t accel_fs_sel)
{
    uint8_t data;
    switch (accel_fs_sel) 
    {
			case ACCEL_FS_SEL_2G:
        data = 0x00;
        break;
			case ACCEL_FS_SEL_4G:
        data = 0x08;
        break;
			case ACCEL_FS_SEL_8G:
        data = 0x10;
        break;
			case ACCEL_FS_SEL_16G://<<
        data = 0x18;
        break;
			default:
        return;
    }
    MPU6050_WriteReg(mpu6050_t, MPU6050_ACCEL_CONFIG, data);
    mpu6050_t->accel_fs_sel = accel_fs_sel;
}

void MPU6050_SetGyroFS(MPU6050_Info_t *mpu6050_t, MPU6050_GYRO_FS_SEL_t gyro_fs_sel)
{
    uint8_t data;
    switch (gyro_fs_sel) 
    {
			case GYRO_FS_SEL_250DPS:
        data = 0x00;
        break;
			case GYRO_FS_SEL_500DPS:
        data = 0x08;
        break;
			case GYRO_FS_SEL_1000DPS:
        data = 0x10;
        break;
			case GYRO_FS_SEL_2000DPS://<<
        data = 0x18;
        break;
			default:
        return;
    }

    MPU6050_WriteReg(mpu6050_t, MPU6050_GYRO_CONFIG, data);
    mpu6050_t->gyro_fs_sel = gyro_fs_sel;
}
uint8_t MPU6050_GetID(MPU6050_Info_t *mpu6050_t)
{	
	uint8_t get_id = 0;
	MPU6050_ReadReg(mpu6050_t,MPU6050_WHO_AM_I,&get_id,1);
	if(get_id != 0x68)
	{
		mpu6050_t->init_flag = 0;
	}
	return get_id;
}

void MPU6050_Init(MPU6050_Info_t *mpu6050_t)
{
	mpu6050_t->init_flag = 7; //default no inited
	mpu6050_t->hi2cx->i2cx = I2Cx_PORT;
	
	MPU6050_WriteReg(mpu6050_t,MPU6050_PWR_MGMT_1, 0x01);
	MPU6050_WriteReg(mpu6050_t,MPU6050_PWR_MGMT_2, 0x00); //default
	
//	MPU6050_WriteReg(mpu6050_t,MPU6050_SMPLRT_DIV, 0x07); //sample rate
//	MPU6050_WriteReg(mpu6050_t,MPU6050_DLPF_CONFIG, 0x06); //DLPF_CFG
//	MPU6050_WriteReg(mpu6050_t,MPU6050_GYRO_CONFIG, 0x18); //full scale
//	MPU6050_WriteReg(mpu6050_t,MPU6050_ACCEL_CONFIG, 0x18); //full scale

	MPU6050_SetSampleRate(mpu6050_t,SAMPLE_RATE_DIV8);
	MPU6050_SetLPF(mpu6050_t,LPF_CUTOFF_FREQ_5HZ);
	MPU6050_SetGyroFS(mpu6050_t,GYRO_FS_SEL_2000DPS);
	MPU6050_SetAccelFS(mpu6050_t,ACCEL_FS_SEL_16G);
}
void MPU6050_GetACCEL(MPU6050_Info_t *mpu6050_t,int16_t *ACCEL_Array)
{
	uint8_t data0;
	uint8_t data1;
	
	MPU6050_ReadReg(mpu6050_t,MPU6050_ACCEL_XOUT_L,&data0,1);
	MPU6050_ReadReg(mpu6050_t,MPU6050_ACCEL_XOUT_H,&data1,1);
	ACCEL_Array[0] = MPU6050_Byte_to_HalfWord(data0,data1);
	
	MPU6050_ReadReg(mpu6050_t,MPU6050_ACCEL_YOUT_L,&data0,1);
	MPU6050_ReadReg(mpu6050_t,MPU6050_ACCEL_YOUT_H,&data1,1);
	ACCEL_Array[1] = MPU6050_Byte_to_HalfWord(data0,data1);
	
	MPU6050_ReadReg(mpu6050_t,MPU6050_ACCEL_ZOUT_L,&data0,1);
	MPU6050_ReadReg(mpu6050_t,MPU6050_ACCEL_ZOUT_H,&data1,1);
	ACCEL_Array[2] = MPU6050_Byte_to_HalfWord(data0,data1);
}

void MPU6050_GetGYRO(MPU6050_Info_t *mpu6050_t,int16_t *GYRO_Array)
{
	uint8_t data0;
	uint8_t data1;
	
	MPU6050_ReadReg(mpu6050_t,MPU6050_GYRO_XOUT_L,&data0,1);
	MPU6050_ReadReg(mpu6050_t,MPU6050_GYRO_XOUT_H,&data1,1);
	GYRO_Array[0] = MPU6050_Byte_to_HalfWord(data0,data1);
	
	MPU6050_ReadReg(mpu6050_t,MPU6050_GYRO_YOUT_L,&data0,1);
	MPU6050_ReadReg(mpu6050_t,MPU6050_GYRO_YOUT_H,&data1,1);
	GYRO_Array[1] = MPU6050_Byte_to_HalfWord(data0,data1);
	
	MPU6050_ReadReg(mpu6050_t,MPU6050_GYRO_ZOUT_L,&data0,1);
	MPU6050_ReadReg(mpu6050_t,MPU6050_GYRO_ZOUT_H,&data1,1);
	GYRO_Array[2] = MPU6050_Byte_to_HalfWord(data0,data1);
}

void MPU6050_GetTemp(MPU6050_Info_t *mpu6050_t,int16_t *Temp_Array)
{
	uint8_t data0;
	uint8_t data1;
	
	MPU6050_ReadReg(mpu6050_t,MPU6050_TEMP_OUT_L,&data0,1);
	MPU6050_ReadReg(mpu6050_t,MPU6050_TEMP_OUT_H,&data1,1);
	Temp_Array[0] = MPU6050_Byte_to_HalfWord(data0,data1);
}
	

void MPU6050_GetAccel_Value(MPU6050_Info_t *mpu6050_t,float *Accel_Value)
{
	int16_t ACCEL_Array[3] = {0};
	
	MPU6050_GetACCEL(mpu6050_t,ACCEL_Array);
	
	switch(mpu6050_t->accel_fs_sel)
	{
		case ACCEL_FS_SEL_2G:
			Accel_Value[0] = (double)ACCEL_Array[0] / 16384.0;
			Accel_Value[1] = (double)ACCEL_Array[1] / 16384.0;
			Accel_Value[2] = (double)ACCEL_Array[2] / 16384.0;
			break;

		case ACCEL_FS_SEL_4G:
			Accel_Value[0] = (double)ACCEL_Array[0] / 8192.0;
			Accel_Value[1] = (double)ACCEL_Array[1] / 8192.0;
			Accel_Value[2] = (double)ACCEL_Array[2] / 8192.0;
			break;

		case ACCEL_FS_SEL_8G:
			Accel_Value[0] = (double)ACCEL_Array[0] / 4096.0;
			Accel_Value[1] = (double)ACCEL_Array[1] / 4096.0;
			Accel_Value[2] = (double)ACCEL_Array[2] / 4096.0;
			break;

		case ACCEL_FS_SEL_16G:
			Accel_Value[0] = (double)ACCEL_Array[0] / 2048.0;
			Accel_Value[1] = (double)ACCEL_Array[1] / 2048.0;
			Accel_Value[2] = (double)ACCEL_Array[2] / 2048.0;
			break;

		default:
				break;
	}
}

void MPU6050_GetGyro_Value(MPU6050_Info_t *mpu6050_t,float *Gyro_Value)
{
	int16_t Gyro_Array[3] = {0};
	
	MPU6050_GetGYRO(mpu6050_t,Gyro_Array);
	
	switch(mpu6050_t->gyro_fs_sel)
	{
		case GYRO_FS_SEL_250DPS:
			Gyro_Value[0] = (double)Gyro_Array[0] / 131.0;
			Gyro_Value[1] = (double)Gyro_Array[1] / 131.0;
			Gyro_Value[2] = (double)Gyro_Array[2] / 131.0;
			break;

		case GYRO_FS_SEL_500DPS:
			Gyro_Value[0] = (double)Gyro_Array[0] / 65.5;
			Gyro_Value[1] = (double)Gyro_Array[1] / 65.5;
			Gyro_Value[2] = (double)Gyro_Array[2] / 65.5;
			break;

		case GYRO_FS_SEL_1000DPS:
			Gyro_Value[0] = (double)Gyro_Array[0] / 32.8;
			Gyro_Value[1] = (double)Gyro_Array[1] / 32.8;
			Gyro_Value[2] = (double)Gyro_Array[2] / 32.8;
			break;

		case GYRO_FS_SEL_2000DPS:
			Gyro_Value[0] = (double)Gyro_Array[0] / 16.4;
			Gyro_Value[1] = (double)Gyro_Array[1] / 16.4;
			Gyro_Value[2] = (double)Gyro_Array[2] / 16.4;
			break;

		default:
			break;
	}
		

}

float MPU6050_GetTemp_Value(MPU6050_Info_t *mpu6050_t)
{
	int16_t Temp_Array[1] = {0};
	MPU6050_GetTemp(mpu6050_t,Temp_Array);
	return ((double)Temp_Array[0])/340.0 + 36.53;
}





