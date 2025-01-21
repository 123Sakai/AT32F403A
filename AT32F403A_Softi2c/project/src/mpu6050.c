#include "mpu6050.h"
#include "DELAY_ONE.h"

//i2c
uint8_t i2c_SCL_WriteorRead(bool readN,uint8_t value)
{
	if(readN)//w
	{
		gpio_bits_write(MPU6050_SCL_GPIO_PORT,MPU6050_SCL_PIN,(confirm_state)value);
	}
	else
	{
		value = gpio_input_data_bit_read(MPU6050_SCL_GPIO_PORT,MPU6050_SCL_PIN);
	}
	delay_us(10);
	return value;
}

uint8_t i2c_SDA_WriteorRead(bool readN,uint8_t value)
{
	if(readN)//w
	{
		gpio_bits_write(MPU6050_SDA_GPIO_PORT,MPU6050_SDA_PIN,(confirm_state)value);
	}
	else
	{
		value = gpio_input_data_bit_read(MPU6050_SDA_GPIO_PORT,MPU6050_SDA_PIN);
	}
	delay_us(10);
	return value;
}

//开始信号：SCL 为高电平时，SDA 由高电平变为低电平
void i2c_start(void)
{
	i2c_SDA_WriteorRead(true,1);
	i2c_SCL_WriteorRead(true,1);
	i2c_SDA_WriteorRead(true,0);
	i2c_SCL_WriteorRead(true,0);
}
//结束信号：SCL 为高电平时，SDA 由低电平变为高电平。
void i2c_stop(void)
{
	i2c_SDA_WriteorRead(true,0);
	i2c_SCL_WriteorRead(true,1);
	i2c_SDA_WriteorRead(true,1);
}

uint8_t i2c_read(void)
{
	uint8_t i = 0;
	uint8_t byte = 0;
	i2c_SDA_WriteorRead(true,1);
	for(i=0;i<8;i++)
	{
		i2c_SCL_WriteorRead(true,1);
		if(i2c_SDA_WriteorRead(false,0))
		{
			byte |= (0x80 >> i);
		}
		i2c_SCL_WriteorRead(true,0);
	}
	return byte;
}
	
void i2c_write(uint8_t byte)
{
	uint8_t i =0;
	uint8_t tmp_bit = 0;
	
	for(i=0;i<8;i++)
	{
		tmp_bit = (byte & (0x80 >>i));
		i2c_SDA_WriteorRead(true,tmp_bit);
		i2c_SCL_WriteorRead(true,1);
		i2c_SCL_WriteorRead(true,0);
	}
}

uint8_t i2c_recvAck(void)
{
	uint8_t ack_bit = 0;
	i2c_SDA_WriteorRead(true,1);
	i2c_SCL_WriteorRead(true,1);
	ack_bit = i2c_SDA_WriteorRead(false,0);
	i2c_SCL_WriteorRead(true,0);
	return ack_bit;
}

void i2c_sendAck(uint8_t ack_bit)
{
	i2c_SDA_WriteorRead(true,ack_bit);
	i2c_SCL_WriteorRead(true,1);
	i2c_SCL_WriteorRead(true,0);
}
//6050
void MPU6050_WriteBuf(uint8_t BufAddress, uint8_t *BufData, uint8_t Length)
{
	uint8_t i = 0;
	
	i2c_start();												//S
	i2c_write(MPU6050_ADDRESS | 0x00);  //AD+W
	i2c_recvAck();                      //ACK
	i2c_write(BufAddress);              //RA
	i2c_recvAck();                      //ACK
	
	for (i = 0; i < Length; i++) 
	{
			i2c_write(BufData[i]);					//DATA
			i2c_recvAck();                  //ACK
	}
	i2c_stop(); 												//P
}

void MPU6050_ReadBuf(uint8_t BufAddress, uint8_t *BufData, uint8_t Length)
{
    uint8_t i;
    
    i2c_start();											 //S
    i2c_write(MPU6050_ADDRESS | 0x00); //AD+W
    i2c_recvAck();										 //ACK
    i2c_write(BufAddress);             //RA
    i2c_recvAck();										 //ACK
    
    i2c_start();											 //S
    i2c_write(MPU6050_ADDRESS | 0x01); //AD+R
    i2c_recvAck();										 //ACK
    
    
    for (i = 0; i < Length; i++) 
    {
        BufData[i] = i2c_read();				//DATA
        if( i < (Length - 1) )         
        {
            i2c_sendAck(0);							//ACK
        }
        else                                
        {
            i2c_sendAck(1);							//NACK
        }
    }
    i2c_stop(); 												//P
}

void MPU6050_WriteReg(uint8_t RegAddr, uint8_t Data)
{
	i2c_start();												//S
	i2c_write(MPU6050_ADDRESS | 0x00);  //AD+W
	i2c_recvAck();                      //ACK
	i2c_write(RegAddr);              		//RA
	i2c_recvAck();                      //ACK

	i2c_write(Data);										//DATA
	i2c_recvAck();                 	   	//ACK
	i2c_stop(); 												//P
}

uint8_t MPU6050_ReadReg(uint8_t RegAddr)
{
	uint8_t Data=0;
	
	i2c_start();											 //S
	i2c_write(MPU6050_ADDRESS | 0x00); //AD+W
	i2c_recvAck();										 //ACK
	i2c_write(RegAddr);             	 //RA
	i2c_recvAck();										 //ACK
	
	i2c_start();											 //S
	i2c_write(MPU6050_ADDRESS | 0x01); //AD+R
	i2c_recvAck();										 //ACK

	Data = i2c_read();								//DATA
	i2c_sendAck(1);										//NACK

	i2c_stop(); 												//P
	return Data;
}

uint8_t MPU6050_GetID(void)
{
	uint8_t get_id = 0;
	get_id = MPU6050_ReadReg(MPU6050_WHO_AM_I);
	return get_id;
}

void MPU6050_Init(void)
{
	/*MPU6050寄存器初始化, 需要对照手册描述来配置*/
	MPU6050_WriteReg(MPU6050_PWR_MGMT_1, 0x01);   //电源管理1寄存器，取消睡眠模式，选择时钟源为X轴陀螺仪
	MPU6050_WriteReg(MPU6050_PWR_MGMT_2, 0x00);   //电源管理2寄存器，保持默认值，所有轴不休眠。
	MPU6050_WriteReg(MPU6050_SMPLRT_DIV, 0x07);   //采样率分频寄存器，
	MPU6050_WriteReg(MPU6050_CONFIG, 0x06);       //配置寄存器
	MPU6050_WriteReg(MPU6050_GYRO_CONFIG, 0x18);  //陀螺仪配置寄存器，选择满量程 ±2000°/s
	MPU6050_WriteReg(MPU6050_ACCEL_CONFIG, 0x18); //加速度计配置寄存器，选择满量程 ±16g
}

int16_t MPU6050_Byte_to_HalfWord(uint8_t DataL, uint8_t DataH)
{
    int16_t Data;
    Data = (DataH << 8) | DataL;
    return Data;
}

void MPU6050_GetACCEL(int16_t *ACCEL_Array)
{
	uint8_t data0;
	uint8_t data1;
	
	data0 = MPU6050_ReadReg(MPU6050_ACCEL_XOUT_L);
	data1 = MPU6050_ReadReg(MPU6050_ACCEL_XOUT_H);
	ACCEL_Array[0] = MPU6050_Byte_to_HalfWord(data0,data1);
	
	data0 = MPU6050_ReadReg(MPU6050_ACCEL_YOUT_L);
	data1 = MPU6050_ReadReg(MPU6050_ACCEL_YOUT_H);
	ACCEL_Array[1] = MPU6050_Byte_to_HalfWord(data0,data1);
	
	data0 = MPU6050_ReadReg(MPU6050_ACCEL_ZOUT_L);
	data1 = MPU6050_ReadReg(MPU6050_ACCEL_ZOUT_H);
	ACCEL_Array[2] = MPU6050_Byte_to_HalfWord(data0,data1);
}
void MPU6050_GetGYRO(int16_t *GYRO_Array)
{
	uint8_t data0;
	uint8_t data1;
	
	data0 = MPU6050_ReadReg(MPU6050_GYRO_XOUT_L);
	data1 = MPU6050_ReadReg(MPU6050_GYRO_XOUT_H);
	GYRO_Array[0] = MPU6050_Byte_to_HalfWord(data0,data1);
	
	data0 = MPU6050_ReadReg(MPU6050_GYRO_YOUT_L);
	data1 = MPU6050_ReadReg(MPU6050_GYRO_YOUT_H);
	GYRO_Array[1] = MPU6050_Byte_to_HalfWord(data0,data1);
	
	data0 = MPU6050_ReadReg(MPU6050_GYRO_ZOUT_L);
	data1 = MPU6050_ReadReg(MPU6050_GYRO_ZOUT_H);
	GYRO_Array[2] = MPU6050_Byte_to_HalfWord(data0,data1);
}

void MPU6050_GetTemp(int16_t *Temp_Array)
{
	uint8_t data0;
	uint8_t data1;
	
	data0 = MPU6050_ReadReg(MPU6050_TEMP_OUT_L);
	data1 = MPU6050_ReadReg(MPU6050_TEMP_OUT_H);
	Temp_Array[0] = MPU6050_Byte_to_HalfWord(data0,data1);
}
	
//param:采样值 量程上限 量程下限
float Scale_Transform(float Sample_Value, float URV, float LRV)
{
    float data;             //定义用来保存变换后的数据变量
    float value_low = -32767.0; //定义采样值下限变量   MPU6050寄存器是16位的，最高位是符号位，
    float value1_up = 32767.0;  //定义采样值上限变量   所以寄存器输出范围是-7FFF~7FFF,对应十进制-32767~32767
    
    /* 公式：当前数据 =（采样值 - 采样值下限）/（采样值上限 - 采样值下限）*（量程上限 - 量程下限）+ 量程下限     */
    data = (Sample_Value - value_low) / (value1_up - value_low) * (URV - LRV) + LRV;
           
    return data;
}

void MPU6050_GetAccel_Value(float *Accel_Value)
{
	int16_t ACCEL_Array[3] = {0};
	
	MPU6050_GetACCEL(ACCEL_Array);
	
	Accel_Value[0] = Scale_Transform((float)ACCEL_Array[0],16.0,-16.0);
	Accel_Value[1] = Scale_Transform((float)ACCEL_Array[1],16.0,-16.0);
	Accel_Value[2] = Scale_Transform((float)ACCEL_Array[2],16.0,-16.0);
}
void MPU6050_GetGyro_Value(float *Gyro_Value)
{
	int16_t Gyro_Array[3] = {0};
	
	MPU6050_GetGYRO(Gyro_Array);
	
	Gyro_Value[0] = Scale_Transform((float)Gyro_Array[0],2000.0,-2000.0);
	Gyro_Value[1] = Scale_Transform((float)Gyro_Array[1],2000.0,-2000.0);
	Gyro_Value[2] = Scale_Transform((float)Gyro_Array[2],2000.0,-2000.0);
}

float MPU6050_GetTemp_Value(void)
{
	int16_t Temp_Array[1] = {0};
	
	MPU6050_GetTemp(Temp_Array);
	
	return ((float)Temp_Array[0])/340.0 + 36.53;
}





