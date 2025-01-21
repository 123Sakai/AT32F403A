#include "mpu6050.h"

#define SDA_SET gpio_bits_set(MPU_SCL_GPIO_PORT,MPU_SCL_PIN)
#define SDA_RESET gpio_bits_reset(MPU_SCL_GPIO_PORT,MPU_SCL_PIN)
#define SCL_SET gpio_bits_set(MPU_SCL_GPIO_PORT,MPU_SCL_PIN)
#define SCL_RESET gpio_bits_reset(MPU_SCL_GPIO_PORT,MPU_SCL_PIN)

int fd;
uint8_t i2c_read()
{
	uint8_t i = 0;
	uint8_t byte = 0;
	uint8_t sda_value = 0;
	SDA_SET;
	for(i=0;i<8;i++)
	{
		SCL_SET;
		if(gpio_input_data_read(MPU_SDA_GPIO_PORT,MPU_SDA_PIN) == 1)
		{
			byte |= (0x80 >> i);
			SCL_RESET;
		}
		delay_ms(1);
	}
	return byte;
}
	
void i2c_write(uint8_t byte)
{
	uint8_t i =0;
	for(i=0;i<8;i++)
	{
		if(byte & (0x80 >>i))
		{
			SDA_SET;
		}
		else
		{
			SDA_RESET;
		}
		SCL_SET;
		delay_ms(10);
		SCL_RESET;
	}
}
void i2c_recvAck(void)
{
	uint8_t ackbit = 0;
	gpio_bits_set(MPU_SDA_GPIO_PORT,MPU_SDA_PIN);
	gpio_bits_set(MPU_SCL_GPIO_PORT,MPU_SCL_PIN);
	ack_bit = gpio_input_data_read(MPU_SDA_GPIO_PORT,MPU_SDA_PIN);
	delay_ms(10);
	gpio_bits_reset(MPU_SCL_GPIO_PORT,MPU_SCL_PIN);
}

void MPU6050_WriteReg(uint8_t Reg, uint8_t Data)
{
	/*发送从机地址*/
	i2c_start_generate(I2C1);
	i2c_data_send(I2C1,MPU6050_ADDRESS); //发MPU6050地址 第0位为0是写操作，1是读操作
	i2c_recvAck();
	/*发送需要写操作的寄存器*/
	i2c_data_send(I2C1,Reg);            
	i2c_recvAck(); 
	
	/*发送需要写入的数据*/
	i2c_data_send(I2C1,Data);
	i2c_recvAck();
	
	i2c_stop_generate(I2C1);
}

void MPU6050_WriteBuf(uint8_t BufAddress, uint8_t *BufData, uint8_t Length)
{
	uint8_t i = 0;
	
	i2c_start_generate(I2C1);
	i2c_write(MPU6050_ADDRESS | 0x00);  //发送从机地址  | 写模式
	i2c_recvAck();                      //接收应答
	i2c_write(BufAddress);              //发送寄存器首地址
	i2c_recvAck();                      //接收应答
	
	for (i = 0; i < Length; i++) 
	{
			i2c_write(BufData[i]);          //连续写入数据
			i2c_recvAck();                  //接收应答
	}
	i2c_stop_generate(I2C1); 
}

uint8_t MPU6050_ReadReg(uint8_t Reg)
{
	uint8_t Reg_Data;
	
	/*send slave addr*/
	i2c_start_generate(I2C1);
	i2c_data_send(I2C1,MPU6050_ADDRESS |0x00); //发MPU6050地址 低位0是写1是读
	i2c_recvAck();
	
	/*send need read data's register addr*/
	i2c_write(Reg);                   
	i2c_recvAck();
	
	/*send read mode*/
	i2c_start_generate(I2C1);
	i2c_data_send(I2C1,MPU6050_ADDRESS | 0x01);  //发送MPU6050地址 | 0x01 读操作
	i2c_recvAck();
	
	/*read slave data*/
	Reg_Data = i2c_read();
	//read ending ,send a set ack_bit
	SDA_SET;
	SCL_SET;
	SCL_RESET;
	i2c_stop_generate(I2C1); 
	
	return Reg_Data;
}

void MPU6050_ReadBuf(uint8_t BufAddress, uint8_t *BufData, uint8_t Length)
{
    uint8_t i;
    
    i2c_start_generate(I2C1);
    i2c_write(MPU6050_ADDRESS | 0x00); //发送从机地址 | 写模式
    i2c_recvAck();
    i2c_write(BufAddress);             //发送寄存器地址
    i2c_recvAck();
    
    i2c_start_generate(I2C1);
    i2c_write(MPU6050_ADDRESS | 0x01); //发送从机地址 | 读模式
    i2c_recvAck();
    
    /*连续读取数据*/
    for (i = 0; i < Length; i++) 
    {
        BufData[i] = i2c_recv();   //把数据存放在i指向的变量
        if( i < (Length - 1) )              
        {
            MyI2C_SendAck(0);               //发送应答
        }
        else                                
        {
            MyI2C_SendAck(1);               //最后一个要发送非应答
        }
    }
    i2c_stop_generate(I2C1); 
}

uint8_t MPU6050_GetID(void)
{
	uint8_t get_id = 0;
	get_id = i2c_data_receive(MPU6050_WHO_AM_I);
}

void MPU6050_Init(void)
{
    
    /*MPU6050寄存器初始化, 需要对照手册描述来配置*/
    MPU6050_WriteReg(MPU6050_PWR_MGMT_1, 0x01);   //电源管理1寄存器，取消睡眠模式，选择时钟源为X轴陀螺仪
    MPU6050_WriteReg(MPU6050_PWR_MGMT_2, 0x00);   //电源管理2寄存器，保持默认值，所有轴不休眠。
    MPU6050_WriteReg(MPU6050_SMPLRT_DIV, 0x09);   //采样率分频寄存器，
    MPU6050_WriteReg(MPU6050_CONFIG, 0x06);       //配置寄存器
    MPU6050_WriteReg(MPU6050_GYRO_CONFIG, 0x18);  //陀螺仪配置寄存器，选择满量程 ±2000°/s
    MPU6050_WriteReg(MPU6050_ACCEL_CONFIG, 0x18); //加速度计配置寄存器，选择满量程 ±16g
    
}

void MPU6050_GetACCEL(int16_t *ACCEL_Array){}
void MPU6050_GetGYRO(int16_t *GYRO_Array){}
 
void MPU6050_GetAccel_Value(float *Accel_Value){}
void MPU6050_GetGyro_Value(float *Gyro_Value){}
 