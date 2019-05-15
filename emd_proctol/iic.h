/*
 * iic.h
 *
 *  Created on: 2019Äê5ÔÂ11ÈÕ
 *      Author: Administrator
 */

#ifndef MAIN_EMD_PROCTOL_IIC_H_
#define MAIN_EMD_PROCTOL_IIC_H_


void iic_test();

esp_err_t I2c_Master_Read_Slave(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data_rd, size_t size);

esp_err_t I2c_Master_Write_Slave(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data_wr, size_t size);

/***************I2C GPIO******************/
#define I2C_Pin_SCL		GPIO_NUM_27
#define I2C_Pin_SDA		GPIO_NUM_13
/*********************************************/

#define SCL_H         gpio_set_level(I2C_Pin_SCL, 1)
#define SCL_L         gpio_set_level(I2C_Pin_SCL, 0)
#define SDA_H         gpio_set_level(I2C_Pin_SDA, 1)
#define SDA_L         gpio_set_level(I2C_Pin_SDA, 0)

#define SDA_read      gpio_get_level(I2C_Pin_SDA)

extern volatile uint8_t I2C_FastMode;

void I2c_Soft_Init(void);
void I2c_Soft_SendByte(uint8_t SendByte);
uint8_t I2c_Soft_ReadByte(uint8_t);

//int I2c_Soft_Single_Write(uint8_t SlaveAddress,uint8_t REG_Address,uint8_t REG_data);
//int I2c_Soft_Single_Read(uint8_t SlaveAddress,uint8_t REG_Address);
//int I2c_Soft_Mult_Read(uint8_t SlaveAddress,uint8_t REG_Address,uint8_t * ptChar,uint8_t size);

uint8_t IIC_Write_1Byte(uint8_t SlaveAddress,uint8_t REG_Address,uint8_t REG_data);
uint8_t IIC_Read_1Byte(uint8_t SlaveAddress,uint8_t REG_Address,uint8_t *REG_data);
uint8_t IIC_Write_nByte(uint8_t SlaveAddress, uint8_t REG_Address, uint8_t len, uint8_t *buf);
uint8_t IIC_Read_nByte(uint8_t SlaveAddress, uint8_t REG_Address, uint8_t len, uint8_t *buf);



#endif /* MAIN_EMD_PROCTOL_IIC_H_ */

