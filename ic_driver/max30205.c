/*
 * max30205.c
 *
 *  Created on: 2019Äê5ÔÂ10ÈÕ
 *      Author: Administrator
 */


#include "../emd_proctol/iic.h"
#include "../emd_proctol/uart.h"

float MAX30205_ReadTemperature(void)
{
	uint8_t temperature[2] = {0, 0};

	I2c_Master_Read_Slave(MAX30205_ADDR, MAX30205_TEMPERATURE, temperature, 2);

	int16_t temp = (temperature[0] << 8) + temperature[1];

	return (float)temp * 0.00390625;
}


void MAX30205_WriteConfiguration(void)
{
	uint8_t configuration = 0xFF;
	uint8_t temp = 0x55;

	I2c_Master_Write_Slave(MAX30205_ADDR, MAX30205_CONFIGURATION, &temp, 1);

	I2c_Master_Read_Slave(MAX30205_ADDR, MAX30205_CONFIGURATION, &configuration, 1);

	disp_buf(&configuration, 1);
}

void MAX30205_WriteTHYST(void)
{
	uint8_t thyst[2] = {0xFF, 0xFF};
	uint8_t temp[2] = {0x55, 0x55};

	I2c_Master_Write_Slave(MAX30205_ADDR, MAX30205_THYST, temp, 2);

	I2c_Master_Read_Slave(MAX30205_ADDR, MAX30205_THYST, thyst, 2);

	disp_buf(&thyst, 2);
}
