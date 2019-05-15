/*
 * max17048.c
 *
 *  Created on: 2019Äê5ÔÂ10ÈÕ
 *      Author: Administrator
 */

#include "../emd_proctol/iic.h"
#include "../emd_proctol/uart.h"



uint16_t Max17048_ReadVersion(void)
{
	uint8_t version[2] = {0, 0};

	I2c_Master_Read_Slave(MAX17048_ADDR, MAX17048_VERSION, version, 2);

	return (version[0] << 8) + version[1];
}

float Max17048_ReadVCell(void)
{
	uint8_t vcell[2] = {0, 0};

	I2c_Master_Read_Slave(MAX17048_ADDR, MAX17048_VCELL, vcell, 2);

	uint16_t temp = (vcell[1] << 8) + vcell[0];

	return (float)temp * 78.125 / 1000000;
}

float Max17048_ReadSoc(void)
{
	uint8_t soc[2] = {0, 0};

	I2c_Master_Read_Slave(MAX17048_ADDR, MAX17048_SOC, soc, 2);

	uint16_t temp = (soc[1] << 8) + soc[0];

	return (float)temp / 256;
}
