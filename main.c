/* Blink Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_system.h"
#include "esp_spi_flash.h"
#include "sdkconfig.h"

#include "emd_proctol/iic.h"
#include "emd_proctol/iic.c"
#include "emd_proctol/uart.h"
#include "emd_proctol/uart.c"


#include "ic_driver/max17048.h"
#include "ic_driver/max17048.c"
#include "ic_driver/max30205.h"
#include "ic_driver/max30205.c"
#include "ic_driver/max77752.h"
#include "ic_driver/max77752.c"
#include "ic_driver/rgb_led.h"
#include "ic_driver/rgb_led.c"

void app_main()
{
	uint8_t reg_data = 0;
	I2c_Master_Init();
	MAX77752_Init();
	RGB_LedInit();
	while(1)
	{
		printf("\r\nTemperature is %f¡æ\r\n",MAX30205_ReadTemperature());
		vTaskDelay(1000 / portTICK_RATE_MS);
		RGB_ColorTest();
//		MAX77752_Init();
		MAX77752_ReadBUCKVoltage(MAX77752_BUCK1);
		MAX77752_ReadBUCKVoltage(MAX77752_BUCK2);
		MAX77752_ReadBUCKVoltage(MAX77752_BUCK3);
		MAX77752_ReadBUCKVoltage(MAX77752_LDO);
		I2c_Master_Read_Slave(MAX77752_ADDR, MAX77752_OPMD1, &reg_data, 1);
		disp_buf(&reg_data, 1);
	}
}
