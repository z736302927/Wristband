/*
 * rgb_led.h
 *
 *  Created on: 2019Äê5ÔÂ15ÈÕ
 *      Author: Administrator
 */

#ifndef MAIN_IC_DRIVER_RGB_LED_H_
#define MAIN_IC_DRIVER_RGB_LED_H_

typedef struct
{
	uint8_t R;
	uint8_t G;
	uint8_t B;

}RGB_LedTypeDef;


#define COLOR_WHITE        0xFFFFFF
#define COLOR_BLACK        0x000000
#define COLOR_RED          0xFF0000
#define COLOR_ORANGE       0xFF7F00
#define COLOR_YELLOW       0xFFFF00
#define COLOR_GREEN        0x00FF00
#define COLOR_CYAN         0x00FFFF
#define COLOR_BLUE         0x0000FF
#define COLOR_PURPLE       0x8B00FF



#endif /* MAIN_IC_DRIVER_RGB_LED_H_ */
