/*
 * max30205.h
 *
 *  Created on: 2019Äê5ÔÂ14ÈÕ
 *      Author: Administrator
 */

#ifndef MAIN_IC_DRIVER_MAX30205_H_
#define MAIN_IC_DRIVER_MAX30205_H_

#define MAX30205_ADDR 			0x48

// Registers
#define MAX30205_TEMPERATURE    0x00  //  get temperature ,Read only
#define MAX30205_CONFIGURATION  0x01  //
#define MAX30205_THYST          0x02  //
#define MAX30205_TOS            0x03  //

float MAX30205_ReadTemperature(void);

#endif /* MAIN_IC_DRIVER_MAX30205_H_ */
