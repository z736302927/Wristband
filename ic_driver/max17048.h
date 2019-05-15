/*
 * max17048.h
 *
 *  Created on: 2019Äê5ÔÂ14ÈÕ
 *      Author: Administrator
 */

#ifndef MAIN_IC_DRIVER_MAX17048_H_
#define MAIN_IC_DRIVER_MAX17048_H_


#define	MAX17048_ADDR		0x36

#define	MAX17048_VCELL		0x02
#define	MAX17048_SOC		0x04
#define	MAX17048_MODE		0x06
#define	MAX17048_VERSION	0x08
#define	MAX17048_HIBRT		0x0A
#define	MAX17048_CONFIG		0x0C
#define	MAX17048_VALRT		0x14
#define	MAX17048_CRATE		0x16
#define	MAX17048_VRESET_ID	0x18
#define	MAX17048_STATUS		0x1A
#define	MAX17048_TABLE		0x40
#define	MAX17048_CMD		0xFE





uint16_t Max17048_ReadVersion(void);

#endif /* MAIN_IC_DRIVER_MAX17048_H_ */
