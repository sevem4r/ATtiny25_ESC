/*
 * main.h
 *
 *  Created on: 8 sty 2022
 *      Author: mariusz
 */

#ifndef MAIN_H_
#define MAIN_H_

#define TIME_CENTER			1500
#define TIME_MIN			1000
#define TIME_MAX			2000
#define TIME_MIN_TOL		500
#define TIME_MAX_TOL		2500

#define TIME_FWD_START		1600
#define TIME_FWD_STOP		1550

#define TIME_BWD_START		1400
#define TIME_BWD_STOP		1450

#define TIME_ERROR_MAX		100

#define VOLTAGE_SCALE		8169
#define VOLTAGE_TOL			200
#define VOLTAGE_3S_MAX		12600
#define VOLTAGE_3S_LOW		9000
#define VOLTAGE_2S_MAX		8400
#define VOLTAGE_2S_LOW		6000

#define LED			(1<<PB0)
#define LED_ON		(PORTB |= LED)
#define LED_OFF		(PORTB &= ~LED)
#define LED_TOG		(PORTB ^= LED)

enum STATE{
	STATE_STARTUP,
	STATE_OK,
	STATE_LOW_VOLTAGE
};

enum DIR{
	DIR_LEFT,
	DIR_RIGHT
};

enum LIPO_TYPE{
	LIPO_TYPE_2S,
	LIPO_TYPE_3S
};

enum LIPO_DETECT{
	LIPO_DETECT_OK,
	LIPO_DETECT_ERROR
};

typedef struct{
	uint8_t type;
	uint16_t voltage;
	uint16_t voltage_low;
}LiPo;


#endif /* MAIN_H_ */
