/*
 * main.c
 *
 *  Created on: 20 gru 2021
 *      Author: mariusz
 */

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <util/atomic.h>
#include "main.h"

static volatile uint8_t warmup_counter_isr;
static volatile uint16_t time_isr;
static uint16_t time;
static uint8_t warmup_counter;
static uint8_t time_error_counter;
static uint8_t state;
static LiPo battery;

static void pwm_set(uint8_t value, uint8_t dir);
static uint16_t adc_battery_voltage(void);
static uint8_t detect_battery_type(void);

int main(void){
	uint8_t pwm;
	uint8_t dir;

//	// clock div: 2 (4MHz)
//	CLKPR = 0x80;
//	CLKPR = (1<<CLKPS0);

	// osc cal: 6,4MHz
	OSCCAL = 68;

	DDRB |= LED;
	DDRB |= (1<<PB1) | (1<<PB4);
	PORTB |= (1<<PB2);

	TCCR1 |= (1<<PWM1A);
	// OC1A (PB1 - 6) -> INB
	TCCR1 |= (1<<COM1A1);
	// OC1B (PB4 - 3) -> /INA
	GTCCR |= (1<<PWM1B);
	GTCCR |= (1<<COM1B1) | (1<<COM1B0);
	TCCR1 |= (1<<CS10);
	OCR1A = 0;
	OCR1B = 0;

	TCCR0B |= (1<<CS01) | (1<<CS00);

	MCUCR |= (1<<ISC01) | (1<<ISC00);
	GIMSK |= (1<<INT0);

	// ADC3
	// 10bit
	// ref: 2,56V
	ADMUX |= (1<<MUX1) | (1<<MUX0);
	ADMUX |= (1<<REFS1) |(1<<REFS2);
	ADCSRA |= (1<<ADEN) | (1<<ADPS2 | (1<<ADPS1));

	if(detect_battery_type() == LIPO_DETECT_OK){
		state = STATE_STARTUP;
	}else{
		state = STATE_LOW_VOLTAGE;
	}

	sei();

	while(1){

		switch(state){
		case STATE_STARTUP:
			ATOMIC_BLOCK(ATOMIC_RESTORESTATE){
				warmup_counter = warmup_counter_isr;
			}

			if(warmup_counter > 10){
				state = STATE_OK;
			}
			_delay_ms(10);
			break;

		case STATE_OK:
			ATOMIC_BLOCK(ATOMIC_RESTORESTATE){
				time = time_isr;
			}

			if(time > TIME_MAX && time < TIME_MAX_TOL){
				time = TIME_MAX;
			}

			if(time < TIME_MIN && time > TIME_MIN_TOL){
				time = TIME_MIN;
			}

			if(time >= TIME_MIN && time <= TIME_MAX){
				time_error_counter = 0;

				pwm = 0;
				dir = DIR_RIGHT;

				if(time > TIME_FWD_START){
					pwm = 255 - (TIME_MAX - time) / 2;
					dir = DIR_LEFT;
					LED_ON;
				}else if(time > TIME_CENTER && time < TIME_FWD_STOP){
					pwm = 0;
					dir = DIR_RIGHT;
					LED_OFF;
				}

				if(time < TIME_BWD_START){
					pwm = 255 - (time - TIME_MIN) / 2;
					dir = DIR_RIGHT;
					LED_ON;
				}else if(time <= TIME_CENTER && time > TIME_BWD_STOP){
					pwm = 0;
					dir = DIR_RIGHT;
					LED_OFF;
				}

				pwm_set(pwm, dir);
			}else{
				time_error_counter++;

				if(time_error_counter > TIME_ERROR_MAX){
					pwm_set(0, DIR_LEFT);
				}
			}

			battery.voltage = adc_battery_voltage();

			if(battery.voltage < battery.voltage_low){
				pwm_set(0, DIR_LEFT);

				state = STATE_LOW_VOLTAGE;
			}

			_delay_ms(10);

			break;

		case STATE_LOW_VOLTAGE:
			LED_TOG;
			_delay_ms(500);
			break;
		}
	}
}

ISR(INT0_vect){
	if(!(MCUCR & (1<<ISC00))){
		time_isr = TCNT0 * 10;

		warmup_counter_isr++;
	}

	TCNT0 = 0;

	MCUCR ^= (1<<ISC00);
}

static void pwm_set(uint8_t value, uint8_t dir){
	if(dir == DIR_LEFT){
		OCR1A = value;
		OCR1B = 0;
	}else{
		OCR1A = 0;
		OCR1B = value;
	}
}

static uint16_t adc_battery_voltage(void){
	static uint16_t voltage_raw = 1023;
	uint16_t voltage;
	uint16_t value;

	ADCSRA |= (1<<ADSC);

	while(ADCSRA & (1<<ADSC));

	value = ADC;

	voltage_raw = (voltage_raw * 90L + value * 10L) / 100L;
	voltage = (2560UL * voltage_raw / 1024UL) * VOLTAGE_SCALE / 1000UL;

	return voltage;
}

static uint8_t detect_battery_type(void){
	for(uint8_t i = 0; i < 128; i++){
		battery.voltage = adc_battery_voltage();
		_delay_ms(10);
	}

	if(battery.voltage > (VOLTAGE_2S_LOW - VOLTAGE_TOL) &&
	   battery.voltage < (VOLTAGE_2S_MAX + VOLTAGE_TOL)){
		battery.type = LIPO_TYPE_2S;
		battery.voltage_low = VOLTAGE_2S_LOW;
	}else if(battery.voltage > (VOLTAGE_3S_LOW - VOLTAGE_TOL) &&
			 battery.voltage < (VOLTAGE_3S_MAX + VOLTAGE_TOL)){
		battery.type = LIPO_TYPE_3S;
		battery.voltage_low = VOLTAGE_3S_LOW;
	}else{
		return LIPO_DETECT_ERROR;
	}

	return LIPO_DETECT_OK;
}
