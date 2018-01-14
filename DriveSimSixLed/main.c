#define F_CPU 16000000UL
#define BAUD 9600
#define ANALOG_STICK_MAX 1024

#define HORIZ_PIN 1
#define VERT_PIN 0

#include <util/setbaud.h>
#include <avr/io.h>
#include <util/delay.h>
#include <stdio.h>
#include <stdlib.h>
#include "uart.h"

void adc_init()
{
	// AREF = AVcc
	ADMUX = (1<<REFS0);
	
	// ADC Enable and pre-scaler of 128
	// 16000000/128 = 125000
	ADCSRA=(1<<ADEN)|(1<<ADPS2)|(1<<ADPS1)|(1<<ADPS0);
}


uint16_t read_adc(uint8_t ch) {
	// Select pin to read
	ADMUX &= 0xf0;
	ADMUX |= ch;
	// Starts the conversion
	ADCSRA |= _BV(ADSC);
	// Wait till conversion is ready
	while( (ADCSRA & _BV(ADSC)) );
	return ADC;
}

void wait() {
	uint8_t i;
	for(i=0;i<20;i++)
	_delay_loop_2(0);
}

void pwm_init() {
	// Timer 0
	//=============================================
	// set none-inverting mode
	TCCR0A |= _BV(COM0A1);
	// set PWN Phase Corrected
	TCCR0A |= _BV(WGM00);
	// set pre-scaler to 8 and starts PWM
	TCCR0B |= _BV(CS01);
	//=============================================
	
	//Timer 1
	//=============================================
	 // set none-inverting mode
	 TCCR1A |= _BV(COM1A1) | _BV(COM1B1);
	// set PWN Phase Corrected
	 TCCR1A |= _BV(WGM10);
	// set pre-scaler to 8 and starts PWM
	 TCCR1B |= _BV(CS11);
	 //=============================================
	
	//Time 2
	//=============================================
	// set none-inverting mode
	TCCR2A |= _BV(COM2A1);
	// set fast PWM Mode
	TCCR2A |= _BV(WGM20);
	// set prescaler to 8 and starts PWM
	TCCR2B |= _BV(CS21);
	//=============================================
}

int main(void){
	
	uint16_t adc_result_horiz;
	uint16_t adc_result_vert;
	float magnitude;
	float angle;
	int left_mag;
	int right_mag;

	//Initialize
	stdout = &uart_output;
	uart_init();
	adc_init();
	pwm_init();
	fflush(stdout);
	
	
	// Set pin 6 to be digital output
	DDRB = _BV(DDB3) | _BV(DDB2) | _BV(DDB1); // pin 11 (bottom L), pin 10 (middle R), pin 9 (middle L)
	PORTB |= _BV(PORTB3) | _BV(PORTB2) | _BV(PORTB1); 
	DDRD = _BV(DDD6) | _BV(DDD5) | _BV(DDD3) | _BV(DDD2); // pin 6 (top L), pin 5 (top R), pin 3 (bottom R), pin 2 (dir)
	PORTD |= _BV(PORTD6) | _BV(PORTD5) | _BV(PORTD3) | _BV(PORTD2);
	
	
	while(1) {
		adc_result_horiz = read_adc(HORIZ_PIN); 
		adc_result_vert = read_adc(VERT_PIN);
		//printf("HORIZ, VERT: %i, %i\n", adc_result_horiz, adc_result_vert);
		
		angle = (float)adc_result_vert/ANALOG_STICK_MAX;
		magnitude = (float)adc_result_horiz/ANALOG_STICK_MAX;
		
		// determine direction
		//printf("Magnitude: %i\n", (int)(magnitude*1000));
		if(magnitude > 0.5){
			PORTD &= ~(_BV(PORTD2));
		}
		else{  
 			PORTD |= _BV(PORTD2);
		}
		
		// Left side
		left_mag = abs((int)((magnitude - 0.5) * angle * 512));
		OCR0A = left_mag;
		OCR1A = left_mag;
		OCR2A = left_mag;
		// Right Side 
		right_mag = abs((int)((magnitude - 0.5) * (1 - angle) * 512));
		OCR0B = right_mag;
		OCR1B = right_mag;
		OCR2B = right_mag;
		//printf("left, right: %i, %i\n", (int)left_mag, (int)right_mag);
		_delay_ms(10);
	}
}