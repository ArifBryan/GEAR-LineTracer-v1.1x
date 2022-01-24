/*
ADc Driver Library

Example:
//ADc Initialization
ADc.Init(1,1,1,0,ADCPRESC_2,0,ADCTRIG_FREERUN,0b11111111);
//ADc Read Result
Result = ADc.Read(1,ADCREF_AREF);

- Arif Bryan
*/

#ifndef _ADc_H
#define _ADc_H

#include <util/delay.h>
#include "bytecon.h"

#define ADCMUX_TEMP	0b1000
#define ADCMUX_VBG	0b1110
#define ADCMUX_GND	0b1111
#define ADCREF_AREF 0b00
#define ADCREF_AVCC 0b01
#define ADCREF_INT	0b11
#define ADCPRESC_2		0b001
#define ADCPRESC_4		0b010
#define ADCPRESC_8		0b011
#define ADCPRESC_16		0b100
#define ADCPRESC_32		0b101
#define ADCPRESC_64		0b110
#define ADCPRESC_128	0b111
#define ADCTRIG_FREERUN 0b000
#define ADCTRIG_ACOMP	0b001
#define ADCTRIG_EXTINT	0b010

class ADc{
	public:
		void Init(bool,bool,bool,bool,uint8_t,bool,uint8_t);
		void DisableDigitalIn(uint8_t);
		uint16_t Read(uint8_t,uint8_t);
}ADc;

void ADc::Init(bool enable,bool startconv,bool autotrig,bool interruptenable,uint8_t prescaler,bool compmuxen,uint8_t trigsrc){
	ADCSRA=(enable<<ADEN)|(startconv<<ADSC)|(autotrig<<ADATE)|(interruptenable<<ADIE)|((prescaler/4%2)<<ADPS2)|((prescaler/2%2)<<ADPS1)|((prescaler%2)<<ADPS0);
	ADCSRB=(compmuxen<<ACME)|((trigsrc/4%2)<<ADTS2)|((trigsrc/2%2)<<ADTS1)|((trigsrc%2)<<ADTS0);
}
void ADc::DisableDigitalIn(uint8_t pin){
	DIDR0=pin;
}
uint16_t ADc::Read(uint8_t mux,uint8_t vref){
	uint8_t adcsra_t;
	adcsra_t=ADCSRA;
	ADMUX=((vref/2%2)<<REFS1)|((vref%2)<<REFS0)|(0<<ADLAR)|((mux/8%2)<<MUX3)|((mux/4%2)<<MUX2)|((mux/2%2)<<MUX1)|((mux%2)<<MUX0);
	_delay_us(20);
	ADCSRA=(1<<ADSC)|adcsra_t;
	while(_READ(&ADCSRA,ADSC));
	return ADCL|(ADCH<<8);
}
#endif