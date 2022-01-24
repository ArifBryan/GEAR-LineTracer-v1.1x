#ifndef _TIMERLIB_H
#define _TIMERLIB_H

#define TIMERPRESC_STOP	0b000
#define TIMERPRESC_1	0b001
#define TIMERPRESC_8	0b010
#define TIMERPRESC_64	0b011
#define TIMERPRESC_256	0b100
#define TIMERPRESC_1024	0b101

void TimerInit(uint8_t prescaler, uint8_t compare){
	TCCR0 = (0<<WGM00)|(0<<COM01)|(0<<COM00)|(1<<WGM01)|(_READ(&prescaler,2)<<CS02)|(_READ(&prescaler,1)<<CS01)|(_READ(&prescaler,0)<<CS00);
	OCR0 = compare;
	TIMSK = (1<<OCIE0);
}
#endif