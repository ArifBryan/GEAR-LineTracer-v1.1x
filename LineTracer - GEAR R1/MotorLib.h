#ifndef _MOTORLIB_H
#define _MOTORLIB_H

class MotorLib{
	public:
		void Init();
		void Config(uint16_t, uint8_t, uint8_t);
		void Drive(int16_t, int16_t);
	private:
		int16_t _motor_vLimit;	
		uint8_t _motor_ofL;
		uint8_t _motor_ofR;
};

void MotorLib::Init(){
	//Port Init
	_SET(MOTOR_L1_DDR,MOTOR_L1_Bit,1);
	_SET(MOTOR_L2_DDR,MOTOR_L2_Bit,1);
	_SET(MOTOR_R1_DDR,MOTOR_R1_Bit,1);
	_SET(MOTOR_R2_DDR,MOTOR_R2_Bit,1);
	//TCNT1 Init (Phase Correct PWM 10-Bit, Clk/8 Prescaler)
	TCCR1A = (1<<COM1A1)|(0<<COM1A0)|(1<<COM1B1)|(0<<COM1B0)|(1<<WGM11)|(1<<WGM10);
	TCCR1B = (0<<WGM13)|(0<<WGM12)|(0<<CS12)|(1<<CS11)|(0<<CS10);
}
void MotorLib::Config(uint16_t vLimit, uint8_t ofL, uint8_t ofR){
	_motor_vLimit = vLimit;
	_motor_ofL = ofL;
	_motor_ofR = ofR;
}
void MotorLib::Drive(int16_t vLeft, int16_t vRight){
	vLeft = vLeft*((float)_motor_ofL/100);
	vRight = vRight*((float)_motor_ofR/100);
	if(vLeft<0){
		vLeft = (vLeft*-1>_motor_vLimit?_motor_vLimit*-1:vLeft);
		MOTOR_L2_OCR = 1023 + vLeft;
	_SET(MOTOR_L1_PORT,MOTOR_L1_Bit,1);}
	else{
		vLeft = (vLeft>_motor_vLimit?_motor_vLimit:vLeft);
		MOTOR_L2_OCR = vLeft;
	_SET(MOTOR_L1_PORT,MOTOR_L1_Bit,0);}
	if(vRight<0){
		vRight = (vRight*-1>_motor_vLimit?_motor_vLimit*-1:vRight);
		MOTOR_R2_OCR = 1023 + vRight;
	_SET(MOTOR_R1_PORT,MOTOR_R1_Bit,1);}
	else{
		vRight = (vRight>_motor_vLimit?_motor_vLimit:vRight);
		MOTOR_R2_OCR = vRight;
	_SET(MOTOR_R1_PORT,MOTOR_R1_Bit,0);}
}
#endif