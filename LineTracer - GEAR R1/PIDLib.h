#ifndef _PIDLIB_H
#define _PIDLIB_H

class PIDLib{
	public:
		void Calculate(int, uint8_t, uint8_t, uint8_t);
		
	private:
};

void PIDLib::Calculate(int Error, uint8_t Kp, uint8_t Ki, uint8_t Kd){
	term_p = Error;
	term_i = Error + _l_error;
	term_d = Error - _l_error;
	Result = (double)(term_p*Kp) + (double)(term_d*Kd) + (double)(term_i*Ki/10);
	_l_error = Error;
}
#endif