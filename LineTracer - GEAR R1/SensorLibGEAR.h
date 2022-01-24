#ifndef _SENSORLIBGEAR_H
#define _SENSORLIBGEAR_H

class SensorLibGEAR{
	public:
		void Init(uint8_t);	
		bool Ctrl(bool);
		void Read(void);
		uint16_t ResultRaw[14];
		uint16_t ResultLow;
		uint16_t ResultHigh;
		uint16_t SensOffs[14];
		bool NewData(void);
	private:
		bool _newdata;
		bool _sensctrl;
		bool _sensctrl_done;
		uint16_t _temp_result[16];
};

#define SENSPRESC_2		0b001
#define SENSPRESC_4		0b010
#define SENSPRESC_8		0b011
#define SENSPRESC_16	0b100
#define SENSPRESC_32	0b101
#define SENSPRESC_64	0b110
#define SENSPRESC_128	0b111

void SensorLibGEAR::Init(uint8_t prescaler){
	_SET(SENSCTRL_L_DDR,SENSCTRL_L_Bit,1);
	_SET(SENSCTRL_R_DDR,SENSCTRL_R_Bit,1);
	ADMUX = (0<<REFS1)|(0<<REFS0);
	ADCSRA = (1<<ADEN)|(1<<ADSC)|(0<<ADATE)|(0<<ADIF)|(_READ(&prescaler,2)<<ADPS2)|(_READ(&prescaler,1)<<ADPS1)|(_READ(&prescaler,0)<<ADPS0);
	while(!_READ(&ADCSRA, ADIF));
	_SET(&ADCSRA, ADIF, 1);
}
bool SensorLibGEAR::Ctrl(bool enable){
	_SET(SENSCTRL_L_PORT,SENSCTRL_L_Bit,(!_sensctrl)&enable);
	_SET(SENSCTRL_R_PORT,SENSCTRL_R_Bit,(_sensctrl)&enable);
	_sensctrl_done = 1;
	return _sensctrl;
}
void SensorLibGEAR::Read(){
	if(_sensctrl_done){
		for(uint8_t idx = 1;idx < 8;idx ++){
			ADMUX = (0b00000111 & idx);
			_delay_us(10);
			_SET(&ADCSRA, ADSC, 1);
			while(!_READ(&ADCSRA, ADIF));
			_SET(&ADCSRA, ADIF, 1);
			//_temp_result[(idx-1)+(_sensctrl*7)] = (ADCL) | (ADCH<<8);
			_temp_result[(idx-1)+(_sensctrl*7)] = ADCW;
		}
		
		if(_sensctrl){
			ResultHigh = 0;
			ResultLow = 1023;
			_newdata = 1;
			for(uint8_t i=0;i<7;i++){ResultRaw[i] = (_temp_result[i] < SensOffs[i]?0:_temp_result[i] - SensOffs[i]);}
			for(uint8_t i=7;i<14;i++){ResultRaw[i] = (_temp_result[20-i] < SensOffs[i]?0:_temp_result[20-i] - SensOffs[i]);}
			for(uint8_t i=0;i<14;i++){
				if(ResultRaw[i]>ResultHigh){ResultHigh = ResultRaw[i];}
				if(ResultRaw[i]<ResultLow){ResultLow = ResultRaw[i];}
			}
		}
		_sensctrl = !_sensctrl;
		_sensctrl_done = 0;
	}
}
bool SensorLibGEAR::NewData(){
	bool ret = _newdata;
	_newdata = 0;
	return ret;
}
#endif