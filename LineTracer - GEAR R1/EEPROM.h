#ifndef _EEPROM_H
#define _EEPROM_H

#include <avr/eeprom.h>

#define BYTE 1

class EEPROM{
	public:
	void Init(uint8_t,uint8_t);
	void WriteByte(uint8_t,uint8_t);
	void WriteArray(uint8_t,uint8_t*);
	uint8_t ReadByte(uint8_t);
	void ReadArray(uint8_t,uint8_t*);
}EEPROM;

uint8_t eeprom_data_addr[255],eeprom_data_len[255],eeprom_len=0;
void EEPROM::Init(uint8_t var,uint8_t len){
	eeprom_data_addr[var]=eeprom_len;
	eeprom_data_len[var]=len;
	eeprom_len=eeprom_len+len;
}
void EEPROM::WriteByte(uint8_t var,uint8_t source){
	eeprom_write_byte(&eeprom_data_addr[var],source);
}
void EEPROM::WriteArray(uint8_t var,uint8_t source[]){
	uint8_t sel,point=0;
	for(sel=1;sel<=eeprom_data_len[var];sel++){
		eeprom_write_byte(&eeprom_data_addr[var]+point,source[sel-1]);
		point++;
	}
}
uint8_t EEPROM::ReadByte(uint8_t var){
	return eeprom_read_byte(&eeprom_data_addr[var]);
}
void EEPROM::ReadArray(uint8_t var,uint8_t output[]){
	uint8_t sel,point=0;
	for(sel=1;sel<=eeprom_data_len[var];sel++){
		output[sel-1]=eeprom_read_byte(&eeprom_data_addr[var]+point);
		point++;
	}
}
#endif