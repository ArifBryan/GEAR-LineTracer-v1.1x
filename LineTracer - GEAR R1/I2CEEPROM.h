#ifndef _I2CEEPROM_H
#define _I2CEEPROM_H

class I2CEEPROM{
	public:
		I2CEEPROM(uint8_t, I2C*);
		bool WriteByte(uint16_t, uint8_t);
		uint8_t ReadByte(uint16_t);
		bool UpdateByte(uint16_t, uint8_t);
		bool WriteWord(uint16_t, uint16_t);
		uint16_t ReadWord(uint16_t);
		bool UpdateWord(uint16_t, uint16_t);
		bool WriteBlock(uint16_t, uint8_t[], uint16_t);
		void ReadBlock(uint16_t, uint8_t[], uint16_t);
		bool UpdateBlock(uint16_t, uint8_t[], uint16_t);
		uint16_t AllocByte(void);
		uint16_t AllocWord(void);
		uint16_t AllocBlock(uint16_t);
		bool Check(void);
	private:
		uint16_t _i2ceeprom_alloc_address;
		uint8_t _i2ceeprom_i2caddress;
		I2C *_i2ceeprom_iic;
};

I2CEEPROM::I2CEEPROM(uint8_t DevAddr, I2C *Device){
	_i2ceeprom_i2caddress = DevAddr<<1;
	_i2ceeprom_iic = Device;
	_i2ceeprom_alloc_address = 0;
}

bool I2CEEPROM::WriteByte(uint16_t Addr, uint8_t Data){
	bool ack=1;
	uint8_t addrh=Addr>>8;
	uint8_t addrl=(uint8_t)Addr&0b0000000011111111;
	_i2ceeprom_iic->Start(_i2ceeprom_i2caddress|0);
	ack&=_i2ceeprom_iic->Write(addrh);
	ack&=_i2ceeprom_iic->Write(addrl);
	ack&=_i2ceeprom_iic->Write(Data);
	_i2ceeprom_iic->Stop();
	if(ack){
		bool done=0;
		while(!done){
			done = _i2ceeprom_iic->Start(_i2ceeprom_i2caddress|0);
			_i2ceeprom_iic->Stop();
		}
	}
	return ack;
}
uint8_t I2CEEPROM::ReadByte(uint16_t Addr){
	uint8_t data=0;
	uint8_t addrh=Addr>>8;
	uint8_t addrl=Addr&0b0000000011111111;
	_i2ceeprom_iic->Start(_i2ceeprom_i2caddress|0);
	_i2ceeprom_iic->Write(addrh);
	_i2ceeprom_iic->Write(addrl);
	if(_i2ceeprom_iic->Start(_i2ceeprom_i2caddress|1)){
		data=_i2ceeprom_iic->Read(0);}
	_i2ceeprom_iic->Stop();
	return data;
}
bool I2CEEPROM::UpdateByte(uint16_t Addr, uint8_t Data){
	bool ack=0;
	if(ReadByte(Addr)!=Data){
		ack = WriteByte(Addr,Data);
	}
	return ack;
}

uint16_t I2CEEPROM::ReadWord(uint16_t Addr){
	uint16_t data=0;
	data = ReadByte(Addr)<<8;
	data |= ReadByte(Addr+1);
	return data;
}
bool I2CEEPROM::WriteWord(uint16_t Addr, uint16_t Data){
	bool ack=1;
	uint8_t datah=Data>>8;
	uint8_t datal=Data&0b0000000011111111;
	ack &= WriteByte(Addr, datah);
	ack &= WriteByte(Addr+1, datal);
	return ack;
}
bool I2CEEPROM::UpdateWord(uint16_t Addr, uint16_t Data){
	bool ack=0;
	if(ReadWord(Addr)!=Data){
		ack = WriteWord(Addr,Data);
	}
	return ack;
}
bool I2CEEPROM::WriteBlock(uint16_t Addr, uint8_t Data[], uint16_t n){
	bool ack=1;
	for(uint16_t p=0;p<n;p++){
		ack &= WriteByte(Addr+p,Data[p]);
	}
	return ack;
}
void I2CEEPROM::ReadBlock(uint16_t Addr, uint8_t Data[], uint16_t n){
	for(uint16_t p=0;p<n;p++){
		Data[p] = ReadByte(Addr+p);
	}
}
bool I2CEEPROM::UpdateBlock(uint16_t Addr, uint8_t Data[], uint16_t n){
	bool ack=1;
	for(uint16_t p=0;p<n;p++){
		ack &= UpdateByte(Addr+p,Data[p]);
	}
	return ack;
}
uint16_t I2CEEPROM::AllocByte(){
	uint16_t tmp=_i2ceeprom_alloc_address;
	_i2ceeprom_alloc_address += 1;
	return tmp;
}
uint16_t I2CEEPROM::AllocWord(){
	uint16_t tmp=_i2ceeprom_alloc_address;
	_i2ceeprom_alloc_address += 2;
	return tmp;
}
uint16_t I2CEEPROM::AllocBlock(uint16_t n){
	uint16_t tmp=_i2ceeprom_alloc_address;
	_i2ceeprom_alloc_address += n;
	return tmp;
}
bool I2CEEPROM::Check(){
	bool ack=0;
	ack = _i2ceeprom_iic->Start(_i2ceeprom_i2caddress|0);
	_i2ceeprom_iic->Stop();
	return ack;
}
#endif