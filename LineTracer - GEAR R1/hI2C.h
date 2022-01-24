/*
Hardware I2C bus driver library

Using I2C base class in I2Cbase.h

Functions :

//Declare object first
hI2C I2C;

//Initialize bus
I2C.Init( [ I2C_SPEED_NORMAL / I2C_SPEED_HIGH ] );

//Start
I2C.Start( [ Slave dev. address ] );

//Transmit data
[ ACK ] = I2C.Write( [ Data ] );

//Receive data
[ Data ] = I2C.Read( [ ACK ] );

//Stop
I2C.Stop();

- Arif Bryan
--03/06/2019--
*/

#ifndef _HI2C_H
#define _HI2C_H

#include "bytecon.h"
#include "I2Cbase.h"
#include <compat/twi.h>

class hI2C : public I2C{
	public:
		void Init(bool);
		bool Start(uint8_t);
		uint8_t Read(bool);
		bool Write(uint8_t);
		void Stop();	
};

#define hI2C_SPEED_HIGH		1
#define hI2C_SPEED_NORMAL	0

void hI2C::Init(bool Speed){
	TWBR = (F_CPU / (Speed?400000UL:100000UL) - 16) / 2 ;
	TWSR = 0;
}

bool hI2C::Start(uint8_t Addr){
	uint8_t twst = 0;
	TWCR = (1<<TWINT)|(1<<TWSTA)|(1<<TWEN);
	while(!(TWCR & (1<<TWINT)));
	twst = TW_STATUS & 0xF8;
	if ((twst != TW_START) && (twst != TW_REP_START)){return 0;}
	else{
		TWDR = Addr;
		TWCR = (1<<TWINT)|(1<<TWEN);
		while(!(TWCR & (1<<TWINT)));
		twst = TW_STATUS & 0xF8;
		if ((twst != TW_MT_SLA_ACK) && (twst != TW_MR_SLA_ACK)){return 0;}
		else{return 1;}
	}
}

uint8_t hI2C::Read(bool ACK){
	TWCR = (1<<TWINT)|(1<<TWEN)|(ACK<<TWEA);
	while(!(TWCR & (1<<TWINT)));
	return TWDR;
}

bool hI2C::Write(uint8_t Data){
	uint8_t twst = 0;
	TWDR = Data;
	TWCR = (1<<TWINT)|(1<<TWEN);
	while(!(TWCR & (1<<TWINT)));
	twst = TW_STATUS & 0xF8;
	if(twst != TW_MT_DATA_ACK){return 0;}
	else{return 1;}
}

void hI2C::Stop(){
	TWCR = (1<<TWINT)|(1<<TWEN)|(1<<TWSTO);
}

#endif