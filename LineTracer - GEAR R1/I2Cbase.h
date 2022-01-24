#ifndef _I2Cbase_H
#define _I2Cbase_H

class I2C{
	public:
	virtual void Init(bool) = 0;
	virtual bool Start(uint8_t) = 0;
	virtual uint8_t Read(bool) = 0;
	virtual bool Write(uint8_t) = 0;
	virtual void Stop(void) = 0;
};

#endif