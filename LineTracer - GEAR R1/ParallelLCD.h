/*
Parallel LCD Library

Initialize LCD first :
lcd_name.Init();

Program Example :
lcd_name.Cursor(x,y);
lcd_name.WriteStr("TEST STRING");

- Arif Bryan
*/

#ifndef _ParallelLCD_H
#define _ParallelLCD_H

#include <util/delay.h>
#include <stdio.h>

class PLCD{
	public:
	PLCD(volatile uint8_t*, volatile uint8_t*, uint8_t);
	void Init(void);
	void Clear(void);
	void Cursor(uint8_t,uint8_t);
	void Control(bool,bool,bool);
	void Command(uint8_t);
	void WriteStr(const char*);
	void WriteChr(char);
	void WriteInt(long int);
	void CustomChr(uint8_t, uint8_t, uint8_t, uint8_t, uint8_t, uint8_t, uint8_t, uint8_t, uint8_t);
	void GraphInit();
	void GraphDisp();
	void GraphShift();
	void GraphInput(uint8_t,int);
	private:
	void Send(bool,uint8_t);
	void Send4bit(uint8_t);
	bool _plcd_backlit;
	volatile uint8_t *_plcd_en_ddr;
	volatile uint8_t *_plcd_en_port;
	uint8_t _plcd_en_bit;
	uint8_t graph_in[16];
};

#define sym_degrees 0b11011111

PLCD::PLCD(volatile uint8_t *EN_DDR, volatile uint8_t *EN_PORT, uint8_t EN_Bit){
	_plcd_en_ddr = EN_DDR;
	_plcd_en_port = EN_PORT;
	_plcd_en_bit = EN_Bit;
}

void PLCD::Send(bool cmd_chr,uint8_t data){
	_SET(PLCD_RS_PORT,PLCD_RS_Bit,cmd_chr);
	_SET(PLCD_D4_PORT,PLCD_D4_Bit,_READ(&data,4));
	_SET(PLCD_D5_PORT,PLCD_D5_Bit,_READ(&data,5));
	_SET(PLCD_D6_PORT,PLCD_D6_Bit,_READ(&data,6));
	_SET(PLCD_D7_PORT,PLCD_D7_Bit,_READ(&data,7));
	_SET(_plcd_en_port,_plcd_en_bit,1);
	_SET(_plcd_en_port,_plcd_en_bit,0);
	_SET(PLCD_D4_PORT,PLCD_D4_Bit,_READ(&data,0));
	_SET(PLCD_D5_PORT,PLCD_D5_Bit,_READ(&data,1));
	_SET(PLCD_D6_PORT,PLCD_D6_Bit,_READ(&data,2));
	_SET(PLCD_D7_PORT,PLCD_D7_Bit,_READ(&data,3));
	_SET(_plcd_en_port,_plcd_en_bit,1);
	_SET(_plcd_en_port,_plcd_en_bit,0);_delay_us(100);
}
void PLCD::Send4bit(uint8_t data){
	_SET(PLCD_RS_PORT,PLCD_RS_Bit,0);
	_SET(PLCD_D4_PORT,PLCD_D4_Bit,_READ(&data,0));
	_SET(PLCD_D5_PORT,PLCD_D5_Bit,_READ(&data,1));
	_SET(PLCD_D6_PORT,PLCD_D6_Bit,_READ(&data,2));
	_SET(PLCD_D7_PORT,PLCD_D7_Bit,_READ(&data,3));
	_SET(_plcd_en_port,_plcd_en_bit,1);
	_SET(_plcd_en_port,_plcd_en_bit,0);_delay_us(100);
}
void PLCD::Command(uint8_t cmd){
	Send(0,cmd);
}
void PLCD::WriteChr(char data){
	Send(1,data);
}
void PLCD::Clear(){
	Send(0,0b00000001);
	_delay_ms(8);
}
void PLCD::Init(){
	//Ports Init
	_SET(_plcd_en_ddr,_plcd_en_bit,1);
	_SET(PLCD_RS_DDR,PLCD_RS_Bit,1);
	_SET(PLCD_D4_DDR,PLCD_D4_Bit,1);
	_SET(PLCD_D5_DDR,PLCD_D5_Bit,1);
	_SET(PLCD_D6_DDR,PLCD_D6_Bit,1);
	_SET(PLCD_D7_DDR,PLCD_D7_Bit,1);
	//LCD Init
	_delay_ms(20);
	Send4bit(0x03);_delay_ms(6);
	Send4bit(0x03);_delay_ms(6);
	Send4bit(0x03);_delay_ms(1);
	Send4bit(0x02);_delay_ms(1);
	Send(0,0x28);_delay_ms(5);
	Send(0,0x08);_delay_ms(5);	
	Clear();_delay_ms(5);
	Send(0,0x06);_delay_ms(5);
	Send(0,0x17);_delay_ms(5);
	Send(0,0x0C);_delay_ms(5);
}
void PLCD::Cursor(uint8_t x,uint8_t y){
	x=(x>15?15:x);
	y=(y>1?1:y);
	Send(0,0b10000000+(y*64)+x);
}
void PLCD::Control(bool OnOff,bool cursor,bool blink){
	Send(0,0b00001000|(OnOff<<2)|(cursor<<1)|(blink));
}
void PLCD::WriteStr(const char* Data){
	uint8_t s=0;
	while(Data[s]!=0){
		Send(1,Data[s]);
		s++;
	}
}
void PLCD::WriteInt(long int Data){
	char buff[16];
	sprintf(buff,"%ld",Data);
	WriteStr(buff);
}
void PLCD::CustomChr(uint8_t memory,uint8_t l0,uint8_t l1,uint8_t l2,uint8_t l3,uint8_t l4,uint8_t l5,uint8_t l6,uint8_t l7){
	Send(0,0b01000000+memory*8);
	Send(1,l0);
	Send(1,l1);
	Send(1,l2);
	Send(1,l3);
	Send(1,l4);
	Send(1,l5);
	Send(1,l6);
	Send(1,l7);
}
void PLCD::GraphInit(){
	static const uint8_t cust_char[8][8]=
	{
	{	0b00000,
		0b00000,
		0b00000,
		0b00000,
		0b00000,
		0b00000,
		0b00000,
		0b11111		},
	{	0b00000,
		0b00000,
		0b00000,
		0b00000,
		0b00000,
		0b00000,
		0b11111,
		0b00000		},
	{	0b00000,
		0b00000,
		0b00000,
		0b00000,
		0b00000,
		0b11111,
		0b00000,
		0b00000		},
	{	0b00000,
		0b00000,
		0b00000,
		0b00000,
		0b11111,
		0b00000,
		0b00000,
		0b00000		},
	{	0b00000,
		0b00000,
		0b00000,
		0b11111,
		0b11111,
		0b11111,
		0b11111,
		0b11111		},
	{	0b00000,
		0b00000,
		0b11111,
		0b11111,
		0b11111,
		0b11111,
		0b11111,
		0b11111		},
	{	0b00000,
		0b11111,
		0b11111,
		0b11111,
		0b11111,
		0b11111,
		0b11111,
		0b11111		},
	{	0b11111,
		0b11111,
		0b11111,
		0b11111,
		0b11111,
		0b11111,
		0b11111,
		0b11111		}};
	for(uint8_t p=0;p<8;p++){
		for(uint8_t i=0;i<8;i++){
			Send(0,0b01000000+i+p*8);
			Send(1,cust_char[p][i]);
		}
		Cursor(0,0);
	}
}
void PLCD::GraphDisp(){
	uint8_t gdisp=0;
	for(uint8_t ptr=0;ptr<=32;ptr++){
		if(ptr<=15){
			if(graph_in[ptr]>=16){gdisp=8;}
			else if(graph_in[ptr]>=15){gdisp=7;}
			else if(graph_in[ptr]>=14){gdisp=6;}
			else if(graph_in[ptr]>=13){gdisp=5;}
			else if(graph_in[ptr]>=12){gdisp=4;}
			else if(graph_in[ptr]>=11){gdisp=3;}
			else if(graph_in[ptr]>=10){gdisp=2;}
			else if(graph_in[ptr]>=9){gdisp=1;}
			else{gdisp=0;}}
		else if(ptr>=16){
			if(graph_in[ptr-16]>=8){gdisp=8;}
			else if(graph_in[ptr-16]>=7){gdisp=7;}
			else if(graph_in[ptr-16]>=6){gdisp=6;}
			else if(graph_in[ptr-16]>=5){gdisp=5;}
			else if(graph_in[ptr-16]>=4){gdisp=4;}
			else if(graph_in[ptr-16]>=3){gdisp=3;}
			else if(graph_in[ptr-16]>=2){gdisp=2;}
			else if(graph_in[ptr-16]>=1){gdisp=1;}
			else{gdisp=0;}}
		if(ptr==0){Cursor(0,0);}
		else if(ptr==16){Cursor(1,0);}
		if(gdisp<=0){Send(1,' ');}
		else{Send(1,gdisp-1);}
	}
}
void PLCD::GraphShift(){
	graph_in[0]=graph_in[1];
	graph_in[1]=graph_in[2];
	graph_in[2]=graph_in[3];
	graph_in[3]=graph_in[4];
	graph_in[4]=graph_in[5];
	graph_in[5]=graph_in[6];
	graph_in[6]=graph_in[7];
	graph_in[7]=graph_in[8];
	graph_in[8]=graph_in[9];
	graph_in[9]=graph_in[10];
	graph_in[10]=graph_in[11];
	graph_in[11]=graph_in[12];
	graph_in[12]=graph_in[13];
	graph_in[13]=graph_in[14];
	graph_in[14]=graph_in[15];
}
void PLCD::GraphInput(uint8_t collumn,int value){
	graph_in[collumn]=value;
}
#endif