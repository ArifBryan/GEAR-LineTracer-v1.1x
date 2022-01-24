/*
 * LineTracer - GEAR R1.cpp
 *
 * Created: 27/09/2019 23:41:23
 * Author : Arif Bryan
 */ 
#define VERSION		"v1.88ag"
#define VBAT_MIN_LIMIT	11800
//#define DBG		//Uncomment for debug mode

#define F_CPU 16000000UL

#define PLCD_RS_PORT	&PORTB
#define PLCD_RS_DDR		&DDRB
#define PLCD_RS_Bit		0
#define PLCD_EN_PORT	&PORTB
#define PLCD_EN_DDR		&DDRB
#define PLCD_EN_Bit		1
#define PLCD_D4_PORT	&PORTB
#define PLCD_D4_DDR		&DDRB
#define PLCD_D4_Bit		3
#define PLCD_D5_PORT	&PORTB
#define PLCD_D5_DDR		&DDRB
#define PLCD_D5_Bit		4
#define PLCD_D6_PORT	&PORTB
#define PLCD_D6_DDR		&DDRB
#define PLCD_D6_Bit		5
#define PLCD_D7_PORT	&PORTB
#define PLCD_D7_DDR		&DDRB
#define PLCD_D7_Bit		7
#define LCDLED_DDR		&DDRD
#define LCDLED_PORT		&PORTD
#define LCDLED_Bit		2

#define SLED_DDR		&DDRD
#define SLED_PORT		&PORTD
#define SLED_Bit		7

#define MOTOR_L1_DDR	&DDRD
#define MOTOR_L1_PORT	&PORTD
#define MOTOR_L1_Bit	3
#define MOTOR_L2_DDR	&DDRD
#define MOTOR_L2_Bit	4
#define MOTOR_L2_OCR	OCR1B
#define MOTOR_R1_DDR	&DDRD
#define MOTOR_R1_PORT	&PORTD
#define MOTOR_R1_Bit	6
#define MOTOR_R2_DDR	&DDRD
#define MOTOR_R2_Bit	5
#define MOTOR_R2_OCR	OCR1A

#define SENSCTRL_L_DDR	&DDRC
#define SENSCTRL_L_PORT	&PORTC
#define SENSCTRL_L_Bit	7
#define SENSCTRL_R_DDR	&DDRC
#define SENSCTRL_R_PORT	&PORTC
#define SENSCTRL_R_Bit	6

#define BTNX1_DDR		&DDRB
#define BTNX1_PIN		&PINB
#define BTNX1_PORT		&PORTB
#define BTNX1_Bit		6
#define BTNX2_DDR		&DDRC
#define BTNX2_PIN		&PINC
#define BTNX2_PORT		&PORTC
#define BTNX2_Bit		3
#define BTNY1_DDR		&DDRC
#define BTNY1_PIN		&PINC
#define BTNY1_PORT		&PORTC
#define BTNY1_Bit		4
#define BTNY2_DDR		&DDRC
#define BTNY2_PIN		&PINC
#define BTNY2_PORT		&PORTC
#define BTNY2_Bit		2
#define BTNY3_DDR		&DDRC
#define BTNY3_PIN		&PINC
#define BTNY3_PORT		&PORTC
#define BTNY3_Bit		5

#define sI2C_SDA_DDR	&DDRC
#define sI2C_SDA_PIN	&PINC
#define sI2C_SDA_PORT	&PORTC
#define sI2C_SDA_Bit	1
#define sI2C_SCL_DDR	&DDRC
#define sI2C_SCL_PIN	&PINC
#define sI2C_SCL_PORT	&PORTC
#define sI2C_SCL_Bit	0

#define IO1_DDR			&DDRB
#define IO1_PIN			&PINB
#define IO1_PORT		&PORTB
#define IO1_Bit			2
#define IO2_DDR			&DDRD
#define IO2_PIN			&PIND
#define IO2_PORT		&PORTD
#define IO2_Bit			0
#define IO3_DDR			&DDRD
#define IO3_PIN			&PIND
#define IO3_PORT		&PORTD
#define IO3_Bit			1


#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <avr/eeprom.h>
#include <util/delay.h>
#include <stdlib.h>
#include "bytecon.h"
#include "MotorLib.h"
#include "ParallelLCD.h"
#include "SensorLibGEAR.h"
#include "LineDetectorLib.h"
#include "TimerLib.h"
#include "hI2C.h"
#include "I2CEEPROM.h"
#include "SysConfig.h"

volatile uint8_t tbase = 0;
ISR(TIMER0_COMP_vect){
	tbase++;
}

PLCD LCD(PLCD_EN_DDR, PLCD_EN_PORT, PLCD_EN_Bit);
SensorLibGEAR Sens;
MotorLib Motor;
LineDetectorLib Line;
hI2C I2C;
I2CEEPROM NVM(0b1010000, &I2C);

static void arrcpy(uint8_t dst[], const uint8_t src[], uint8_t n){
	while(n > 0){
		dst[n-1] = src[n-1];
		n --; 
	}
} 
static void BarGraph(uint8_t val){
	val = (val>8?8:val);
	if(val>0){LCD.WriteChr(0b00000111 & (val-1));}
	else{LCD.WriteChr(' ');}
}
static void sensDisp(uint16_t sensBit){
	for(uint8_t i = 0;i < 14;i ++){
		LCD.Cursor(i+1,0);
		if((sensBit>>14) < 2){LCD.WriteChr((_READ(&sensBit, 13-i)?7:0));}
		else{LCD.WriteChr((_READ(&sensBit, 13-i)?4:3));}
	}
	switch(sensBit>>14){
		case 0:LCD.WriteChr('&');break;
		case 1:LCD.WriteChr('|');break;
		case 2:LCD.WriteChr('&');break;
		case 3:LCD.WriteChr('|');break;
	}
}

#define _lim(n,l,h)		(n<l?l:(n>h?h:n))
#define _loop(n,l,h)	(n<l?h:(n>h?l:n))

static void setMaskByte(uint8_t *out, uint8_t mask, uint8_t in){
	for(uint8_t i = 0;i < 8;i ++){
		if(_READ(&mask, i)){_SET(out, i, _READ(&in, i));}
	}
}

//Internal EEPROM
static uint16_t EEMEM sensMask_eep[31];
static uint8_t EEMEM progIdxStop_eep[10];
static uint8_t EEMEM progCheckPointSel_eep[10];
static uint8_t EEMEM checkPoint_eep[10][21];
static uint8_t EEMEM progStartIdx_eep[10][2];
static uint8_t EEMEM PDM_eep[10][3];
static uint8_t EEMEM progSel_eep;
static uint8_t EEMEM scanVel_eep;
static uint8_t EEMEM scanPDM_eep[3];
static uint8_t EEMEM prescT_eep;
static uint8_t EEMEM prescD_eep;
static uint8_t EEMEM mtrOffsL_eep;
static uint8_t EEMEM mtrOffsR_eep;
static uint8_t EEMEM vVbatComp_eep;
static uint8_t EEMEM runDly_eep;
static uint8_t EEMEM toutTime_eep;
static uint8_t EEMEM sysConfigs_eep;
static uint16_t EEMEM sensHi_eep;
static uint16_t EEMEM sensLo_eep;
static uint16_t EEMEM sensLLo_eep;
static uint16_t EEMEM sensLHi_eep;
static uint8_t EEMEM brakeTime_eep;
static uint8_t EEMEM first_reset_eep;
static uint16_t EEMEM sensOffs_eep[14];

//Constants
static const char aboutDisp[][17] PROGMEM = {" POLINEMA 2019  ","   WSEC-23 AB   "};
static const char menuStr[][17] PROGMEM = {
	" Program",
	" Stop at i",
	" CheckPoint",
	" Set PD ",
	" Sensor Config",
	" Motor Offset",
	" SelfTest",
	" Prescalers",
	" Vbat ",
	" Settings"};
static const char lineColStr[] PROGMEM = {'A','B','W'};
static const char idxCmdStr[][6] PROGMEM = {
	"Run  ",
	"Left ",
	"Right",
	"Fwrd ",
	"Revrs",
	"Stop ",
	"Servo",
	"GoTo ",
	"End  "};
static const char settStr[][14] PROGMEM = {
	" Run Dly ",
	" Timeout ",
	" End Brake ",
	" Vbat Comp ",
	" Diagnostics"};
static const char pidModeStr[][4] PROGMEM = {"RDC","DIF"};
static const char lineModeStr[][6] PROGMEM = {"Nline","Cline","Sline","Lline","Rline"};
static const char offOnStr[][4] PROGMEM = {"OFF","ON "};
static const char invNrmStr[][4] PROGMEM = {"Inv","Nrm"};
static const char stopRunStr[][5] PROGMEM = {"STOP","RUN "};
static const char progMenuStr[] PROGMEM = "Prog Menu";
static const char idxMenuStr[] PROGMEM = "Index Menu";
static const char editSensStr[] PROGMEM = "Edit Sensor";
static const char startIdxStr[] PROGMEM = "   Start Index  ";
static const char dlySensStr[][3] PROGMEM = {"--","L7","L6","L5","L4","L3","L2","L1","R1","R2","R3","R4","R5","R6","R7"};
static const uint16_t sensPrst[] PROGMEM = {
	0b000000011000000,
	0b100000111100000,
	0b110000000000000,
	0b111000000000000,
	0b111100000000000,
	0b100000000000001,
	0b100000000000011,
	0b100000000000111,
	0b000000111100000,
	0b000011110000000,
	0b000000001111000};
static const uint8_t indexDefVal[8] = {0,0,0,0,0,0,55,2};
//Unsigned 8-bit
static uint8_t menu = 0;
static uint8_t submenu = 0;
static uint8_t sidemenu = 0;
static uint8_t presc_1ms;
static uint8_t pbTimer;
static uint8_t pb[6];
static uint8_t pbLast;
static uint8_t dspCursor;
static uint8_t gpVar1;
static uint8_t progSel;
static uint8_t idxSel = 0;
static uint8_t idxStop;
static uint8_t startIdx[2];
static uint8_t indexBuff[8];
static uint8_t checkPoint[21];
static uint8_t checkPointSel;
static uint8_t PDMsel;
static uint8_t PDMnow[3];
static uint8_t scanPDM[3];
static uint8_t scanVel;
static uint8_t mtrOffsL = 100;
static uint8_t mtrOffsR = 100;
static uint8_t runDly;
static uint8_t toutTime;
static uint8_t vVbatComp;
static uint8_t prescT = 20;
static uint8_t prescD = 10;
static uint8_t sysConfigs;
static uint8_t presc_Dms;
static uint8_t presc_Tms;
static uint8_t Ttmr;
static uint8_t Dtmr;
static uint8_t idxPtr;
static uint8_t progPtr;
static uint8_t toutTmr;
static uint8_t runDtmr;
static uint8_t seq;
static uint8_t errTmr;
static uint8_t sampleTmr;
static uint8_t gpVar2;
static uint8_t presc_10ms;
static uint8_t presc_100ms;
static uint8_t brakeTime;
static uint8_t brakeTmr;
static uint8_t brake;
static uint8_t cal;
static uint8_t servoTmr;
static uint8_t servoPos[3];
//Signed 8-bit
static int8_t lastError;
//Signed 16-bit
static int16_t mtrVelLeft;
static int16_t mtrVelRight;
static int16_t PDresult;
//Unsigned 16-bit
static uint16_t sensMask;
static uint16_t progMem_eep[10];
static uint16_t runTmr;
static uint16_t runTotalTime;
static uint16_t sensHi;
static uint16_t sensLo;
static uint16_t vbat;
static uint16_t runVel;
//Character & string
static char dspCursorChr = '>';
static char dspBuff[2][20];
static char strBuff[20];
//Boolean
static bool readRunMem;
static bool sensNewData;
static bool tout;
static bool runDisp;
static bool useATC;
static bool pointer;
static bool pbPress;
static bool dspRfs = 1;
static bool sample;
static bool testPD;
static bool uppDsp = 1;
static bool progInv;
static bool sysRun;
static bool lowBatt;
static bool sensOff;

static void presetIdx(){
	switch(indexBuff[0]&0b00001111){
		//Run
		case 0:
			setMaskByte(&indexBuff[0], 0b11110000, 0);
			indexBuff[2] = 0;
			indexBuff[3] = 0;
			indexBuff[4] = 0;		
		break;
		//Left
		case 1:
			setMaskByte(&indexBuff[0], 0b11110000, 5<<4);
			indexBuff[2] = 50/prescD;
			indexBuff[3] = 60;
			indexBuff[4] = -55;
		break;
		//Right
		case 2:
			setMaskByte(&indexBuff[0], 0b11110000, 10<<4);
			indexBuff[2] = 50/prescD;
			indexBuff[3] = -55;
			indexBuff[4] = 60;
			indexBuff[6] = 55;
		break;
		//Fwrd
		case 3:
			setMaskByte(&indexBuff[0], 0b11110000, 0);
			indexBuff[2] = 160/prescD;
			indexBuff[3] = 60;
			indexBuff[4] = 60;
		break;
		//Revrs
		case 4:
			indexBuff[2] = 160/prescD;
			indexBuff[3] = -60;
			indexBuff[4] = -60;
		break;
		//Stop
		case 5:
			indexBuff[2] = 400/prescD;
			indexBuff[3] = 0;
			indexBuff[4] = 0;
			indexBuff[5] = 0;
			indexBuff[6] = 0;
			indexBuff[7] = 4;
		break;
		//Servo
		case 6:
			indexBuff[2] = 100;
			indexBuff[3] = 100;
			indexBuff[4] = 100;
			indexBuff[5] = 100;
			indexBuff[6] = 100;
			indexBuff[7] = 100;
		break;
		//GoTo
		case 7:
			indexBuff[2] = 0;
			indexBuff[3] = 0;
		break;
		//End
		case 8:
		break;
	}
}
static void readProgMem(uint8_t prog){
	if(idxSel>0){NVM.ReadBlock(progMem_eep[prog]+((idxSel-1)*8), indexBuff, 8);}		
}
static void updateProgMem(uint8_t prog){
	if(idxSel>0){NVM.UpdateBlock(progMem_eep[prog]+((idxSel-1)*8), indexBuff, 8);}
}

#define readPDMmem(x)		eeprom_read_block(PDMnow, PDM_eep[x], 3)
#define readCPselMem()		checkPointSel = eeprom_read_byte(&progCheckPointSel_eep[progSel])
#define readCheckPoint()	eeprom_read_block(checkPoint, checkPoint_eep[progSel], 21)
#define readStopIdx()		idxStop = eeprom_read_byte(&progIdxStop_eep[progSel])

static uint16_t readVbat(){
	uint16_t ret=0;	
	ADMUX = (0b00000111 & 0);
	_delay_us(10);
	_SET(&ADCSRA, ADSC, 1);
	while(!_READ(&ADCSRA, ADIF));
	_SET(&ADCSRA, ADIF, 1);
	//ret = (ADCL)|(ADCH<<8);
	ret = ADCW;
	ret = ret*4.887/0.3197+250;
	return ret;
}
static void readSensOffs(){
	for(gpVar2 = 0;gpVar2 < 14;gpVar2 ++){
		Sens.SensOffs[gpVar2] = eeprom_read_word(&sensOffs_eep[gpVar2]);
	}
}
static void readSensMem(){
	uint16_t tmp = eeprom_read_word(&sensMask_eep[(indexBuff[1]&0b0011111)]);
	if(progInv){
		for(gpVar2 = 0;gpVar2 < 14;gpVar2 ++){
			_SET(&sensMask, gpVar2, _READ(&tmp, 13 - gpVar2));
		}
		_SET(&sensMask, 14, _READ(&tmp, 14));
		_SET(&sensMask, 15, _READ(&tmp, 15));
	}
	else{sensMask = tmp;}
}
static void readStartIdx(void){
	eeprom_read_block(startIdx, progStartIdx_eep[progSel], 2);
	progInv = _READ(&startIdx[1], 7);
	_SET(&startIdx[1], 7, 0);
	if(idxSel == 0){
		indexBuff[5] = startIdx[0];
		indexBuff[6] = startIdx[1];
	}
}
static void readSensCfg(){
	if(useATC){
		sensLo = eeprom_read_word(&sensLLo_eep);
		sensHi = eeprom_read_word(&sensLHi_eep);
	}
	else{
		sensLo = eeprom_read_word(&sensLo_eep);
		sensHi = eeprom_read_word(&sensHi_eep);
	}
}
static void readConfMem(void){
	eeprom_read_block(scanPDM, scanPDM_eep, 3);
	progSel = eeprom_read_byte(&progSel_eep);
	scanVel = eeprom_read_byte(&scanVel_eep);
	prescT = eeprom_read_byte(&prescT_eep);
	prescD = eeprom_read_byte(&prescD_eep);
	mtrOffsL = eeprom_read_byte(&mtrOffsL_eep);
	mtrOffsR = eeprom_read_byte(&mtrOffsR_eep);
	vVbatComp = eeprom_read_byte(&vVbatComp_eep);
	runDly = eeprom_read_byte(&runDly_eep);
	toutTime = eeprom_read_byte(&toutTime_eep);
	sysConfigs = eeprom_read_byte(&sysConfigs_eep);
	brakeTime = eeprom_read_byte(&brakeTime_eep);
	useATC = _READ(&sysConfigs, 1);
}

#define updatePDMmem(x)		eeprom_update_block(PDMnow, PDM_eep[x], 3)
#define updateCPselMem()	eeprom_update_byte(&progCheckPointSel_eep[progSel], checkPointSel)
#define updateCheckPoint()	eeprom_update_block(checkPoint, checkPoint_eep[progSel], 21)
#define updateStopIdx()		eeprom_update_byte(&progIdxStop_eep[progSel], idxStop)

static void updateSensOffs(){
	for(gpVar2 = 0;gpVar2 < 14;gpVar2 ++){
		eeprom_update_word(&sensOffs_eep[gpVar2], Sens.SensOffs[gpVar2]);
	}
}
static void updateSensMem(){
	uint16_t tmp = 0;
	if(progInv){
		for(gpVar2 = 0;gpVar2 < 14;gpVar2 ++){
			_SET(&tmp, gpVar2, _READ(&sensMask, 13 - gpVar2));
		}
		_SET(&tmp, 14, _READ(&sensMask, 14));
		_SET(&tmp, 15, _READ(&sensMask, 15));
	}
	else{tmp = sensMask;}
	eeprom_update_word(&sensMask_eep[(indexBuff[1]&0b0011111)], tmp);
}
static void updateStartIdx(void){
	if(idxSel == 0){
		startIdx[0] = indexBuff[5];
		startIdx[1] = indexBuff[6];
	}
	_SET(&startIdx[1], 7, progInv);
	eeprom_update_block(startIdx, progStartIdx_eep[progSel], 2);
}
static void updateSensCfg(){
	if(useATC){
		eeprom_update_word(&sensLLo_eep, sensLo);
		eeprom_update_word(&sensLHi_eep, sensHi);
	}
	else{
		eeprom_update_word(&sensLo_eep, sensLo);
		eeprom_update_word(&sensHi_eep, sensHi);
	}
}
static void updateConfMem(void){
	eeprom_update_block(scanPDM, scanPDM_eep, 3);
	eeprom_update_byte(&progSel_eep, progSel);
	eeprom_update_byte(&scanVel_eep, scanVel);
	eeprom_update_byte(&prescT_eep, prescT);
	eeprom_update_byte(&prescD_eep, prescD);
	eeprom_update_byte(&mtrOffsL_eep, mtrOffsL);
	eeprom_update_byte(&mtrOffsR_eep, mtrOffsR);
	eeprom_update_byte(&vVbatComp_eep, vVbatComp);
	eeprom_update_byte(&runDly_eep, runDly);
	eeprom_update_byte(&toutTime_eep, toutTime);
	eeprom_update_byte(&brakeTime_eep, brakeTime);
	sysConfigs = (useATC<<1);
	eeprom_update_byte(&sysConfigs_eep, sysConfigs);
}
static void copyProg(uint8_t dst){
	uint8_t srcProg = progSel;
	uint8_t srcIdx = idxSel;
	for(idxSel = 1;idxSel <= 100;idxSel ++){
		readProgMem(srcProg);
		updateProgMem(dst);
	}
	idxSel = srcIdx;
	readCheckPoint();
	readCPselMem();
	readStartIdx();
	readStopIdx();
	progSel = dst;
	updateCheckPoint();
	updateCPselMem();
	updateStartIdx();
	updateStopIdx();
	progSel = srcProg;
}
static void invProg(){
	uint8_t srcIdx = idxSel;
	for(idxSel = 1;idxSel <= 100;idxSel ++){
		readProgMem(progSel);
		if((indexBuff[0]&0b00001111) <= 4){
			switch(indexBuff[0]&0b00001111){
				case 1:gpVar1 = 2;break;
				case 2:gpVar1 = 1;break;
				default:gpVar1 = indexBuff[0]&0b00001111;break;
			}
			setMaskByte(&indexBuff[0], 0b00001111, gpVar1);
			switch((indexBuff[0]>>4)){
				case 1:gpVar1 = 14;break;
				case 2:gpVar1 = 13;break;
				case 3:gpVar1 = 12;break;
				case 4:gpVar1 = 11;break;
				case 5:gpVar1 = 10;break;
				case 6:gpVar1 = 9;break;
				case 7:gpVar1 = 8;break;
				case 8:gpVar1 = 7;break;
				case 9:gpVar1 = 6;break;
				case 10:gpVar1 = 5;break;
				case 11:gpVar1 = 4;break;
				case 12:gpVar1 = 3;break;
				case 13:gpVar1 = 2;break;
				case 14:gpVar1 = 1;break;
				default:gpVar1 = (indexBuff[0]&0b11110000)>>4;break; 
			}
			setMaskByte(&indexBuff[0], 0b11110000, gpVar1<<4);
			switch((indexBuff[7]>>4)){
				case 3:gpVar1 = 4;break;
				case 4:gpVar1 = 3;break;
				default:gpVar1 = (indexBuff[7]&0b11110000)>>4;break;
			}
			setMaskByte(&indexBuff[7], 0b11110000, gpVar1<<4);
			gpVar1 = indexBuff[3];
			indexBuff[3] = indexBuff[4];
			indexBuff[4] = gpVar1;
		}
		updateProgMem(progSel);
	}
	readStartIdx();
	progInv = !progInv;
	updateStartIdx();
	idxSel = srcIdx;
}
static void deleteProg(){
	uint8_t srcIdx = idxSel;	
	for(gpVar1 = 0;gpVar1 < 21;gpVar1 ++){
		checkPoint[gpVar1] = 0;
	}
	checkPointSel = 0;
	idxSel = 0;
	arrcpy(indexBuff, indexDefVal, 8);
	idxStop = 0;
	updateCheckPoint();
	updateCPselMem();
	progInv = 0;
	updateStartIdx();
	updateStopIdx();
	for(idxSel = 1;idxSel <= 100;idxSel ++){
		updateProgMem(progSel);
	}
	idxSel = srcIdx;
}
static void shortDisp_P(uint8_t pos, const char *Pchar){
	LCD.Clear();
	strcpy_P(dspBuff[0], Pchar);
	LCD.Cursor(3,0);LCD.WriteStr(dspBuff[0]);
	_delay_ms(500);	LCD.Clear();
}

static void Prescaler(void);
static void RunTime(void);
static void Sampling(void);
static void Buttons(void);
static void Interface(void);
static void Storage(void);

int main(void){
	//Init
	SLED(1);
	SysInit();
	I2C.Init(hI2C_SPEED_HIGH);
	TimerInit(TIMERPRESC_64,25);
	Motor.Init();
	Sens.Init(SENSPRESC_128);
	LCD.Init();
	LCD.GraphInit();
	BCKLT(1);
	//Allocate
	for(uint8_t i = 0;i < 10;i ++){
		progMem_eep[i] = NVM.AllocBlock(8*100);
	}
	if(!NVM.Check()){
		LCD.Cursor(2,0);
		LCD.WriteStr("EEPROM error");
		while(1);
	}
	//Reset Menu
	BTNCOM(0);BTNCOM(0);
	pb[0] = BTN1;
	pb[1] = BTN2;
	pb[2] = BTN3;
	BTNCOM(1);BTNCOM(1);
	pb[3] = BTN4;
	pb[4] = BTN5;
	pb[5] = BTN6;
	bool first_reset = (eeprom_read_byte(&first_reset_eep) != 1);
	if((pb[2] && pb[3]) || first_reset){
		LCD.Cursor(2,0);
		LCD.WriteStr(" Reset Menu ");
		_delay_ms(1000);
		while(BTN3 | BTN4);
		LCD.Clear();
		uint8_t reset_mode=1;
		bool reset_enter=0;
		bool reset_abort=0;
		if(first_reset){reset_mode=4;}
		reset_prg:
		LCD.Clear();
		LCD.Cursor(3,1);
		LCD.WriteStr("Reset");
		if(reset_mode>=1&&reset_mode<=4){
			LCD.Cursor(4,0);
		LCD.WriteStr("N      Y");}
		if(reset_mode==1){
			LCD.Cursor(9,1);
		LCD.WriteStr("Conf?");}
		else if(reset_mode==2){
			LCD.Cursor(9,1);
		LCD.WriteStr("PD?  ");}
		else if(reset_mode==3){
			LCD.Cursor(9,1);
		LCD.WriteStr("Prog?");}
		else if(reset_mode==4){
			LCD.Cursor(9,1);
		LCD.WriteStr("All? ");}
		reset_confirm:
		if(reset_enter){
			LCD.Clear();
			LCD.Cursor(4,0);
			LCD.WriteStr("N      Y");
			LCD.Cursor(4,1);
			LCD.WriteStr("CONFIRM");
		}
		while(BTN3||BTN4||BTN5||BTN6);
		_delay_ms(20);
		while(1){
			if(!reset_enter){
				BTNCOM(0);_delay_us(10);
				if(BTN2&&reset_mode<4){
					reset_mode++;
				goto reset_prg;}
				if(BTN1&&reset_mode>1){
					reset_mode--;
				goto reset_prg;}
				BTNCOM(1);_delay_us(10);
				if(BTN4){
					if(reset_mode>0){reset_enter=1;}
					else{reset_abort=1;}
				goto reset_confirm;}
			}
			BTNCOM(0);_delay_us(10);
			if(BTN3||reset_abort){
				LCD.Clear();
				LCD.Cursor(5,0);
				LCD.WriteStr("CANCEL");
				_delay_ms(300);
				LCD.Clear();
			break;}
			else if(reset_enter){
				BTNCOM(1);_delay_us(10);
				if(BTN4){
					LCD.Clear();
					LCD.Cursor(3,0);
					LCD.WriteStr("Erasing...");
					if(reset_mode==1||reset_mode==4){
						//Config.
						scanPDM[0] = 10;
						scanPDM[1] = 20;
						scanPDM[2] = 0;
						progSel = 0;
						scanVel = 40;
						prescT = 20;
						prescD = 10;
						mtrOffsL = 100;
						mtrOffsR = 100;
						vVbatComp = 13;
						runDly = 20;
						toutTime = 5;
						brakeTime = 35;
						useATC = 1;
						for(gpVar2 = 0;gpVar2 < 14;gpVar2 ++){Sens.SensOffs[gpVar2] = 0;}
						updateConfMem();
						updateSensOffs();
						eeprom_update_word(&sensLLo_eep, 400);
						eeprom_update_word(&sensLHi_eep, 180);
						eeprom_update_word(&sensLo_eep, 1023);
						eeprom_update_word(&sensHi_eep, 0);
					}
					if(reset_mode==2||reset_mode==4){
						//PID
						PDMnow[0] = 3;
						PDMnow[1] = 10;
						PDMnow[2] = 0;
						updatePDMmem(0);
						PDMnow[0] = 17;
						PDMnow[1] = 30;
						PDMnow[2] = 0;
						for(gpVar1 = 1;gpVar1 < 5;gpVar1 ++){
							updatePDMmem(gpVar1);
							PDMnow[0] += 3;
							PDMnow[1] += 5;
						}
						PDMnow[0] = 15;
						PDMnow[1] = 80;
						PDMnow[2] = 1;
						for(gpVar1 = 5;gpVar1 < 10;gpVar1 ++){
							updatePDMmem(gpVar1);
							PDMnow[0] += 2;
							PDMnow[1] += 20;
						}
					}
					if(reset_mode==3||reset_mode==4){
						//Plan
						eeprom_update_word(&sensMask_eep[0], 0);
						for(gpVar1 = 1;gpVar1 < 12;gpVar1 ++){
							uint16_t tmp = pgm_read_word(&sensPrst[gpVar1-1]);
							eeprom_update_word(&sensMask_eep[gpVar1], tmp);
						}
						for(gpVar1 = 12;gpVar1 < 31;gpVar1 ++){
							eeprom_update_word(&sensMask_eep[gpVar1], 0);
						}
						for(progSel = 0;progSel < 10;progSel ++){
							deleteProg();
						}
						progSel = 0;
						eeprom_update_byte(&first_reset_eep, 1);
					}
					_delay_ms(300);
					LCD.Clear();
					break;
				}
			}
		}
	}
	//Read EEPROM
	readConfMem();
	readCPselMem();
	readCheckPoint();
	readSensCfg();
	readSensOffs();
	//PU-Sequence
	Motor.Config(1023, mtrOffsL, mtrOffsR);
	LCD.Cursor(6,0);
	LCD.WriteStr("GEAR");
	LCD.Cursor(5,1);
	LCD.WriteStr(VERSION);
	#ifdef DBG
	LCD.Cursor(13,1);
	LCD.WriteStr("DBG");
	#endif
	_delay_ms(300);
	LCD.Clear();
	SLED(0);
	
	for(gpVar1 = 0;gpVar1 < 6;gpVar1 ++){pb[gpVar1] = 0;}

	sei();
	
    while(1){
		Prescaler();
		Storage();
		Sampling();
		RunTime();
		Buttons();
		Interface();
    }
}

void RunTime(){
	if(sysRun){
		if(testPD){seq = 1;}
		BCKLT(0);
		sample = 1;
		if(runDtmr >= runDly){runDtmr = runDly;
			//Start Index
			if(seq == 0){
				SLED(0);
				if(Ttmr >= startIdx[0]){
					if(idxStop == 0){seq = 5; dspRfs = 1;}
					else{
						idxPtr ++;
						readRunMem = 1;
						seq ++;
						dspRfs = 1;}
				}
				else{
					mtrVelLeft = startIdx[1]*2;
					mtrVelRight = startIdx[1]*2;
				}
				Dtmr = 0;presc_Dms = 0;
				toutTmr = 0;
				brake = 1; brakeTmr = 0;
			}
			//Scan 
			else if(seq == 1){
				SLED(1);
				if(sensNewData){
					Line.Detect(0);
					sensNewData = 0;
					if(!testPD){
						switch(sensMask>>14){
							case 0:
							if((Line.SensByte & (sensMask&0b0011111111111111)) == (sensMask&0b0011111111111111)){seq ++;}
							break;
							case 1:
							if((Line.SensByte & (sensMask&0b0011111111111111)) > 0){seq ++;}
							break;
							case 2:
							if((((~Line.SensByte)&0b0011111111111111) & (sensMask&0b0011111111111111)) == (sensMask&0b0011111111111111)){seq ++;}
							break;
							case 3:
							if((((~Line.SensByte)&0b0011111111111111) & (sensMask&0b0011111111111111)) > 0){seq ++;}
							break;
						}
					}
					PDresult = (Line.LinePos * scanPDM[0]) + ((Line.LinePos - lastError) * scanPDM[1]);
					if(errTmr >= 5){errTmr = 0;lastError = Line.LinePos;}
					if(seq > 1){toutTmr = 0; dspRfs = 1;}
				}
				switch(scanPDM[2]){
					//RDC - Reduction
					case 0:
						if(PDresult < 0){mtrVelLeft = scanVel*2 + PDresult; mtrVelRight = scanVel*2;}
						else{mtrVelLeft = scanVel*2; mtrVelRight = scanVel*2 - PDresult;}
						if(Line.LinePos == -31){mtrVelLeft = -200; mtrVelRight = scanVel*2;}
						else if(Line.LinePos == 31){mtrVelLeft = scanVel*2; mtrVelRight = -200;}
					break;
					//DIF - Differential
					case 1:
						mtrVelLeft = scanVel*2 + PDresult;
						mtrVelRight = scanVel*2 - PDresult;
						if(Line.LinePos == -31){mtrVelLeft = -200; mtrVelRight = 200;}
						else if(Line.LinePos == 31){mtrVelLeft = 200; mtrVelRight = -200;}
					break;
				}
				if(toutTmr >= toutTime && toutTime != 0 && !testPD){seq = 5; tout = 1; dspRfs = 1; sensOff = 1;}
				gpVar1 = 0;
				Dtmr = 0;presc_Dms = 0;
				brakeTmr = 0;
			}
			//Delay Motion
			if(seq == 2){
				SLED(0);
				switch(indexBuff[0]&0b00001111){
					//Servo
					case 6:
						if(Dtmr >= 80){
							seq ++; dspRfs = 1;
						}
						servoPos[0] = indexBuff[2];
						servoPos[1] = indexBuff[3];
						servoPos[2] = indexBuff[4];
						mtrVelLeft = 0;
						mtrVelRight = 0;
					break;
					//GoTo
					case 7:
						readRunMem = 1;
						//Prog
						idxPtr = indexBuff[3];
						//Index
						progPtr = indexBuff[2];
						if(idxPtr == 0){seq = 0; toutTmr = 0; dspRfs = 1;}
						else{seq = 1; toutTmr = 0; dspRfs = 1;}
					break;
					//End
					case 8:
						seq = 5;
						dspRfs = 1;
					break;
					//Else
					default:
						if(sensNewData){Line.Detect(indexBuff[7]>>4);}
						if(Dtmr >= indexBuff[2]){
							if((indexBuff[0]>>4) > 0){
								if(Line.SensBits[(indexBuff[0]>>4)-1] == 0){gpVar1 = 1;}
								else if(gpVar1 == 1){seq ++; dspRfs = 1;}
								if(toutTmr >= toutTime && toutTime != 0){seq = 5; tout = 1; dspRfs = 1; sensOff = 1;}
							}
							else{
								seq ++; dspRfs = 1;}
						}
						mtrVelLeft = ((int8_t)indexBuff[4])*2;
						mtrVelRight = ((int8_t)indexBuff[3])*2;
						brake = 2;
					break;
				}
				Ttmr = 0;presc_Tms = 0;
				lastError = 0;
				PDresult = 0;
				brakeTmr = 0;
			}
			//Follow Line
			if(seq == 3){
				SLED(0);
				switch(indexBuff[0]&0b00001111){
					//Servo
					case 6:
						if(Ttmr >= 40){
							seq ++; dspRfs = 1;
						}
						servoPos[0] = indexBuff[5];
						servoPos[1] = indexBuff[6];
						servoPos[2] = indexBuff[7];
						mtrVelLeft = 0;
						mtrVelRight = 0;
					break;
					//Else
					default:
						if(sensNewData){
							Line.Detect(indexBuff[7]>>4);
							sensNewData = 0;
							PDresult = (Line.LinePos * PDMnow[0]) + ((Line.LinePos - lastError) * PDMnow[1]);
							if(errTmr >= 75){errTmr = 0;lastError = Line.LinePos;}
						}
						if(Ttmr >= indexBuff[5]){
							seq ++; dspRfs = 1;
							if(indexBuff[5] > 0){brake = 1;}
						}
						else{
							runVel = indexBuff[6]*2;
							switch(PDMnow[2]){
								//RDC - Reduction
								case 0:
									if(PDresult < 0){mtrVelLeft = runVel + PDresult; mtrVelRight = runVel;}
									else{mtrVelLeft = runVel; mtrVelRight = runVel - PDresult;}
									if(Line.LinePos == -31){mtrVelLeft = -200; mtrVelRight = runVel;}
									else if(Line.LinePos == 31){mtrVelLeft = runVel; mtrVelRight = -200;}
								break;
								//DIF - Differential
								case 1:
									mtrVelLeft = runVel+ PDresult;
									mtrVelRight = runVel - PDresult;
									if(Line.LinePos == -31){mtrVelLeft = -200; mtrVelRight = 200;}
									else if(Line.LinePos == 31){mtrVelLeft = 200; mtrVelRight = -200;}
								break;
							}
						}
					break;
				}
				Dtmr = 0;presc_Dms = 0;
				toutTmr = 0;
				brakeTmr = 0;
			}
			//Next
			if(seq == 4){
				SLED(0);
				if(idxPtr >= idxStop){seq = 5; dspRfs = 1;}
				else{idxPtr ++; seq = 1; toutTmr = 0; dspRfs = 1;}
				readRunMem = 1;
				Dtmr = 0;
				presc_Dms = 0;
				Ttmr = 0;
				presc_Tms = 0;
				toutTmr = 0;
				brakeTmr = 0;
			}
			//Terminate
			if(seq >= 5){
				SLED(0);
				sensOff = 1;
				runTotalTime = runTmr;
				if(brake > 0){
					if(brakeTmr < brakeTime){
						if(brake == 2){
							mtrVelLeft = (((int8_t)indexBuff[4]) < 0?200:-200);
							mtrVelRight = (((int8_t)indexBuff[3]) < 0?200:-200);
						}
						else{
							mtrVelLeft = -100;
							mtrVelRight = -100;
						}
					}
					else{
						sysRun = 0;
						mtrVelLeft = 0;
						mtrVelRight = 0;
						brake = 0;
					}
				}
				else{
					sysRun = 0;
				}
			}
		}
		else{
			SLED(1);
			Dtmr = 0;
			presc_Dms = 0;
			Ttmr = 0;
			presc_Tms = 0;
			toutTmr = 0;
			lastError = 0;
			errTmr = 0;
			runTotalTime = 0;
			tout = 0;
			brake = 0;
			brakeTmr = 0;
		}
		int16_t vBatDiff = 12600 - vbat;
		float vCompVal = 0;
		if(vVbatComp == 0){vBatDiff = 0;}
		if(vBatDiff < 0){vCompVal = vBatDiff*-1 / 1000.0 * (vVbatComp/10.0);}
		else if(vBatDiff > 0){vCompVal = vBatDiff / 100.0 * (vVbatComp/10.0);}
		Motor.Config(1023, mtrOffsL + vCompVal, mtrOffsR + vCompVal);
		Motor.Drive(mtrVelLeft*5, mtrVelRight*5);
	}
	else{
		BCKLT(1);
		SLED(0);
		runDtmr = 0;
		if(!(menu+pointer == 7 && submenu == 2)){Motor.Drive(0,0);}
		mtrVelLeft = 0;
		mtrVelRight = 0;
		Dtmr = 0;
		presc_Dms = 0;
		Ttmr = 0;
		presc_Tms = 0;
		lastError = 0;
		errTmr = 0;
		runTmr = 0;
		presc_100ms = 0;
		brakeTmr = 0;
	}
}

void Storage(){
	if(readRunMem){readRunMem = 0;
		uint8_t tmp = idxSel;
		uint8_t tmp2 = progSel;
		progSel = progPtr;
		idxSel = idxPtr;
		readProgMem(progPtr);
		readStartIdx();
		readStopIdx();
		readPDMmem(indexBuff[7]&0b00001111);
		readSensMem();
		if(idxPtr > idxStop){
			seq = 5; dspRfs = 1;
		}
		idxSel = tmp;
		progSel = tmp2;
	}
}

void Interface(){
	if((menu == 0 || cal > 0 || testPD == 1) && !pbPress && !lowBatt && !sensOff){sample = 1;}
	else{sample = 0;}
	if(pbPress || cal > 0){
		pbPress = 0;
		dspRfs = 1;
		if(submenu == 0 && menu > 0 && sidemenu == 0){
			if(pb[0] == 1){
				if(pointer == 1 && menu < 9){menu ++;}
				pointer = 1;
				LCD.Clear();
			}
			if(pb[1] == 1){
				if(pointer == 0 && menu > 1){menu --;}
				pointer = 0;
				LCD.Clear();
			}
			if(pb[2] == 3){
				menu = 0;pointer = 0;
				pb[2] = 0;
				LCD.Clear();}
		}
		switch(menu+pointer){
			//Idle
			case 0:
				if(!sysRun && !runDisp){
					if(pb[3] == 3){
						readConfMem();
						readStopIdx();
						readStartIdx();
						menu = 1;
						pointer = 0;
						LCD.Clear();
					}
					if(pb[4] && checkPointSel < 20){checkPointSel ++; updateCPselMem();}
					if(pb[5] && checkPointSel > 0){checkPointSel --; updateCPselMem();}
					if(pb[2] == 3){
						LCD.Clear();
						lowBatt = 0;
						if(vbat <= VBAT_MIN_LIMIT){
							runDisp = 1;
							lowBatt = 1;
						}			
						else{			
							sysRun = 1;
							idxPtr = checkPoint[checkPointSel];
							progPtr = progSel;
							if(idxPtr == 0){seq = 0;}
							else{seq = 1;}
							readRunMem = 1;
							runDisp = 1;
							tout = 0;
							Line.LinePos = 0;
						}
					}
				}
				else{
					if(pb[0] | pb[1] | pb[2] | (pb[3] == 3) | pb[4] | pb[5]){
						pb[0] = pb[1] = pb[2] = pb[3] = pb[4] = pb[5] = 0;
						sysRun = 0;
						runDisp = 0;
						sample = 1;
						lowBatt = 0;
						sensOff = 0;
						LCD.Clear();
					}
				}
			break;
			//Program
			case 1:
				if(sidemenu > 0 && submenu == 0){
					sidemenu = _lim(sidemenu+pb[0]-pb[1], 1, 3);
					if(pb[2] == 3){sidemenu = 0;LCD.Clear();}
					if(sidemenu == 1){
						gpVar1 = _lim(gpVar1+pb[4]-pb[5], 0, 9);
						if(pb[3] == 3){copyProg(gpVar1);sidemenu = 0;LCD.Clear();}
					}
					else if(sidemenu == 2){
						if(pb[3] == 3){invProg();sidemenu = 0;LCD.Clear();}
					}
					else{if(pb[3] == 3){deleteProg();sidemenu = 0;LCD.Clear();}}
				}
				else if(submenu > 0){
					if(sidemenu == 0){
						switch(submenu){
							case 1 ... 4:
								uppDsp = 0;
								if(idxSel > 0){submenu = _loop(submenu+pb[1]-pb[0], 1, 4);}
								else{submenu = _loop(submenu+pb[1]-pb[0], 1, 3);}
								if(pb[2] == 3){
									uppDsp = 1;
									submenu = 0;updateProgMem(progSel);updateStartIdx();LCD.Clear();}
								if((indexBuff[0]&0b00001111) != 8){
									if(pb[3] == 3){
										uppDsp = 1;
										if(idxSel > 0){submenu = 5;LCD.Clear();}
									}
								}
								switch(submenu){
									case 1:
										if(pb[4] == 1 && idxSel < 100){
											updateProgMem(progSel);
											updateStartIdx();
											idxSel ++;
											readProgMem(progSel);
											readSensMem();
											uppDsp = 0;
										}
										if(pb[5] == 1 && idxSel > 0){
											updateProgMem(progSel);
											idxSel --;
											readProgMem(progSel);
											readStartIdx();
											readSensMem();
											if(idxSel == 0){uppDsp = 1;}
										}
										if(pb[3] == 2 && idxSel > 0){uppDsp = 1;sidemenu = 1;gpVar1 = idxSel;shortDisp_P(3, idxMenuStr);}
									break;
									case 2:
										if(idxSel == 0){indexBuff[5] = _lim(indexBuff[5]+pb[4]-pb[5], 0, 255);}
										else{
											setMaskByte(&indexBuff[0], 0b00001111, _lim((indexBuff[0]&0b00001111)+pb[4]-pb[5], 0, 8));
											//Preset Values
											if(pb[4] | pb[5]){presetIdx();}
										}
									break;
									case 3:
										if(idxSel == 0){indexBuff[6] = _lim(indexBuff[6]+pb[4]-pb[5], 0, 100);}
										else{
											setMaskByte(&indexBuff[1], 0b0011111, _lim((indexBuff[1]&0b0011111)+pb[4]-pb[5], 0, 30));
											if(pb[3] == 2 && idxSel > 0){uppDsp = 0;sidemenu = 1;gpVar1 = 20;shortDisp_P(2, editSensStr);}
											if(pb[4] | pb[5]){readSensMem();}
										}
									break;
									case 4:setMaskByte(&indexBuff[1], 0b1100000, _lim((indexBuff[1]>>5)+pb[4]-pb[5], 0, 2)<<5);break;
								}
							break;
							case 5 ... 8:
								uppDsp = 1;
								switch(indexBuff[0]&0b00001111){
									//Servo
									case 6:
										submenu = _loop(submenu+pb[1]-pb[0], 5, 7);
										switch(submenu){
											case 5:indexBuff[2] = _lim(indexBuff[2]+pb[4]-pb[5], 1, 254);break;
											case 6:indexBuff[3] = _lim(indexBuff[3]+pb[4]-pb[5], 1, 254);break;
											case 7:indexBuff[4] = _lim(indexBuff[4]+pb[4]-pb[5], 1, 254);break;
										}
										if(pb[3] == 3){submenu = 9;LCD.Clear();}
									break;
									//GoTo
									case 7:
										submenu = _loop(submenu+pb[1]-pb[0], 5, 6);
										if(submenu == 5){indexBuff[2] = _lim(indexBuff[2]+pb[4]-pb[5], 0, 9);}
										else{indexBuff[3] = _lim(indexBuff[3]+pb[4]-pb[5], 0, 100);}
										if(pb[3] == 3){submenu = 1;LCD.Clear();uppDsp = 0;}
									break;
									default:
										submenu = _loop(submenu+pb[1]-pb[0], 5, 8);
										if(pb[3] == 3){submenu = 9;LCD.Clear();}
									break;
								}
								if((indexBuff[0]&0b00001111) < 6){
									switch(submenu){
										case 5:indexBuff[2] = _lim(indexBuff[2]+pb[4]-pb[5], 0, 255);break;
										case 6:
											setMaskByte(&indexBuff[0], 0b11110000, _lim((indexBuff[0]>>4)+pb[4]-pb[5], 0, 14)<<4);
										break;
										case 7:indexBuff[3] = _lim((int8_t)indexBuff[3]+pb[4]-pb[5], -100, 100);break;
										case 8:indexBuff[4] = _lim((int8_t)indexBuff[4]+pb[4]-pb[5], -100, 100);break;
									}
								}
								if(pb[2] == 3){submenu = 1;LCD.Clear();uppDsp = 0;}
							break;
							case 9 ... 12:
								if((indexBuff[0]&0b00001111) == 6){
									submenu = _loop(submenu+pb[1]-pb[0], 9, 11);
									switch(submenu){
										case 9:indexBuff[5] = _lim(indexBuff[5]+pb[4]-pb[5], 1, 254);break;
										case 10:indexBuff[6] = _lim(indexBuff[6]+pb[4]-pb[5], 1, 254);break;
										case 11:indexBuff[7] = _lim(indexBuff[7]+pb[4]-pb[5], 1, 254);break;
									}
								}
								else{
									submenu = _loop(submenu+pb[1]-pb[0], 9, 12);
									switch(submenu){
										case 9:indexBuff[5] = _lim(indexBuff[5]+pb[4]-pb[5], 0, 255);break;
										case 10:
											setMaskByte(&indexBuff[7], 0b00001111, _lim((indexBuff[7]&0b00001111)+pb[4]-pb[5], 0, 9));
										break;
										case 11:
											setMaskByte(&indexBuff[7], 0b11110000, _lim((indexBuff[7]>>4)+pb[4]-pb[5], 0, 4)<<4);
										break;
										case 12:indexBuff[6] = _lim(indexBuff[6]+pb[4]-pb[5], 0, 100);break;
									}
								}
							if(pb[3] == 3){submenu = 1;LCD.Clear();uppDsp = 0;}
							if(pb[2] == 3){submenu = 5;LCD.Clear();}
							break;
						}
						
							
					}
					else{
						switch(submenu){
							case 1:
								sidemenu = _lim(sidemenu+pb[0]-pb[1], 1, 3);
								if(pb[2] == 3){sidemenu = 0;LCD.Clear();uppDsp = 0;}
								switch(sidemenu){
									case 1:
										gpVar1 = _lim(gpVar1+pb[4]-pb[5], 0, 100);
										if(pb[3] == 3){
											uint8_t srcIdx = idxSel;
											readProgMem(progSel);
											idxSel = gpVar1;
											updateProgMem(progSel);
											idxSel = srcIdx;
											sidemenu = 0;LCD.Clear();
											uppDsp = 0;
										}
									break;
									case 2:
										if(pb[3] == 3){
											uint8_t gpVar1 = idxSel;
											for(idxSel = 100;idxSel > gpVar1;idxSel --){
												uint8_t sidx = idxSel;
												idxSel -= 1;
												readProgMem(progSel);
												idxSel = sidx;
												updateProgMem(progSel);
											}
											arrcpy(indexBuff, indexDefVal, 8);
											idxSel = gpVar1;
											updateProgMem(progSel);
											readSensMem();
											sidemenu = 0;LCD.Clear();
											uppDsp = 0;
										}
									break;
									case 3:
										if(pb[3] == 3){
											uint8_t gpVar1 = idxSel;
											while(idxSel < 100){
												uint8_t sidx = idxSel;
												idxSel += 1;
												readProgMem(progSel);
												idxSel = sidx;
												updateProgMem(progSel);
												idxSel ++;
											}
											arrcpy(indexBuff, indexDefVal, 8);
											idxSel = 100;
											updateProgMem(progSel);
											idxSel = gpVar1;
											readProgMem(progSel);
											readSensMem();
											sidemenu = 0;LCD.Clear();
											uppDsp = 0;
										}
									break;
								}
							break;
							case 3:
								sidemenu = _loop(sidemenu+pb[1]-pb[0], 1, 15);
								if(pb[2] == 3 || pb[3] == 3){sidemenu = 0;updateSensMem();LCD.Clear();uppDsp = 0;}
								switch(sidemenu){
									case 1 ... 14:
										if(pb[4]){_SET(&sensMask, 14-sidemenu, 1);}
										if(pb[5]){_SET(&sensMask, 14-sidemenu, 0);}
									break;
									case 15:
										if((sensMask>>14) > 0 && pb[5]){sensMask = (((sensMask>>14)-1)<<14) | (sensMask&0b0011111111111111);}
										if((sensMask>>14) < 3 && pb[4]){sensMask = (((sensMask>>14)+1)<<14) | (sensMask&0b0011111111111111);}
									break;
								}
							break;
							case 4:
							
							break;
						}
					}
				}
				else{
					progSel = _lim(progSel+pb[4]-pb[5],0,9);
					if(pb[4] | pb[5]){
						readStartIdx();
						readStopIdx();
						updateConfMem();
						readCPselMem();
						readCheckPoint();}
					if(pb[3] == 2){
						sidemenu = 1;
						shortDisp_P(3, progMenuStr);
						gpVar1 = progSel;}
					else if(pb[3] == 3){
						if(idxSel > 0){uppDsp = 0;}
						submenu = 1;
						readProgMem(progSel);readStartIdx();
						readSensMem();
						LCD.Clear();}
				}
			break;
			//Stop at
			case 2:
				idxStop = _lim(idxStop+pb[4]-pb[5], 0, 100);
				if(pb[4] | pb[5]){updateStopIdx();}
			break;
			//Checkpoint
			case 3:
				switch(submenu){
					case 0:if(pb[3] == 3){submenu = 1;gpVar1 = 0;readCheckPoint();LCD.Clear();}break;
					case 1:
						gpVar1 = _lim(gpVar1+pb[4]-pb[5], 0, 20);
						submenu = _lim(submenu+pb[0]-pb[1], 1, 2);
						if(pb[2] == 3){submenu = 0;updateCheckPoint();LCD.Clear();}
					break;
					case 2:
						checkPoint[gpVar1] = _lim(checkPoint[gpVar1]+pb[4]-pb[5], 0, 100);
						submenu = _lim(submenu+pb[0]-pb[1], 1, 2);
						if(pb[2] == 3){submenu = 0;updateCheckPoint();LCD.Clear();}
					break;
				}
			break;
			//PD
			case 4:
				if(submenu > 0){
					if(PDMsel > 0){
						submenu = _loop(submenu+pb[1]-pb[0], 1, 3);
						if(pb[2] == 3){submenu = 0;LCD.Clear();updatePDMmem(PDMsel-1);}
					}
					else{
						submenu = _loop(submenu+pb[1]-pb[0], 1, 4);
						if(pb[2] == 3){submenu = 0;LCD.Clear();
							arrcpy(scanPDM, PDMnow, 3);
							updateConfMem();
						}
					}
					switch(submenu){
						case 1:PDMnow[0] = _lim(PDMnow[0]+pb[4]-pb[5], 0, 255);break;
						case 2:PDMnow[1] = _lim(PDMnow[1]+pb[4]-pb[5], 0, 255);break;
						case 3:PDMnow[2] = _lim(PDMnow[2]+pb[4]-pb[5], 0, 1);break;
						case 4:scanVel = _lim(scanVel+pb[4]-pb[5], 0, 100);break;
					}
				}
				else{
					PDMsel = _lim(PDMsel+pb[4]-pb[5], 0 , 10);
					if(pb[3] == 3){submenu = 1;
						if(PDMsel == 0){
							readConfMem();
							arrcpy(PDMnow, scanPDM, 3);
						}
						else{readPDMmem(PDMsel-1);}
					}
				}
			break;
			//Sensor
			case 5:
				if(submenu > 0){
					if(submenu == 1){
						useATC = _lim(useATC+pb[4]-pb[5], 0, 1);
						if(pb[4] | pb[5]){
							updateConfMem();
							readSensCfg();
						}
					}
					if(useATC && cal == 0){
						submenu = _lim(submenu+pb[0]-pb[1], 1 ,3);
						if(pb[1] && submenu == 1){updateSensCfg();}
						switch(submenu){
							case 2:sensLo = _lim(sensLo+pb[4]-pb[5], 0, 1023);break;
							case 3:sensHi = _lim(sensHi+pb[4]-pb[5], 0, 300);break;
						}
					}
					else{
						if(cal == 0){submenu = _lim(submenu+pb[0]-pb[1], 1 ,2);}
						if(submenu == 2){
							if(cal == 0){
								if(pb[3] == 3){
									uppDsp = 0;
									pb[3] = 0;
									cal = 1;
									sensHi = 0;
									sensLo = 1023;
									LCD.Clear();
								}
							}
							else{
								if(pb[1]){
									cal = 2;
									sensHi = 1023; sensLo = 0;
									for(gpVar2 = 0;gpVar2 < 14;gpVar2 ++){Sens.SensOffs[gpVar2] = 0;}
								}
								if(cal == 1){
									if(Sens.ResultLow < sensLo && Sens.ResultHigh != 1023){sensLo = Sens.ResultLow;}
									if(Sens.ResultHigh > sensHi && Sens.ResultHigh != 1023){sensHi = Sens.ResultHigh;}
								}
								if(pb[3] == 3){
									if(cal == 1){updateSensCfg();}
									else if(cal == 2){
										for(gpVar2 = 0;gpVar2 < 14;gpVar2 ++){Sens.SensOffs[gpVar2] = Sens.ResultRaw[gpVar2] - Sens.ResultLow;}
										updateSensOffs();
										readSensCfg();
									}
									uppDsp = 1;
									pb[3] = 0;
									cal = 0;
									LCD.Clear();
								}
								if(pb[2] == 3){
									if(cal == 1){updateSensCfg();}
									else if(cal == 2){
										for(gpVar2 = 0;gpVar2 < 14;gpVar2 ++){Sens.SensOffs[gpVar2] = Sens.ResultRaw[gpVar2] - Sens.ResultLow;}
										updateSensOffs();
										readSensCfg();
									}
									uppDsp = 1;
									pb[2] = 0;
									cal = 0;
									LCD.Clear();
								}
							}
						}
					}
					if(pb[2] == 3){
						updateSensCfg();
						submenu = 0;uppDsp = 1;LCD.Clear();}
				}
				else{
					if(pb[3] == 3){
						readSensCfg();
						submenu = 1;LCD.Clear();}
				}
			break;
			//Motor Offset
			case 6:
				if(submenu > 0){
					submenu = _lim(submenu+pb[0]-pb[1], 1 ,2);
					if(pb[2] == 3){submenu = 0;
						Motor.Config(1023, mtrOffsL, mtrOffsR);
						updateConfMem();LCD.Clear();
					}
					if(submenu == 1){mtrOffsL = _lim(mtrOffsL+pb[4]-pb[5], 1, 199);}
					else{mtrOffsR = _lim(mtrOffsR+pb[4]-pb[5], 1, 199);}
				}
				else{
					if(pb[3] == 3){submenu = 1;LCD.Clear();}
				}
			break;
			//Selftest
			case 7:
				if(submenu > 0){
					submenu = _lim(submenu+pb[0]-pb[1], 1 ,2);
					if(submenu == 1){
						if(testPD == 0){ 
							if(pb[3] == 3){
								testPD = 1; sysRun = 1; sensOff = 0;
								if(PDMsel == 0){readConfMem();}
								else{
									readPDMmem(PDMsel-1);
									scanPDM[0] = PDMnow[0];
									scanPDM[1] = PDMnow[1];
									scanPDM[2] = PDMnow[2];}
							}
						}
						else{
							if(pb[0] | pb[1] | pb[2] | pb[3] | pb[4] | pb[5]){
								pb[2] = 0;testPD = 0;sysRun = 0;
								readConfMem();}
						}
						gpVar1 = 0;
					}
					else{testPD = 0; sysRun = 0;}
					if(pb[2] == 3){submenu = 0;testPD = 0;sysRun = 0;readConfMem(); LCD.Clear();}
					if(submenu == 2){
						gpVar1 = _lim(gpVar1+pb[4]-pb[5], 0, 4);
						switch(gpVar1){
							case 0:Motor.Drive(0,0);break;
							case 1:Motor.Drive(-500,-500);break;
							case 2:Motor.Drive(500,500);break;
							case 3:Motor.Drive(500,-500);break;
							case 4:Motor.Drive(-500,500);break;
						}
					}
				}
				else{
					testPD = 0;
					sysRun = 0;
					if(pb[3] == 3){submenu = 1;gpVar1 = 0;LCD.Clear();}
					Motor.Drive(0,0);
				}
			break;
			//Prescalers
			case 8:
				if(submenu > 0){
					submenu = _lim(submenu+pb[0]-pb[1], 1 ,2);
					if(submenu == 1){prescT = _lim(prescT+pb[4]-pb[5], 1, 100);}
					else{prescD = _lim(prescD+pb[4]-pb[5], 1, 100);}
					if(pb[2] == 3){submenu = 0;updateConfMem();LCD.Clear();}
				}
				else{
					if(pb[3] == 3){submenu = 1;LCD.Clear();}
				}
			break;
			//Vbat
			case 9:
				if(pb[3] == 3){submenu = 1;LCD.Clear();}
				if(submenu > 0 && pb[2] == 3){submenu = 0;LCD.Clear();}
			break;
			//Settings
			case 10:
				if(submenu > 0){
					if(pb[0] == 1){
						if(gpVar1 == 1 && submenu < 4){submenu ++;}
						gpVar1 = 1;
						LCD.Clear();
					}
					if(pb[1] == 1){
						if(gpVar1 == 0 && submenu > 1){submenu --;}
						gpVar1 = 0;
						LCD.Clear();
					}
					if(pb[2] == 3){submenu = 0;updateConfMem();LCD.Clear();}
					switch(submenu+gpVar1){
						case 1:runDly = _lim(runDly+pb[4]-pb[5], 5, 50);break;//Run Delay
						case 2:toutTime = _lim(toutTime+pb[4]-pb[5], 0, 50);break;//Timeout
						case 3:brakeTime = _lim(brakeTime+pb[4]-pb[5], 0, 255);break;//End Brake
						case 4:vVbatComp = _lim(vVbatComp+pb[4]-pb[5], 0, 200);break;//V-Vbat Comp
						case 5:break;//Diagnostics
					}
				}
				else{
					if(pb[3] == 3){submenu = 1;gpVar1 = 0;LCD.Clear();}
				}
			break;
		}
	}
	if(dspRfs){
		dspRfs = 0;
		if(submenu == 0 && menu > 0 && sidemenu == 0){
			for(uint8_t i = 0;i < 2;i ++){
				uint8_t tmp = menu - (1 - i);
				strcpy_P(strBuff, menuStr[tmp]);
				switch(tmp){
					case 0:sprintf(dspBuff[i], "%s%c [%d] ", strBuff, (progInv?'\'':' '), progSel+1);break;
					case 1:sprintf(dspBuff[i], "%s%d  ", strBuff, idxStop);break;
					case 3:
						if(PDMsel <= 0){sprintf(dspBuff[i], "%s[S] ", strBuff);}
						else{sprintf(dspBuff[i], "%s[%d] ", strBuff, PDMsel);}
					break;
					case 8:sprintf(dspBuff[i], "%s%d.%d%dV ", strBuff, vbat/1000, vbat/100%10, vbat/10%10);break;
					default:sprintf(dspBuff[i], "%s ", strBuff);break;
				}
			}
			dspCursor = (pointer?17:1);
			dspCursorChr = '>';
		}
		switch(menu+pointer){
			//Idle
			case 0:
				#ifndef DBG
				if(runDisp && lowBatt){
					LCD.Cursor(2,0);
					sprintf(dspBuff[0], "Low Battery!");
					LCD.WriteStr(dspBuff[0]);
					LCD.Cursor(5,1);
					sprintf(dspBuff[0], "%d.%d%dV ", vbat/1000, vbat/100%10, vbat/10%10);
					LCD.WriteStr(dspBuff[0]);
				}
				else if(runDisp){
					LCD.Cursor(0,0);
					sprintf(dspBuff[0], "i%d  ", (tout?idxPtr-1:idxPtr));
					LCD.WriteStr(dspBuff[0]);
					LCD.Cursor(0,1);
					sprintf(dspBuff[0], "P%d%c ", progPtr+1, (progInv?'\'':' '));
					LCD.WriteStr(dspBuff[0]);
					if(tout){sprintf(dspBuff[0], "TIMEOUT");}
					else if(seq == 1){sprintf(dspBuff[0], " SCAN  ");}
					else if(seq < 5){sprintf(dspBuff[0], " RUN   ");}
					else{sprintf(dspBuff[0], " STOP  ");}
					LCD.Cursor(5,0);
					LCD.WriteStr(dspBuff[0]);
					if(seq >= 5 || tout){
						LCD.Cursor(5,1);
						sprintf(dspBuff[0], "%u:%u%u.%u", (uint8_t)(runTotalTime/600),(uint8_t)(runTotalTime%600/100%10),(uint8_t)(runTotalTime%600/10%10),(uint8_t)(runTotalTime%600%10));
						LCD.WriteStr(dspBuff[0]);
					}
				}
				else{
					for(uint8_t i=0;i<14;i++){
						LCD.Cursor(1+i,0);
						if(Sens.ResultRaw[i]>1015){LCD.WriteChr('x');}
						else{
							BarGraph(Sens.ResultRaw[i]*9/(Line.Threshold==0?1:Line.Threshold*1.7));
						}
					}
					LCD.Cursor(0,1);
					sprintf(strBuff, "CP%d-i%d  ", checkPointSel, checkPoint[checkPointSel]);
					LCD.WriteStr(strBuff);
					LCD.Cursor(12,1);
					sprintf(strBuff, "P%d%c ", progSel+1, (progInv?'\'':' '));
					LCD.WriteStr(strBuff);
//					LCD.Cursor(0,1);
//					sprintf(strBuff, "%dmV  ", vbat);
//					LCD.WriteStr(strBuff);
				}
				#else
				for(uint8_t i=0;i<14;i++){
					LCD.Cursor(1+i,0);
					if(Sens.ResultRaw[i]>1015){LCD.WriteChr('x');}
					else{
						BarGraph(Sens.ResultRaw[i]*9/(Line.Threshold==0?1:Line.Threshold*1.7));
					}
				}
				LCD.Cursor(1,1);
				for(uint8_t i = 0;i < 14;i ++){
					LCD.WriteInt(_READ(&Line.SensByte, 13-i));
				}
				#endif
			break;
			//Program
			case 1:
				if(sidemenu > 0 && submenu == 0){
					sprintf(dspBuff[0], " Copy to P%d ", gpVar1+1);
					strcpy_P(strBuff, invNrmStr[progInv]);
					sprintf(dspBuff[1], " %s  Del  [P%d] ", strBuff, progSel+1);
					switch(sidemenu){
						case 1:dspCursor = 1; dspCursorChr = '>';break;
						case 2:dspCursor = 17; dspCursorChr = '>';break;
						case 3:dspCursor = 22; dspCursorChr = '>';break;
					}
				}
				else if(submenu > 0){
					switch(submenu){
						case 1 ... 4:
							if(idxSel == 0){
								strcpy_P(dspBuff[0], startIdxStr);
								sprintf(dspBuff[1], " i%-3d T%-3d  V%-3d ", idxSel, indexBuff[5], indexBuff[6]);
							}
							else{
								if(uppDsp == 0){
									LCD.Cursor(0,0);
									if(sidemenu == 0){LCD.WriteChr(pgm_read_byte(&lineColStr[((indexBuff[1]&0b1100000)>>5)]));}
									else{LCD.WriteChr(' ');}
									sensDisp(sensMask);
								}
								strcpy_P(strBuff, idxCmdStr[(indexBuff[0]&0b00001111)]);
								sprintf(dspBuff[1], " i%-3d%c %s S%d ", idxSel, (progInv?'\'':' '), strBuff, (indexBuff[1]&0b11111));
							}
							switch(submenu){
								case 1:
									if(sidemenu > 0){
										sprintf(dspBuff[0], " Copy to i%d ", gpVar1);
										sprintf(dspBuff[1], " Ins  Del [i%d] ", idxSel);
										switch(sidemenu){
											case 1:dspCursor = 1; dspCursorChr = '>';break;
											case 2:dspCursor = 17; dspCursorChr = '>';break;
											case 3:dspCursor = 22; dspCursorChr = '>';break;
										}
									}
									else{
										dspCursor = 17; dspCursorChr = '>';
									}
								break;
								case 2:
									if(idxSel > 0){
										dspCursor = 23; dspCursorChr = '>';
									}
									else{
										dspCursor = 22; dspCursorChr = '>';
									}
								break;
								case 3:
									if(sidemenu > 0){
										LCD.Cursor(1,0);
										sensDisp(sensMask);
										if(sidemenu < 7){sprintf(dspBuff[1], "        S%d      ", (indexBuff[1]&0b0011111));}
										else{sprintf(dspBuff[1], " S%d             ", (indexBuff[1]&0b0011111));}
										switch(sidemenu){
											case 1 ... 15:
												dspCursor = 17 + sidemenu; dspCursorChr = '^';
											break;
										}
									}
									else if(idxSel > 0){
										dspCursor = 29; dspCursorChr = '>';
									}
									else{
										dspCursor = 28; dspCursorChr = '>';
									}
								break;
								case 4:dspCursor = 17; dspCursorChr = '^';break;
							}
						break;
						case 5 ... 8:
							switch(indexBuff[0]&0b00001111){
								//Servo
								case 6:
									sprintf(dspBuff[0], " S1 %-3d   S2 %d ", indexBuff[2], indexBuff[3]);
									sprintf(dspBuff[1], " [1]      S3 %d ", indexBuff[4]);
									switch(submenu){
										case 5:dspCursor = 1; dspCursorChr = '>';break;
										case 6:dspCursor = 10; dspCursorChr = '>';break;
										case 7:dspCursor = 26; dspCursorChr = '>';break;
									}
								break;
								//GoTo
								case 7:
									sprintf(dspBuff[0], " Prog: %-3d  ", indexBuff[2]+1);
									sprintf(dspBuff[1], " Index: %-3d ", indexBuff[3]);
									switch(submenu){
										case 5:dspCursor = 1; dspCursorChr = '>';break;
										case 6:dspCursor = 17; dspCursorChr = '>';break;
									}
								break;
							}
							if((indexBuff[0]&0b00001111) < 6){
								strcpy_P(strBuff, dlySensStr[(indexBuff[0]>>4)]);
								sprintf(dspBuff[0], " D%-3d     s%s ", indexBuff[2], strBuff);
								sprintf(dspBuff[1], " L%-4i    R%i ", (int8_t)indexBuff[4], (int8_t)indexBuff[3]);
								switch(submenu){
									case 5:dspCursor = 1; dspCursorChr = '>';break;
									case 6:dspCursor = 10; dspCursorChr = '>';break;
									case 7:dspCursor = 26; dspCursorChr = '>';break;
									case 8:dspCursor = 17; dspCursorChr = '>';break;
								}
							}
						break;
						case 9 ... 12:
							if((indexBuff[0]&0b00001111) == 6){
								sprintf(dspBuff[0], " S1 %-3d   S2 %d ", indexBuff[5], indexBuff[6]);
								sprintf(dspBuff[1], " [2]      S3 %d ", indexBuff[7]);
								switch(submenu){
									case 9:dspCursor = 1; dspCursorChr = '>';break;
									case 10:dspCursor = 10; dspCursorChr = '>';break;
									case 11:dspCursor = 26; dspCursorChr = '>';break;
								}
							}
							else{
								strcpy_P(strBuff, lineModeStr[indexBuff[7]>>4]);
								sprintf(dspBuff[0], " T%-3d     PD%d ", indexBuff[5], (indexBuff[7]&0b00001111)+1);
								sprintf(dspBuff[1], " V%-3d     %s ", indexBuff[6], strBuff);
								switch(submenu){
									case 9:dspCursor = 1; dspCursorChr = '>';break;
									case 10:dspCursor = 10; dspCursorChr = '>';break;
									case 11:dspCursor = 26; dspCursorChr = '>';break;
									case 12:dspCursor = 17; dspCursorChr = '>';break;
								}
							}
						break;
					}
				}
			break;
			//Stop at
			case 2:
			break;
			//CheckPoint
			case 3:
				if(submenu > 0){
					sprintf(dspBuff[0], " CheckPoint: %d ", gpVar1);
					sprintf(dspBuff[1], " Index: %d ", checkPoint[gpVar1]);
					switch(submenu){
						case 1:dspCursor = 1; dspCursorChr = '>';break;
						case 2:dspCursor = 17; dspCursorChr = '>';break;
					}
				}
			break;
			//PD
			case 4:
				if(submenu > 0){
					strcpy_P(strBuff, pidModeStr[PDMnow[2]]);
					sprintf(dspBuff[0], " Kp%-3d    Kd%d ", PDMnow[0], PDMnow[1]);
					if(PDMsel == 0){sprintf(dspBuff[1], " V%-3d     %s ", scanVel, strBuff);}
					else{sprintf(dspBuff[1], "          %s ", strBuff);}
					switch(submenu){
						case 1:dspCursor = 1; dspCursorChr = '>';break;
						case 2:dspCursor = 10; dspCursorChr = '>';break;
						case 3:dspCursor = 26; dspCursorChr = '>';break;
						case 4:dspCursor = 17; dspCursorChr = '>';break;
					}
				}
			break;
			//Sensor
			case 5:
				if(submenu > 0){
					if(cal > 0){
						for(uint8_t i=0;i<14;i++){
							LCD.Cursor(1+i,0);
							BarGraph((Sens.ResultRaw[i]-Sens.ResultLow)*9/(Line.Threshold*1.7));
						}
						switch(cal){
							case 1:sprintf(dspBuff[1], "Cal  L%-4d H%d  ", sensLo, sensHi);break;
							case 2:sprintf(dspBuff[1], "Offs. White     ");break;
						}
						dspCursor = 0;
					}
					else{
						strcpy_P(strBuff, offOnStr[useATC]);
						sprintf(dspBuff[0], " ATC %s ", strBuff);
						if(useATC){sprintf(dspBuff[1], " DM:%-4d TH:%d ", sensLo, sensHi);}
						else{sprintf(dspBuff[1], " Calibrate      ");}
						switch(submenu){
							case 1:dspCursor = 1; dspCursorChr = '>';break;
							case 2:dspCursor = 17; dspCursorChr = '>';break;
							case 3:dspCursor = 25; dspCursorChr = '>';break;
						}
					}
				}
			break;
			//Motor Offset
			case 6:
				if(submenu > 0){
					sprintf(dspBuff[0], " Offs.L: %d.%d%d ", mtrOffsL/100, mtrOffsL/10%10, mtrOffsL%10);
					sprintf(dspBuff[1], " Offs.R: %d.%d%d ", mtrOffsR/100, mtrOffsR/10%10, mtrOffsR%10);
					switch(submenu){
						case 1:dspCursor = 1; dspCursorChr = '>';break;
						case 2:dspCursor = 17; dspCursorChr = '>';break;
					}
				}
			break;
			//Selftest
			case 7:
				if(submenu > 0){
					strcpy_P(strBuff, stopRunStr[testPD]);
					sprintf(dspBuff[0], " Test PD %s ", strBuff);
					strcpy_P(strBuff, idxCmdStr[5-gpVar1]);
					sprintf(dspBuff[1], " Motor: %s ", strBuff);
					switch(submenu){
						case 1:dspCursor = 1; dspCursorChr = '>';break;
						case 2:dspCursor = 17; dspCursorChr = '>';break;
					}
				}
			break;
			//Prescalers
			case 8:
				if(submenu > 0){
					sprintf(dspBuff[0], " T/Div: %dms ", prescT);
					sprintf(dspBuff[1], " D/Div: %dms ", prescD);
					switch(submenu){
						case 1:dspCursor = 1; dspCursorChr = '>';break;
						case 2:dspCursor = 17; dspCursorChr = '>';break;
					}
				}
			break;
			//Vbat
			case 9:
				if(submenu > 0){
					strcpy_P(dspBuff[0], aboutDisp[0]);
					strcpy_P(dspBuff[1], aboutDisp[1]);
					dspCursor = 0;
				}
			break;
			//Settings
			case 10:
				if(submenu > 0){
					for(uint8_t i = 0;i < 2;i ++){
						uint8_t tmp = submenu - (1 - i);
						strcpy_P(strBuff, settStr[tmp]);
						switch(tmp){
							case 0:sprintf(dspBuff[i], "%s%dms ", strBuff, runDly*10);break;
							case 1:
								if(toutTime > 0){sprintf(dspBuff[i], "%s%d.%ds ", strBuff, toutTime/10, toutTime%10);}
								else{sprintf(dspBuff[i], "%sOFF ", strBuff);}		
							break;
							case 2:sprintf(dspBuff[i], "%s%dms ", strBuff, brakeTime);break;
							case 3:
								if(vVbatComp > 0){sprintf(dspBuff[i], "%s%d.%d ", strBuff, vVbatComp/10, vVbatComp%10);}
								else{sprintf(dspBuff[i], "%sOFF  ", strBuff);}	
							break;
							default:sprintf(dspBuff[i], "%s ", strBuff);break;
						}
					}
					dspCursor = (gpVar1?17:1);
					dspCursorChr = '>';
				}
			break;
		}
		if(menu != 0){
			if(uppDsp == 1){
				LCD.Cursor(0,0);
				LCD.WriteStr(dspBuff[0]);
			}
			LCD.Cursor(0,1);
			LCD.WriteStr(dspBuff[1]);
			if(dspCursor > 0){
				LCD.Cursor((dspCursor>16?dspCursor-17:dspCursor-1), dspCursor > 16);
				LCD.WriteChr(dspCursorChr);
			}
		}
	}
}

void Sampling(){
	if(sampleTmr >= 20 && sample){sampleTmr = 0;
		Sens.Read();
		Sens.Ctrl(1);
		#ifndef DBG
		if(Sens.Ctrl(1) && !sysRun){
			//Servo PWM
			if(servoTmr >= 15){
				servoTmr = 0;
				IO1(1);IO2(1);IO3(1);
				for(uint8_t l=0;l<240;l++){
					if(l>=servoPos[0]+35){IO1(0);}
					if(l>=servoPos[1]+35){IO2(0);}
					if(l>=servoPos[2]+35){IO3(0);}
				_delay_us(10);}
				IO1(0);IO2(0);IO3(0);
				_delay_ms(1.6);
			}
			else{_delay_ms(4);}
			vbat += (readVbat() + readVbat()) / 2;
			vbat /= 2;
		}
		#else
		if(Sens.Ctrl(1)){_delay_ms(4);}
		#endif
		if(Sens.NewData()){
			if(useATC){
				Line.Digitize(	(Sens.ResultLow),
								(Sens.ResultHigh),
								sensLo, sensHi,
								Sens.ResultRaw,
								(indexBuff[1]>>5));

			}
			else{
				Line.Digitize(	(sensLo),
								(sensHi),
								400, 180,
								Sens.ResultRaw,
								(indexBuff[1]>>5));
			}
			#ifndef DBG
			if(!sysRun){dspRfs = 1;}
			#else
			dspRfs = 1;
			#endif
			sensNewData = 1;
		}
	}
	else if(sampleTmr >= 200 && !sample){
		Sens.Ctrl(0);
		vbat += (readVbat() + readVbat()) / 2;
		vbat /= 2;
		if(menu+pointer >= 8 && submenu == 0){dspRfs = 1;}
	}
	else if(!sample && !(menu+pointer >= 8 && submenu == 0)){
		Sens.Ctrl(0);
		sampleTmr = 0;
	}
}
	
void Buttons(){
	//Button Handler
	BTNCOM(0);BTNCOM(0);
	pb[0] = BTN1;
	pb[1] = BTN2;
	pb[2] = BTN3;
	BTNCOM(1);BTNCOM(1);
	pb[3] = BTN4;
	pb[4] = BTN5;
	pb[5] = BTN6;
	if(!(pb[0] | pb[1] | pb[2] | pb[3] | pb[4] | pb[5])){pbTimer = 0;if(pbLast == 1 || pbLast == 4){pbLast = 0;}}
	if(pbTimer >= 5){
		if(pb[0] | pb[1] | pb[4] | pb[5]){
			if(pbTimer >= 50){
				pbTimer = 47;
			}
			else if(pbLast == 1){
				for(uint8_t i = 0;i < 6;i ++){
					pb[i] = 0;
				}
			}
			pbLast = 1;
		}	
		if(pb[2] | pb[3]){
			if(pbTimer >= 30){
				if(pbLast == 2 || pbLast == 3){
					pb[2] *= 2;
					pb[3] *= 2;
					pbLast = 4;
				}
				pbTimer = 30;
			}
			else if(pbLast == 0){
				if(pb[2]){pbLast = 2;}
				if(pb[3]){pbLast = 3;}
			}
		}
	}
	else{
		for(uint8_t i = 0;i < 6;i ++){
			pb[i] = 0;
		}
	}
	if(!pb[2] && pbLast == 2){pb[2] = 3;pbLast = 0;}
	if(!pb[3] && pbLast == 3){pb[3] = 3;pbLast = 0;}
	pbPress = 0;
	for(uint8_t i = 0;i < 6;i ++){
		if(pb[i] > 0 && pb[i] < 4){pbPress = 1;}
	}
	if(pb[2] == 1 || pb[3] == 1){pbPress = 0;}
}

void Prescaler(){
	//Timing
	if(tbase >= 1){
		presc_1ms += tbase;
		sampleTmr += tbase;
		errTmr += tbase;
		presc_Tms += presc_1ms/8;
		presc_Dms += presc_1ms/8;
		brakeTmr += presc_1ms/8;
		tbase = 0;
	}
	if(presc_1ms >= 8){
		presc_10ms += presc_1ms/8;
		presc_100ms += presc_1ms/8;
		servoTmr += presc_1ms/8;
		presc_1ms = 0;
	}
	if(presc_10ms >= 10){
		pbTimer ++;
		runDtmr ++;
		presc_10ms = 0;
	}
	if(presc_100ms >= 100){
		toutTmr ++;
		runTmr += presc_100ms/100;
		presc_100ms = 0;	
	}
	if(presc_Dms >= prescD){
		Dtmr += presc_Dms/prescD;
		presc_Dms = 0;
	}
	if(presc_Tms >= prescT){
		Ttmr += presc_Tms/prescT;
		presc_Tms = 0;
	}
}

