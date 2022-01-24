#ifndef _LINEDETECTORLIB_H
#define _LINEDETECTORLIB_H

static const uint16_t lineLUT[] = {
	0b10000000000000,
	0b11000000000000,
	0b01000000000000,
	0b01100000000000,
	0b00100000000000,
	0b00110000000000,
	0b00010000000000,
	0b00011000000000,
	0b00001000000000,
	0b00001100000000,
	0b00000100000000,
	0b00000110000000,
	0b00000010000000,
	0b00000011000000,//13
	0b00000001000000,
	0b00000001100000,
	0b00000000100000,
	0b00000000110000,
	0b00000000010000,
	0b00000000011000,
	0b00000000001000,
	0b00000000001100,
	0b00000000000100,
	0b00000000000110,
	0b00000000000010,
	0b00000000000011,
	0b00000000000001
};

class LineDetectorLib{
	public:
		void Digitize(uint16_t, uint16_t, uint16_t, uint16_t, uint16_t[], uint8_t);	
		void Detect(uint8_t);
		uint16_t DeltaVal;
		uint16_t Threshold;
		int8_t LinePos;
		bool SensBits[14];
		uint16_t SensByte;
		uint16_t avgResLow;
		uint16_t avgResHigh;
};

void LineDetectorLib::Digitize(uint16_t ResultLow, uint16_t ResultHigh, uint16_t DeltaMin, uint16_t ThresDiv, uint16_t ResultRaw[], uint8_t abw){
	uint8_t blkCnt = 0;
	uint8_t i = 0;
	DeltaVal = ResultHigh - ResultLow;
	if(DeltaVal < DeltaMin){
		if(ResultLow > 400){Threshold = 0;}
		else{Threshold = 1023;}
	}
	else{
		if(ResultLow < avgResLow){avgResLow = ResultLow;}
		else{
			avgResLow += ResultLow/100;
		}
		if(ResultHigh > avgResHigh){avgResHigh = ResultHigh;}
		else{
			avgResHigh -= ResultHigh/100;
		}
		DeltaVal = ResultHigh - ResultLow;
		Threshold = DeltaVal/(ThresDiv/100.0);
		Threshold += ResultLow;
	}
	for(i = 0;i < 14;i ++){
		if(ResultRaw[i] >= Threshold){SensBits[i] = 1;}
		//else if(ResultRaw[i] < Threshold){SensBits[i] = 0;}
		else if(ResultRaw[i] <= (Threshold - (DeltaVal/50))){SensBits[i] = 0;}

		blkCnt += SensBits[i];	
	}
	if(!((SensBits[0] | SensBits[1]) | (SensBits[12] | SensBits[13]))){blkCnt = 0;}
	for(i = 0;i < 14;i ++){
		if(abw == 0){if(blkCnt >= 8){SensBits[i] = !SensBits[i];}}
		else if(abw == 2){SensBits[i] = !SensBits[i];}
		_SET(&SensByte,(13-i), SensBits[i]);
	}
}
void LineDetectorLib::Detect(uint8_t Mode){
	if(Mode==5){//Lblck
		if		((SensByte&lineLUT[0])>0){LinePos=-30;}
		else if ((SensByte&lineLUT[1])>0){LinePos=-26;}
		else if ((SensByte&lineLUT[2])>0){LinePos=-22;}
		else if ((SensByte&lineLUT[3])>0){LinePos=-18;}
		else if ((SensByte&lineLUT[4])>0){LinePos=-14;}
		else if ((SensByte&lineLUT[5])>0){LinePos=-10;}
		else if ((SensByte&lineLUT[6])>0){LinePos=-7;}
		else if ((SensByte&lineLUT[7])>0){LinePos=-6;}
		else if ((SensByte&lineLUT[8])>0){LinePos=-5;}
		else if ((SensByte&lineLUT[9])>0){LinePos=-4;}
		else if ((SensByte&lineLUT[10])>0){LinePos=-3;}
		else if ((SensByte&lineLUT[11])>0){LinePos=-2;}
		else if ((SensByte&lineLUT[12])>0){LinePos=-1;}
		else if	((SensByte&lineLUT[13])>0){LinePos=0;}
		else if ((SensByte&lineLUT[14])==lineLUT[14]){LinePos=1;}
		else if ((SensByte&lineLUT[15])==lineLUT[15]){LinePos=2;}
		else if ((SensByte&lineLUT[16])==lineLUT[16]){LinePos=3;}
		else if ((SensByte&lineLUT[17])==lineLUT[17]){LinePos=4;}
		else if ((SensByte&lineLUT[18])==lineLUT[18]){LinePos=5;}
		else if ((SensByte&lineLUT[19])==lineLUT[19]){LinePos=6;}
		else if ((SensByte&lineLUT[20])==lineLUT[20]){LinePos=7;}
		else if ((SensByte&lineLUT[21])==lineLUT[21]){LinePos=10;}
		else if ((SensByte&lineLUT[22])==lineLUT[22]){LinePos=14;}
		else if ((SensByte&lineLUT[23])==lineLUT[23]){LinePos=18;}
		else if ((SensByte&lineLUT[24])==lineLUT[24]){LinePos=22;}
		else if ((SensByte&lineLUT[25])==lineLUT[25]){LinePos=26;}
		else if ((SensByte&lineLUT[26])==lineLUT[26]){LinePos=30;}
		else if (SensByte==0){LinePos = 31;}
	}
	else if(Mode==6){//Rblck
		if		((SensByte&lineLUT[26])>0){LinePos=30;}
		else if ((SensByte&lineLUT[25])>0){LinePos=26;}
		else if ((SensByte&lineLUT[24])>0){LinePos=22;}
		else if ((SensByte&lineLUT[23])>0){LinePos=18;}
		else if ((SensByte&lineLUT[22])>0){LinePos=14;}
		else if ((SensByte&lineLUT[21])>0){LinePos=10;}
		else if ((SensByte&lineLUT[20])>0){LinePos=7;}
		else if ((SensByte&lineLUT[19])>0){LinePos=6;}
		else if ((SensByte&lineLUT[18])>0){LinePos=5;}
		else if ((SensByte&lineLUT[17])>0){LinePos=4;}
		else if ((SensByte&lineLUT[16])>0){LinePos=3;}
		else if ((SensByte&lineLUT[15])>0){LinePos=2;}
		else if ((SensByte&lineLUT[14])>0){LinePos=1;}
		else if ((SensByte&lineLUT[13])>0){LinePos=0;}
		else if ((SensByte&lineLUT[12])==lineLUT[12]){LinePos=-1;}
		else if ((SensByte&lineLUT[11])==lineLUT[11]){LinePos=-2;}
		else if ((SensByte&lineLUT[10])==lineLUT[10]){LinePos=-3;}
		else if ((SensByte&lineLUT[9])==lineLUT[9]){LinePos=-4;}
		else if ((SensByte&lineLUT[8])==lineLUT[8]){LinePos=-5;}
		else if ((SensByte&lineLUT[7])==lineLUT[7]){LinePos=-6;}
		else if ((SensByte&lineLUT[6])==lineLUT[6]){LinePos=-7;}
		else if ((SensByte&lineLUT[5])==lineLUT[5]){LinePos=-10;}
		else if ((SensByte&lineLUT[4])==lineLUT[4]){LinePos=-14;}
		else if ((SensByte&lineLUT[3])==lineLUT[3]){LinePos=-18;}
		else if ((SensByte&lineLUT[2])==lineLUT[2]){LinePos=-22;}
		else if ((SensByte&lineLUT[1])==lineLUT[1]){LinePos=-26;}
		else if	((SensByte&lineLUT[0])==lineLUT[0]){LinePos=-30;}
		else if (SensByte==0){LinePos = -31;}
	}
	else if(Mode==3){//Lline
		if		((SensByte&lineLUT[0])>0){LinePos=-30;}
		else if ((SensByte&lineLUT[1])>0){LinePos=-26;}
		else if ((SensByte&lineLUT[2])>0){LinePos=-22;}
		else if ((SensByte&lineLUT[3])>0){LinePos=-18;}
		else if ((SensByte&lineLUT[4])>0){LinePos=-14;}
		else if ((SensByte&lineLUT[5])>0){LinePos=-10;}
		else if ((SensByte&lineLUT[6])>0){LinePos=-7;}
		else if ((SensByte&lineLUT[7])>0){LinePos=-6;}
		else if ((SensByte&lineLUT[8])>0){LinePos=-5;}
		else if ((SensByte&lineLUT[9])>0){LinePos=-4;}
		else if ((SensByte&lineLUT[10])>0){LinePos=-3;}
		else if ((SensByte&lineLUT[11])>0){LinePos=-2;}
		else if ((SensByte&lineLUT[12])>0){LinePos=-1;}
		else if	((SensByte&lineLUT[13])>0){LinePos=0;}
		else if ((SensByte&lineLUT[14])==lineLUT[14]){LinePos=1;}
		else if ((SensByte&lineLUT[15])==lineLUT[15]){LinePos=2;}
		else if ((SensByte&lineLUT[16])==lineLUT[16]){LinePos=3;}
		else if ((SensByte&lineLUT[17])==lineLUT[17]){LinePos=4;}
		else if ((SensByte&lineLUT[18])==lineLUT[18]){LinePos=5;}
		else if ((SensByte&lineLUT[19])==lineLUT[19]){LinePos=6;}
		else if ((SensByte&lineLUT[20])==lineLUT[20]){LinePos=7;}
		else if ((SensByte&lineLUT[21])==lineLUT[21]){LinePos=10;}
		else if ((SensByte&lineLUT[22])==lineLUT[22]){LinePos=14;}
		else if ((SensByte&lineLUT[23])==lineLUT[23]){LinePos=18;}
		else if ((SensByte&lineLUT[24])==lineLUT[24]){LinePos=22;}
		else if ((SensByte&lineLUT[25])==lineLUT[25]){LinePos=26;}
		else if ((SensByte&lineLUT[26])==lineLUT[26]){LinePos=30;}
		else if (SensByte==0){
			if(LinePos == -30){LinePos = -31;}
			else{LinePos = 31;}
		}
	}
	else if(Mode==4){//Rline
		if		((SensByte&lineLUT[26])>0){LinePos=30;}
		else if ((SensByte&lineLUT[25])>0){LinePos=26;}
		else if ((SensByte&lineLUT[24])>0){LinePos=22;}
		else if ((SensByte&lineLUT[23])>0){LinePos=18;}
		else if ((SensByte&lineLUT[22])>0){LinePos=14;}
		else if ((SensByte&lineLUT[21])>0){LinePos=10;}
		else if ((SensByte&lineLUT[20])>0){LinePos=7;}
		else if ((SensByte&lineLUT[19])>0){LinePos=6;}
		else if ((SensByte&lineLUT[18])>0){LinePos=5;}
		else if ((SensByte&lineLUT[17])>0){LinePos=4;}
		else if ((SensByte&lineLUT[16])>0){LinePos=3;}
		else if ((SensByte&lineLUT[15])>0){LinePos=2;}
		else if ((SensByte&lineLUT[14])>0){LinePos=1;}
		else if ((SensByte&lineLUT[13])>0){LinePos=0;}
		else if ((SensByte&lineLUT[12])==lineLUT[12]){LinePos=-1;}
		else if ((SensByte&lineLUT[11])==lineLUT[11]){LinePos=-2;}
		else if ((SensByte&lineLUT[10])==lineLUT[10]){LinePos=-3;}
		else if ((SensByte&lineLUT[9])==lineLUT[9]){LinePos=-4;}
		else if ((SensByte&lineLUT[8])==lineLUT[8]){LinePos=-5;}
		else if ((SensByte&lineLUT[7])==lineLUT[7]){LinePos=-6;}
		else if ((SensByte&lineLUT[6])==lineLUT[6]){LinePos=-7;}
		else if ((SensByte&lineLUT[5])==lineLUT[5]){LinePos=-10;}
		else if ((SensByte&lineLUT[4])==lineLUT[4]){LinePos=-14;}
		else if ((SensByte&lineLUT[3])==lineLUT[3]){LinePos=-18;}
		else if ((SensByte&lineLUT[2])==lineLUT[2]){LinePos=-22;}
		else if ((SensByte&lineLUT[1])==lineLUT[1]){LinePos=-26;}
		else if	((SensByte&lineLUT[0])==lineLUT[0]){LinePos=-30;}
		else if (SensByte==0){
			if(LinePos == 30){LinePos = 31;}
			else{LinePos = -31;}
		}
	}
	else if(Mode == 2){//Sline
		if		((SensByte&lineLUT[0])==lineLUT[0]){LinePos=-30;}
		else if ((SensByte&lineLUT[26])==lineLUT[26]){LinePos=30;}
		else if ((SensByte&lineLUT[1])==lineLUT[1]){LinePos=-26;}
		else if ((SensByte&lineLUT[25])==lineLUT[25]){LinePos=26;}
		else if ((SensByte&lineLUT[2])==lineLUT[2]){LinePos=-22;}
		else if ((SensByte&lineLUT[24])==lineLUT[24]){LinePos=22;}
		else if ((SensByte&lineLUT[3])==lineLUT[3]){LinePos=-18;}
		else if ((SensByte&lineLUT[23])==lineLUT[23]){LinePos=18;}
		else if ((SensByte&lineLUT[4])==lineLUT[4]){LinePos=-14;}
		else if ((SensByte&lineLUT[22])==lineLUT[22]){LinePos=14;}
		else if ((SensByte&lineLUT[5])==lineLUT[5]){LinePos=-10;}
		else if ((SensByte&lineLUT[21])==lineLUT[21]){LinePos=10;}
		else if ((SensByte&lineLUT[6])==lineLUT[6]){LinePos=-7;}
		else if ((SensByte&lineLUT[20])==lineLUT[20]){LinePos=7;}
		else if ((SensByte&lineLUT[7])==lineLUT[7]){LinePos=-6;}
		else if ((SensByte&lineLUT[19])==lineLUT[19]){LinePos=6;}
		else if ((SensByte&lineLUT[8])==lineLUT[8]){LinePos=-5;}
		else if ((SensByte&lineLUT[18])==lineLUT[18]){LinePos=5;}
		else if ((SensByte&lineLUT[9])==lineLUT[9]){LinePos=-4;}
		else if ((SensByte&lineLUT[17])==lineLUT[17]){LinePos=4;}
		else if ((SensByte&lineLUT[10])==lineLUT[10]){LinePos=-3;}
		else if ((SensByte&lineLUT[16])==lineLUT[16]){LinePos=3;}
		else if ((SensByte&lineLUT[11])==lineLUT[11]){LinePos=-2;}
		else if ((SensByte&lineLUT[15])==lineLUT[15]){LinePos=2;}
		else if ((SensByte&lineLUT[12])==lineLUT[12]){LinePos=-1;}
		else if ((SensByte&lineLUT[14])==lineLUT[14]){LinePos=1;}
		else if	((SensByte&lineLUT[13])==lineLUT[13]){LinePos=0;}
		else if (SensByte==0){
			if(LinePos == 30){LinePos = 31;}
			else if(LinePos == -30){LinePos = -31;}
		}
	}
	else if(Mode == 1){//Cline
		if		((SensByte&lineLUT[13])==lineLUT[13]){LinePos=0;}
		else if ((SensByte&lineLUT[12])==lineLUT[12]){LinePos=-1;}
		else if ((SensByte&lineLUT[14])==lineLUT[14]){LinePos=1;}
		else if ((SensByte&lineLUT[11])==lineLUT[11]){LinePos=-2;}
		else if ((SensByte&lineLUT[15])==lineLUT[15]){LinePos=2;}
		else if ((SensByte&lineLUT[10])==lineLUT[10]){LinePos=-3;}
		else if ((SensByte&lineLUT[16])==lineLUT[16]){LinePos=3;}
		else if ((SensByte&lineLUT[9])==lineLUT[9]){LinePos=-4;}
		else if ((SensByte&lineLUT[17])==lineLUT[17]){LinePos=4;}
		else if ((SensByte&lineLUT[8])==lineLUT[8]){LinePos=-5;}
		else if ((SensByte&lineLUT[18])==lineLUT[18]){LinePos=5;}
		else if ((SensByte&lineLUT[7])==lineLUT[7]){LinePos=-6;}
		else if ((SensByte&lineLUT[19])==lineLUT[19]){LinePos=6;}
		else if ((SensByte&lineLUT[6])==lineLUT[6]){LinePos=-7;}
		else if ((SensByte&lineLUT[20])==lineLUT[20]){LinePos=7;}
		else if ((SensByte&lineLUT[5])==lineLUT[5]){LinePos=-10;}
		else if ((SensByte&lineLUT[21])==lineLUT[21]){LinePos=10;}
		else if ((SensByte&lineLUT[4])==lineLUT[4]){LinePos=-14;}
		else if ((SensByte&lineLUT[22])==lineLUT[22]){LinePos=14;}
		else if ((SensByte&lineLUT[3])==lineLUT[3]){LinePos=-18;}
		else if ((SensByte&lineLUT[23])==lineLUT[23]){LinePos=18;}
		else if ((SensByte&lineLUT[2])==lineLUT[2]){LinePos=-22;}
		else if ((SensByte&lineLUT[24])==lineLUT[24]){LinePos=22;}
		else if ((SensByte&lineLUT[1])==lineLUT[1]){LinePos=-26;}
		else if ((SensByte&lineLUT[25])==lineLUT[25]){LinePos=26;}
		else if	((SensByte&lineLUT[0])==lineLUT[0]){LinePos=-30;}
		else if ((SensByte&lineLUT[26])==lineLUT[26]){LinePos=30;}
		else if (SensByte==0){LinePos=7;}
	}
	else{//Nline
		if		((SensByte&lineLUT[13])==lineLUT[13]){LinePos=0;}
		else if ((SensByte&lineLUT[12])==lineLUT[12]){LinePos=-1;}
		else if ((SensByte&lineLUT[14])==lineLUT[14]){LinePos=1;}
		else if ((SensByte&lineLUT[11])==lineLUT[11]){LinePos=-2;}
		else if ((SensByte&lineLUT[15])==lineLUT[15]){LinePos=2;}
		else if ((SensByte&lineLUT[10])==lineLUT[10]){LinePos=-3;}
		else if ((SensByte&lineLUT[16])==lineLUT[16]){LinePos=3;}
		else if ((SensByte&lineLUT[9])==lineLUT[9]){LinePos=-4;}
		else if ((SensByte&lineLUT[17])==lineLUT[17]){LinePos=4;}
		else if ((SensByte&lineLUT[8])==lineLUT[8]){LinePos=-5;}
		else if ((SensByte&lineLUT[18])==lineLUT[18]){LinePos=5;}
		else if ((SensByte&lineLUT[7])==lineLUT[7]){LinePos=-6;}
		else if ((SensByte&lineLUT[19])==lineLUT[19]){LinePos=6;}
		else if ((SensByte&lineLUT[6])==lineLUT[6]){LinePos=-7;}
		else if ((SensByte&lineLUT[20])==lineLUT[20]){LinePos=7;}
		else if ((SensByte&lineLUT[5])==lineLUT[5]){LinePos=-10;}
		else if ((SensByte&lineLUT[21])==lineLUT[21]){LinePos=10;}
		else if ((SensByte&lineLUT[4])==lineLUT[4]){LinePos=-14;}
		else if ((SensByte&lineLUT[22])==lineLUT[22]){LinePos=14;}
		else if ((SensByte&lineLUT[3])==lineLUT[3]){LinePos=-18;}
		else if ((SensByte&lineLUT[23])==lineLUT[23]){LinePos=18;}
		else if ((SensByte&lineLUT[2])==lineLUT[2]){LinePos=-22;}
		else if ((SensByte&lineLUT[24])==lineLUT[24]){LinePos=22;}
		else if ((SensByte&lineLUT[1])==lineLUT[1]){LinePos=-26;}
		else if ((SensByte&lineLUT[25])==lineLUT[25]){LinePos=26;}
		else if	((SensByte&lineLUT[0])==lineLUT[0]){LinePos=-30;}
		else if ((SensByte&lineLUT[26])==lineLUT[26]){LinePos=30;}
		else if (SensByte==0){
			if(LinePos == 30){LinePos = 31;}
			else if(LinePos == -30){LinePos = -31;}
		}
	}
}
#endif