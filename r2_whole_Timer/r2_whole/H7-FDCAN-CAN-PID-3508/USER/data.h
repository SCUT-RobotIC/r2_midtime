#ifndef __DATA_H__
#define __DATA_H__
#include "stdint.h"
#define DATA_FRAME_SIZE		14
typedef struct{
		int16_t LX;
		int16_t LY;
		int16_t RX;
	  int16_t RY;
		int16_t BUTTON;
		int16_t SWITCH;
		int16_t KNOB;
		uint8_t STATE;
}receiveData;

extern receiveData data;
uint8_t update_remotedata(uint8_t *buf);

#endif
