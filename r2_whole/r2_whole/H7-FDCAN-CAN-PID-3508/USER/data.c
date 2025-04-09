#include "data.h"
receiveData data;
uint8_t update_remotedata(uint8_t *buf){
	if (buf[13] == 0xB2&&buf[0]==0x2B)
    {
    data.STATE = 1;
    data.LX = (int16_t)(((uint16_t)buf[4] << 8) | buf[3]);
		if(data.LX<=5000&&data.LX>=-5000) data.LX=0;
    data.LY = (int16_t)(((uint16_t)buf[6] << 8) | buf[5]);
    if(data.LY<=5000&&data.LY>=-5000) data.LY=0;
		data.RX = (int16_t)(((uint16_t)buf[8] << 8) | buf[7]);
		if(data.RX<=5000&&data.RX>=-5000) data.RX=0;
		data.RY = (int16_t)(((uint16_t)buf[10] << 8) | buf[9]);
    if(data.RY<=5000&&data.RY>=-5000) data.RY=0;
		data.BUTTON = (int16_t)buf[11] & 0x000F;
		data.SWITCH = (int16_t)buf[12] & 0x000F;
		data.KNOB		=	(int16_t)buf[12] & 0x00F0;
    return 1;
    }
    else 
    {
        data.STATE = 0;
        return 0;
    }
}
