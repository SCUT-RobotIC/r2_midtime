#ifndef __runball_H__ 
#define __runball_H__

#include "main.h"

#define runball_gpio GPIOB
#define runball_gate GPIO_PIN_1
#define runball_hit GPIO_PIN_2

extern uint8_t key_sta;
extern uint32_t delaycnt;
extern uint8_t runball_flag;
extern int ReadyAngle, FinishAngle;
typedef struct{
    uint32_t bothOpen_delay;
    uint32_t upperClose_delay;
    uint32_t sideClose_delay;
}runballDelay;

void runball_Init(void);
void runball_Getkey(void);//in HAL_GPIO_EXTI_Callback
void runball_ready(void);
void runball_Control(void);//in HAL_TIM_PeriodElapsedCallback

#endif 

