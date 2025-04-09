#include "runball.h"
#include "gpio.h"
#include "tim.h"
#include "motorctrl.h"
#include "data.h"
uint8_t key_sta=0;
uint32_t delaycnt=0;
uint8_t runball_flag=0;
int ReadyAngle=0, FinishAngle=0;
runballDelay RealDelays={
	//.bothopen_delay= 
	50,
	//.upperclose_delay= 
	80,
	//.sideclose_delay= 
	280
};
void runball_Init(void){

    HAL_GPIO_WritePin(runball_gpio,runball_gate, GPIO_PIN_RESET);//upper PE1
    HAL_GPIO_WritePin(runball_gpio,runball_hit, GPIO_PIN_RESET);//side PE0

}


void runball_Getkey(void)
{
	if((data.BUTTON&0x01)==0x00)//key is PG4
		key_sta=1;
	else if((data.BUTTON&0x01)==0x01){
	  key_sta=0;
      delaycnt=0;
    }
}

void runball_ready(){
   if(key_sta==1){
    set_target(2,4,ReadyAngle);
    runball_flag=1;
   }
   if(key_sta==0){
    set_target(2,4,FinishAngle);
    runball_flag=0;
   }

}


void runball_Control(){ 

    if(runball_flag==1){
        delaycnt++;
        if(delaycnt==RealDelays.bothOpen_delay){
            HAL_GPIO_WritePin(runball_gpio ,runball_hit, GPIO_PIN_SET);//upper open
            HAL_GPIO_WritePin(runball_gpio ,runball_gate, GPIO_PIN_SET);//side open
            }

        else if(delaycnt==RealDelays.upperClose_delay)
            HAL_GPIO_WritePin(runball_gpio ,runball_hit, GPIO_PIN_RESET);//upper close
           
        else if(delaycnt==RealDelays.sideClose_delay){
            HAL_GPIO_WritePin(runball_gpio ,runball_gate, GPIO_PIN_RESET);//side close
            delaycnt=0;
            }
      
         }
    if(runball_flag==0){
         HAL_GPIO_WritePin(runball_gpio ,runball_hit, GPIO_PIN_RESET);
         HAL_GPIO_WritePin(runball_gpio ,runball_gate, GPIO_PIN_RESET);
        }
}


