
/* Includes ------------------------------------------------------------------*/
#include "main.h"


void configureIO();
void enableClk();


int main(void){
	HAL_Init();
	enableClk();
	configureIO();



	while (1) {




	}

}



void configureIO(){

}

void enableClk(){
	RCC -> APB2ENR |= RCC_APB2ENR_IOPAEN;
	RCC -> APB2ENR |= RCC_APB2ENR_IOPBEN;
	RCC -> APB1ENR |= RCC_APB1ENR_TIM2EN;
}



