
/* Includes ------------------------------------------------------------------*/
#include "main.h"


void configureIO();
void enableClk();


int main(void){
	HAL_Init();
	enableClk();
	configureIO();



	while (1) {

		GPIOA -> ODR |= (1 << 4);


	}

}



void configureIO(){

	GPIOA -> CRL |= 0x44444448; //A0 input


	/*
	 * Output signal Sequence B9 B7 B4 A15 A9 B15 B13 A4 A6 B11
	 *
	 * A => A4 A6 A9 A15
	 * B => B4 B7 B9 B11 B13 B15
	 * */

	GPIOA -> CRL |= 0x42424444; // A4 A6 output 8MHz
	GPIOA -> CRH |= 0x24444424; // A9 A15 output 8MHz
	GPIOB -> CRL |= 0x24424444; // B4 B7 output 8MHz
	GPIOB -> CRH |= 0x24242424; // B9 B11 B13 B15 output 8MHz

}

void enableClk(){
	RCC -> APB2ENR |= RCC_APB2ENR_IOPAEN;
	RCC -> APB2ENR |= RCC_APB2ENR_IOPBEN;
	RCC -> APB2ENR |= (1 << 9); // enable clk for ADC1
}



