
/* Includes ------------------------------------------------------------------*/
#include "main.h"


void configureIO();
void enableClk();


int main(void){
	enableClk();
	configureIO();



	while (1) {

		GPIOB -> ODR |= (1 << 13);
		GPIOB -> ODR |= (1 << 9);
		GPIOB -> ODR |= (1 << 8);
		GPIOB -> ODR |= (1 << 6);
		GPIOB -> ODR |= (1 << 15);
		GPIOB -> ODR |= (1 << 12);
		GPIOB -> ODR |= (1 << 0);
		GPIOB -> ODR |= (1 << 7);
		GPIOB -> ODR |= (1 << 10);
		GPIOB -> ODR |= (1 << 11);


	}

}



void configureIO(){

	/*
	 * Output signal Sequence B9 B8 B7 B6 B15 B13 B12 B0 B10 B11
	 *
	 * B => B0 B6 B7 B8 B9 B10 B11 B12 B13 B15
	 *
	 * */

	GPIOA -> CRL = 0x44444448;  // A0 input
	GPIOB -> CRL = 0x22444442;  // B0 B6 B7 output 8MHz
	GPIOB -> CRH = 0x24222222;  // B8 B9 B10 B11 B12 B13 B15 output 8MHz

}

void enableClk(){
	RCC -> APB2ENR |= RCC_APB2ENR_IOPAEN;
	RCC -> APB2ENR |= RCC_APB2ENR_IOPBEN;
	RCC -> APB2ENR |= (1 << 9); // enable clk for ADC1
}



