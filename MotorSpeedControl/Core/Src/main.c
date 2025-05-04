
/* Includes ------------------------------------------------------------------*/
#include "main.h"


void configureIO();
void enableClk();
void initADC();
void delay(uint16_t t);
void turnON(uint8_t i);
void turnOFF(uint8_t i);


int main(void){
	enableClk();
	configureIO();


	//initADC();

	//uint16_t prev;
	//uint8_t flag = 1;


	while (1) {

		if (GPIOB -> IDR & (1 << 13)){
			delay(50);
			while (GPIOB -> IDR & (1 << 13)){
				turnON(0);

			}
			turnOFF(0);
		}

		if (GPIOB -> IDR & (1 << 14)){
			delay(50);
			while (GPIOB -> IDR & (1 << 14)){
				turnON(1);

			}
			turnOFF(1);
		}










	}

}



void configureIO(){

	/*
	 *
	 * LEDS => B10 A7 output
	 * Buttons => B13 B14 Input pull down (Active high)
	 *
	 * */

	GPIOA -> CRL = 0x24444444;
	GPIOB -> CRH = 0x48844244;

}

void enableClk(){
	RCC -> APB2ENR |= RCC_APB2ENR_IOPAEN;
	RCC -> APB2ENR |= RCC_APB2ENR_IOPBEN;
	RCC -> APB1ENR |= 0b11; //enable TIM2 & TIM3
	RCC -> APB2ENR |= (1 << 9); // enable clk for ADC1
}


void initADC(){

		ADC1->SQR1 = 0x00000000;  // Reset SQR1 (sequence length = 1 by default)
		ADC1->SQR3 = 0; // Channel 0 first in the sequence
		ADC1 -> SMPR2 &= 0; // reseting the sample time

		ADC1->CR2 |= ADC_CR2_ADON; // ADC on
		for (volatile int i = 0; i < 10000; i++);  // Short delay

		ADC1->CR2 |= ADC_CR2_CAL;          // Start calibration
		while (ADC1->CR2 & ADC_CR2_CAL);   // Wait for calibration to finish

		ADC1->CR2 |= ADC_CR2_EXTTRIG;  // Enable external trigger

		ADC1->CR2 &= ~ADC_CR2_EXTSEL;       // Clear EXTSEL bits
		ADC1->CR2 |= (0b111 << 17);         // Set EXTSEL = 111 for SWSTART

		ADC1->CR2 &= ~ADC_CR2_CONT;  // Clear CONT bit for single conversion mode

		ADC1->CR2 &= ~ADC_CR2_ALIGN;  // 0 = Right alignment (default)

}

void turnON(uint8_t i){


	switch (i){

		case 0: GPIOA ->ODR |= (1 << 7); return;
		case 1: GPIOB ->ODR |= (1 << 10); return;

		default: break;
	}
}

void turnOFF(uint8_t i){


	switch (i){

		case 0: GPIOA ->ODR &= ~(1 << 7); return;
		case 1: GPIOB ->ODR &= ~(1 << 10); return;

		default: break;
	}
}



void delay(uint16_t t){
	TIM3 ->PSC = 8000-1;
	TIM3 ->ARR = t-1;
	TIM3 ->CR1 |= TIM_CR1_CEN;
	while(!(TIM3->SR & TIM_SR_UIF));
	TIM3->SR &= ~TIM_SR_UIF;
	TIM3 ->CR1 &= ~(TIM_CR1_CEN);
}




