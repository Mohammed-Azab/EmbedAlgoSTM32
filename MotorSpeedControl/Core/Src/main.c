
/* Includes ------------------------------------------------------------------*/
#include "main.h"


void configureIO();
void enableClk();
void initADC();
void delay(uint16_t t);
void turnON(uint8_t i);
void turnOFF(uint8_t i);
void setRotationDir(uint8_t i);
void rotate(uint16_t pwm);
void pressBreak();
uint16_t getADCVal();


int main(void){
	enableClk();
	configureIO();


	initADC();

	//uint16_t prev;
	//uint8_t flag = 1;


	while (1) {





		if (GPIOB -> IDR & (1 << 13) && GPIOB -> IDR & (1 << 14)){ // break
			pressBreak();
			continue;

		}


		if (GPIOB -> IDR & (1 << 13)){
			delay(50);
			while (GPIOB -> IDR & (1 << 13)){
				turnON(0);
				setRotationDir(0);
				rotate(1);
//				if (getADCVal() >=50){
//					turnON(0);
//					rotate();
//				}
//				else {
//					turnOFF(0);
//				}

			}
			turnOFF(0);

		}

		if (GPIOB -> IDR & (1 << 14)){
			delay(50);
			while (GPIOB -> IDR & (1 << 14)){
				turnON(1);
				setRotationDir(1);
				rotate(1);
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
	 * ADC => A0 input
	 * HBridge IN3 => B7
	 * HBridge IN4 => B8
	 * HBridge ENB => B9
	 *
	 * */

	GPIOA -> CRL = 0x24444440;
	GPIOB -> CRL = 0x24444444;
	GPIOB -> CRH = 0x48844222;


}

void enableClk(){
	RCC -> APB2ENR |= RCC_APB2ENR_IOPAEN;
	RCC -> APB2ENR |= RCC_APB2ENR_IOPBEN;
	RCC -> APB1ENR |= 0b11; //enable TIM2 & TIM3
	RCC -> APB2ENR |= (1 << 9); // enable CLK for ADC1
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

uint16_t getADCVal(){
	ADC1->CR2 |= ADC_CR2_SWSTART; // start conversion
	while (!(ADC1->SR & ADC_SR_EOC));     // Wait for conversion complete
	ADC1->SR &= ~(ADC_SR_EOC);
	uint16_t ADCVal = ADC1->DR & 0x3FF;  // Read result (10-bit mask)
	return ADCVal;

	/*
	 *  Why & 0x3FF?
	 *
	 *  0x3FF in hexadecimal = 1023 in decimal = 0b0000001111111111 in binary (10 bits set to 1).
	 *  This mask ensures you only keep the lowest 10 bits and discard any upper garbage bits that might accidentally exist in ADC1->DR.
	 *  It's a way to guarantee you're reading a clean 10-bit result
	 *
     * */
}

void setRotationDir(uint8_t i){
	switch(i){
		case 0 : GPIOB -> ODR |= (1 << 7) ;break;
		case 1 : GPIOB -> ODR |= (1 << 8) ;break;
		default: break;
	}
}


void pressBreak(){
	GPIOB -> ODR |= (11 << 7) ;

}

void rotate(uint16_t pwm){

	GPIOB -> ODR |= (1 << 9) ;
}




