
/* Includes ------------------------------------------------------------------*/
#include "main.h"


void configureIO();
void enableClk();
void initADC();


int main(void){
	enableClk();
	configureIO();


	initADC();

	while (1) {


		ADC1->CR2 |= ADC_CR2_SWSTART; // start conversion
		while (!(ADC1->SR & ADC_SR_EOC));     // Wait for conversion complete
		uint16_t adcVal = ADC1->DR & 0x3FF;  // Read result (10-bit mask)

		/*
		 *  Why & 0x3FF?
		 *  0x3FF in hexadecimal = 1023 in decimal = 0b0000001111111111 in binary (10 bits set to 1).
		 *  This mask ensures you only keep the lowest 10 bits and discard any upper garbage bits that might accidentally exist in ADC1->DR.
		 *  It's a way to guarantee you're reading a clean 10-bit result
		 *
		 * */



		/*
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
		*/


	}

}



void configureIO(){

	/*
	 * Output signal Sequence B9 B8 B7 B6 B15 B13 B12 B0 B10 B11
	 *
	 * B => B0 B6 B7 B8 B9 B10 B11 B12 B13 B15
	 *
	 * */

	GPIOA -> CRL = 0x44444440;  // A0 input Analog
	GPIOB -> CRL = 0x22444442;  // B0 B6 B7 output 8MHz
	GPIOB -> CRH = 0x24222222;  // B8 B9 B10 B11 B12 B13 B15 output 8MHz

}

void enableClk(){
	RCC -> APB2ENR |= RCC_APB2ENR_IOPAEN;
	RCC -> APB2ENR |= RCC_APB2ENR_IOPBEN;
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

		ADC1->SQR3 = 0;  // Channel 0 → first in regular sequence

		ADC1->CR2 &= ~ADC_CR2_CONT;  // Clear CONT bit for single conversion mode

		ADC1->CR2 &= ~ADC_CR2_ALIGN;  // 0 = Right alignment (default)

}



