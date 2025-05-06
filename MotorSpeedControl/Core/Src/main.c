
/* Includes ------------------------------------------------------------------*/
#include "main.h"


void configureIO();
void enableClk();
void initADC();
void delay(uint16_t t);
void turnON(uint8_t i);
void turnOFF(uint8_t i);
void setRotationDir(uint8_t i);
void rotateMax(uint16_t pwm);
void pressBreak();
void freeMotor();
uint16_t getADCVal();
void initPWM();
void writePWM (float dutyCycle);
void controlMotor(uint8_t motorIndex, uint8_t dirBit)


int main(void){
	enableClk();
	configureIO();


	initADC();
	initPWM();


	while (1) {

		freeMotor();



		if (GPIOB -> IDR & (1 << 13) && GPIOB -> IDR & (1 << 14)){ // break
			pressBreak();
			continue;

		}


		if (GPIOB->IDR & (1 << 13)) controlMotor(0, 13); // CCW
		if (GPIOB->IDR & (1 << 14)) controlMotor(1, 14); // CW



	}

}



void configureIO(){

	/*
	 *
	 * LEDS => B10 A7 output "2"
	 * Buttons => B13 B14 Input pull down (Active high) "8"
	 * ADC => A0 input "0"
	 * PWM => A1 output AF "A"
	 * HBridge IN3 => B7
	 * HBridge IN4 => B8
	 * HBridge ENB => B9
	 *
	 * */

	GPIOA -> CRL = 0x244444A0;
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
		case 0 : GPIOB -> ODR |= (1 << 7) ;break; // Counter CLockwise
		case 1 : GPIOB -> ODR |= (1 << 8) ;break; // CLockwise
		default: break;
	}
}


void pressBreak(){
	GPIOB -> ODR |= (11 << 7) ;

}

void rotateMax(uint16_t pwm){

	GPIOB -> ODR |= (1 << 9) ;
}

void freeMotor(){
	GPIOB -> ODR &= ~(11 << 7);
}

void initPWM(){
	RCC ->APB1ENR |= RCC_APB1ENR_TIM2EN; //enable Timer

	TIM2 -> PSC = 8 - 1;
	TIM2 -> ARR = 1000 - 1;

	TIM2->CCMR1 |= (7 << 12); // mode 2 for ch2

	TIM2 -> CCR2 = 0; //duty Cycle =0

	TIM2->CCMR1 |= TIM_CCMR1_OC1PE; // Enable preload

	TIM2->CCER |= TIM_CCER_CC2E;    // Enable output for CH1

	TIM2->CR1 |= TIM_CR1_ARPE;   // Enable auto-reload Preload

	TIM2->EGR |= TIM_EGR_UG;     // Force update to load registers

	TIM2->CR1 |= TIM_CR1_CEN;    // Enable timer

}

void writePWM (float dutyCycle){
	if (dutyCycle > 100){
		dutyCycle = 100;
	}

	TIM2 -> CCR2 = (TIM2->ARR + 1) * dutyCycle / 100;

}

void controlMotor(uint8_t motorIndex, uint8_t dirBit){
	delay(50);
	while (GPIOB -> IDR & (1 << dirBit)){
		turnON(motorIndex);
		setRotationDir(motorIndex);
		uint16_t ADCVal = getADCVal();
		float dutyCycle = ((float)ADCVal * 100.0f) / 4095.0f;
		writePWM(dutyCycle);
	}
	turnOFF(motorIndex);
}


/*
 * ⚙️ Steps to Configure PWM in Bare-Metal STM32

`	Assuming Timer2 and a pin like PA0:

    1- Enable GPIOA and TIM2 clocks.

    2- Configure PA0 as alternate function (AF mode).

    3- Set timer PSC and ARR for desired frequency.

    4- Set CCRx for duty cycle (e.g., CCR1 for channel 1).

    5- Set PWM mode in TIMx_CCMR1 (PWM Mode 1 or 2).

    6- Enable output in TIMx_CCER (channel enable).

    7- Start the timer by enabling the counter in TIMx_CR1.
 */




