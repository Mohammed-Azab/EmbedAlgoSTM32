
/* Includes ------------------------------------------------------------------*/
#include "main.h"




void configureIO();
void enableClk();
void initADC1();
void initADC2();
void delay(uint16_t t);
void turnON(uint8_t i);
void turnOFF(uint8_t i);
void setRotationDir(uint8_t i);
void pressBreak();
void freeMotor();
uint16_t getADCVal(uint8_t i);
void initPWM();
void writePWM (float dutyCycle);
void controlMotor(uint8_t motorIndex, uint8_t dirBit);
int getcurrentPosition();
void CAL();

uint8_t FT = 1 ;
#define TOLERANCE 35



float Kp;
float Ki;
float Kd;
int errDerivative = 0 ;
int errIntegral = 0 ;
int prevErr = 0 ;
int err =0;
void PIDController();
int ref = 0 ;
int curr = 0;
int u;


#define LOWER_LIMIT 1050
#define UPPER_LIMIT 3400






int main(void){
	enableClk();
	configureIO();


	initADC1();
	initADC2();
	initPWM();

	 Kp = 1;
	 Ki = 1.4;
	 Kd = 1.2;


while (1) {



	switch (FT){

		 	 case 0 : break;

		 	 case 1 : freeMotor(); FT =0 ; CAL(); break;

		 	 default : break;
		 }

	 ref = getADCVal(0);

	 curr = getcurrentPosition();

	 if (ref > LOWER_LIMIT ||  ref < UPPER_LIMIT || curr != ref){

		 PIDController();

	 }

	 delay(50);


	 /*
	int val1 = getADCVal(0); // 0 : 4095
	int val2 = getADCVal(1); // ~490 : 3500
	// 3400 from Motor Side to 1050 to the Potentiometer Side gives angle from 0 to 180

	turnON(0);
	  */

}
}



void configureIO(){

	/*
	 *
	 * LEDS => B10 A7 B13 output "2"
	 * ADC => A0 input "0"
	 * ADC => A2 input "0"
	 * PWM => A1 output AF "A"
	 * HBridge IN3 => B7
	 * HBridge IN4 => B8
	 * HBridge ENB => B9
	 * CAL Button => B14
	 *
	 * */

	GPIOA -> CRL = 0x244440A0;
	GPIOB -> CRL = 0x24444444;
	GPIOB -> CRH = 0x48244222;


}

void enableClk(){
	RCC -> APB2ENR |= RCC_APB2ENR_IOPAEN;
	RCC -> APB2ENR |= RCC_APB2ENR_IOPBEN;
	RCC -> APB1ENR |= 0b11; //enable TIM2 & TIM3
	RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;
	RCC -> APB2ENR |= (1 << 9) | (1 << 10); // enable CLK for ADC1
}


void initADC1(){

		ADC1->SQR1 = 0x00000000;  // Reset SQR1 (sequence length = 1 by default)
		ADC1->SQR3 = 0; // Channel 0 first in the sequence
		ADC1 -> SMPR2 &= 0; // reseting the sample time
		ADC1->SMPR2 |= (0b010 << 0); // Channel 0 7.5 cycles

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


void initADC2(){

		ADC2->SQR1 = 0x00000000;  // Reset SQR1 (sequence length = 1 by default)
		ADC2->SQR3 = 2; // Channel 2
		ADC2 -> SMPR2 &= 0; // reseting the sample time
		ADC2->SMPR2 |= (0b010 << 6); // Channel 2 7.5 cycles

		ADC2->CR2 |= ADC_CR2_ADON; // ADC on
		for (volatile int i = 0; i < 10000; i++);  // Short delay

		ADC2->CR2 |= ADC_CR2_CAL;          // Start calibration
		while (ADC2->CR2 & ADC_CR2_CAL);   // Wait for calibration to finish

		ADC2->CR2 |= ADC_CR2_EXTTRIG;  // Enable external trigger

		ADC2->CR2 &= ~ADC_CR2_EXTSEL;       // Clear EXTSEL bits
		ADC2->CR2 |= (0b111 << 17);         // Set EXTSEL = 111 for SWSTART

		ADC2->CR2 &= ~ADC_CR2_CONT;  // Clear CONT bit for single conversion mode

		ADC2->CR2 &= ~ADC_CR2_ALIGN;  // 0 = Right alignment (default)

}

void turnON(uint8_t i){


	switch (i){

		case 0: GPIOA ->ODR |= (1 << 7);  return; // Blue indicates reached goal
		case 1: GPIOB ->ODR |= (1 << 10); return; // Red indicates still processing
		case 2: GPIOB ->ODR |= (1 << 13); return; // Green indicates CAL for first time

		default: break;
	}
}

void turnOFF(uint8_t i){


	switch (i){

		case 0: GPIOA ->ODR &= ~(1 << 7);  return;
		case 1: GPIOB ->ODR &= ~(1 << 10); return;
		case 2: GPIOB ->ODR &= ~(1 << 13); return;

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

uint16_t getADCVal(uint8_t i){
	if (i ==0){
		ADC1->CR2 |= ADC_CR2_SWSTART; // start conversion
			while (!(ADC1->SR & ADC_SR_EOC));     // Wait for conversion complete
			ADC1->SR &= ~(ADC_SR_EOC);
			uint16_t ADCVal = ADC1->DR;  // Read result (10-bit mask)
			return ADCVal;
	}
	else {
		ADC2->CR2 |= ADC_CR2_SWSTART; // start conversion
		while (!(ADC2->SR & ADC_SR_EOC));     // Wait for conversion complete
		ADC2->SR &= ~(ADC_SR_EOC);
		uint16_t ADCVal = ADC2->DR;  // Read result (10-bit mask)
		return ADCVal;
	}


}

void setRotationDir(uint8_t i){
	switch(i){
		case 0 : GPIOB -> ODR |= (1 << 7) ;break; // Counter CLockwise
		case 1 : GPIOB -> ODR |= (1 << 8) ;break; // CLockwise
		default: break;
	}
}


void pressBreak(){
	GPIOB->ODR |= (1 << 7) | (1 << 8);

}


void freeMotor(){
	GPIOB->ODR &= ~((1 << 7) | (1 << 8));
}

void initPWM(){
	RCC ->APB1ENR |= RCC_APB1ENR_TIM2EN; //enable Timer

	TIM2 -> PSC = 8 - 1;
	TIM2 -> ARR = 1000 - 1;

	TIM2->CCMR1 |= (6 << 12); // mode 1 for ch2

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
	while (GPIOB -> IDR & (1 << dirBit)){
		turnON(motorIndex);
		setRotationDir(motorIndex);
		uint16_t ADCVal = getADCVal(0);
		float dutyCycle = ((float)ADCVal * 100.0f) / 4095.0f;
		writePWM(dutyCycle);
	}
	turnOFF(motorIndex);
}

int getcurrentPosition(){

	return getADCVal(1);

}

void CAL(){
	while (!(GPIOB -> IDR & 1 << 14) ){ // exit CAL
		delay(50);
		if (getADCVal(1) != getADCVal(0)){
			turnOFF(2);
		}
		else {
			turnON(2);
		}
	}
}


// PID -> u(t) = Kp * E(t) + Ki * ∫E(t)dt + Kd * dE(t)/dt

void PIDController(){
	uint8_t count = 0;
    while ((abs(ref - curr) > TOLERANCE && count++ < 200)){

        curr = getcurrentPosition();

        // Enforce limits
        if ((curr <= LOWER_LIMIT && (ref - curr) < 0) ||
            (curr >= UPPER_LIMIT && (ref - curr) > 0) || curr < LOWER_LIMIT ||  curr > UPPER_LIMIT) {
            pressBreak(); // Immediate stop
            turnON(1);     // RED LED: indicates Dead END
            turnOFF(0);    // Turn off Blue LED
            return;
        }

        if (errIntegral > 1000) errIntegral = 1000;
        else if (errIntegral < -1000) errIntegral = -1000;


        turnON(1); // RED for processing
        turnOFF(0);

        err = ref - curr;
        errIntegral += err;
        errDerivative = err - prevErr;
        prevErr = err;

        if (abs(err) <= TOLERANCE) break;

        u = Kp * err + Ki * errIntegral + Kd * errDerivative;

        float duty = fabs(u);  // make sure it's positive
        duty = duty > 100.0f?  100.0f : duty;

        freeMotor();

        if (u > 0) {
            setRotationDir(1); //  clockwise -> counter-clockwise and vice versa
        } else {
            setRotationDir(0); // 	counter-clockwise
        }

        writePWM(duty);
    }

    pressBreak(); // stop motor when done
    turnON(0); // Blue for finishing
    turnOFF(1);
}

// Motor goes CLKWISE then the Indicator goes anti CLKWISE because of the gearing





