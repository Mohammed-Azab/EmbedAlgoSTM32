
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include <math.h>


void configureIO();
void enableClk();
void delay(int t);




int main(void){
	HAL_Init();
	enableClk();
	configureIO();



	while (1) {


		if (GPIOB -> IDR & (1 << 10)){ //Active High

			while (-1){
				GPIOA -> ODR ^= 1 << 1;
				delay(100);
				if (!(GPIOB ->IDR & (1<<11))){
					break;
				}
			}

		}

		else if (!(GPIOB ->IDR & (1<<11))){ //Active low

			while (-1){
				GPIOA -> ODR ^= 1 << 1;
				delay(500);
				if (GPIOB -> IDR & (1 << 10)){
					break;
				}

		}



	}
}
}



void configureIO(){

	GPIOA -> CRL = 0x44444424; //output 2 MHz A1
	//GPIOA -> CRL |= 0x444444A4; //A1 Alternating Function output 2MHz
	GPIOB -> CRH = 0x44448844; //input pull-down/up. B10, B11
	GPIOB->ODR |= (1 << 11); // B11 Pull UP
	GPIOB->ODR &= ~(1 << 10); // B10 pull down
}

void enableClk(){
	RCC -> APB2ENR |= RCC_APB2ENR_IOPAEN;
	RCC -> APB2ENR |= RCC_APB2ENR_IOPBEN;
	RCC -> APB1ENR |= RCC_APB1ENR_TIM2EN;
}

void delay(int t){
	TIM2 ->PSC = 8000-1;
	//int ARRValP1 = ((t) * 8 * pow(10,3)) / 8000 ;
	TIM2 ->ARR = t-1;
	TIM2 ->CR1 |= TIM_CR1_CEN; //Enable the counter
	while(!(TIM2->SR & TIM_SR_UIF)); //Waiting until finishing counting
	TIM2->SR &= ~TIM_SR_UIF; //reset the flag

}



