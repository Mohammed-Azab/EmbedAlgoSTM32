
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




/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
