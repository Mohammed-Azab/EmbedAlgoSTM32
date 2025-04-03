/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include <math.h>



void enableClk();
void configureIO();
void delay(int t);



int main(void) {

	enableClk();
	configureIO();

  while (1) {

	  if (GPIOB ->IDR & (1<<10)){ //Active High

		  delay(60);

		  if (GPIOB ->IDR & (1<<10)){

			  while (-1){
			  			  GPIOA -> ODR ^= (1 << 1);
			  			  delay(100);
			  		  }

		  }


	  }
	  else if (!(GPIOB ->IDR & (1<<11))){ //Active low

		  delay(60);

		  if (!(GPIOB ->IDR & (1<<11))){

			  while (-1){
		  			  GPIOA -> ODR ^= (1 << 1);
		  			  delay(500);
		  		  }
		  }

	  }


  }

}

void enableClk(){

	RCC -> APB2ENR |= RCC_APB2ENR_IOPAEN;
	RCC -> APB2ENR |= RCC_APB2ENR_IOPBEN;
	RCC -> APB1ENR |= RCC_APB1ENR_TIM2EN;

}
void configureIO(){

	GPIOB -> CRH |= 0x44448844; //B10 & B10 input
	GPIOA -> CRL |= 0x444444A4; //A1 Alternating Function output 2MHz
	GPIOB -> ODR |= (1<<11); //B11 Pull Up // B10 Pull down by default

}
void delay(int t){
	TIM2 ->PSC = 8000-1;
	int ARRValP1 = ((t/1000) * 8 * pow(10,6)) / 8000 ;
	TIM2 ->ARR = ARRValP1-1;

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
