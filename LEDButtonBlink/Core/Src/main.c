
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include <stdbool.h>

void configureIO();
void enableClk();
void light(char i);



int main(void){
	HAL_Init();
	enableClk();
	configureIO();
	_Bool pressed = false;
	//GPIOA->ODR &= ~(0x03); // clear output

	char c = 1;


	while (1) {

	  if (GPIOB -> IDR & (1 << 10)){
		  HAL_Delay(50);
		  while (GPIOB -> IDR & (1 << 10));
		  pressed = true;
	  }


	  if (c<=5 && pressed){
		  	  pressed = false;
		  	  if (c==1)
		  		  light(1);
		  	  c++;
	  }

	  else if (c == 6){
		  	  c++;
		  	  light(2);
	  }


   	  }


}



void configureIO(){

	GPIOA -> CRL = 0x44444422; //output 2 MHz A0,A1
	GPIOB -> CRH = 0x44444844; //input pull-down/up. B10
	GPIOB->ODR &= ~(1 << 10);
}

void enableClk(){
	RCC -> APB2ENR |= RCC_APB2ENR_IOPAEN;
	RCC -> APB2ENR |= RCC_APB2ENR_IOPBEN;
}


void light (char i){

	switch (i){
			case 1: GPIOA -> ODR |= 1 << 0;   break; //A0
			case 2: GPIOA -> ODR |= 1 << 1;  break; //A1
			default: break;
		}

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
