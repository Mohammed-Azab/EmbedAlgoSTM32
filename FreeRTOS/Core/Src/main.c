/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"
#include "FreeRTOS.h"
#include "task.h"
#include "stm32f1xx.h"
#include "queue.h"
#include "timers.h"
#include "semphr.h"
#include "event_groups.h"



void TaskBlinkLED1(void *arg);

void TaskBlinkLED2(void *arg);


int main(){

	RCC ->APB2ENR |= RCC_APB2ENR_IOPBEN | RCC_APB2ENR_IOPAEN;
	GPIOB -> CRH = 0x44442244;
	GPIOA -> CRL = 0x24444444; //using A7 as GND

	xTaskCreate(TaskBlinkLED1,"T1",128,NULL, 1 , NULL);
	xTaskCreate(TaskBlinkLED2,"T2",128,NULL, 1 , NULL);
	vTaskStartScheduler();

}





void TaskBlinkLED1(void *arg){
	while (1){
		GPIOB ->ODR ^= (1 << 10);
		vTaskDelay(pdMS_TO_TICKS(50)); // 10Hz -> 1/10 = 100ms -> delay each 50ms
	}
}

void TaskBlinkLED2(void *arg){
	while (1){
		GPIOB ->ODR ^= (1 << 11);
		vTaskDelay(pdMS_TO_TICKS(25)); // 20Hz -> 1/20 = 50ms -> delay each 20ms
	}
}




