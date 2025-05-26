
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"
#include "FreeRTOS.h"
#include "task.h"
#include "stm32f1xx.h"

void Task1(void* arg);


int main(){

	xTaskCreate(Task1,"T1",128,NULL, 1 , NULL);
	vTaskStartScheduler();

}





void Task1(void* arg){

}
