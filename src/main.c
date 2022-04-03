/**
  ******************************************************************************
  * @file    main.c
  * @author  Ac6
  * @version V1.0
  * @date    01-December-2013
  * @brief   Default main function.
  ******************************************************************************
*/


#include "bsp.h"
			

int main(void)
{

	BSP_Init();

	for(;;){
		BSP_LED_Toggle(LED_RED);
		BSP_Delay(500);
	}
}
