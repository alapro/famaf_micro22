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
		BSP_LED_Blinky(LED_RED, 2, 200, 500);
		BSP_LED_Blinky(LED_BLUE, 3, 500, 500);
		BSP_Delay(5000);
	}
}
