/*
 * bsp.h
 *
 *  Created on: 3 abr. 2022
 *      Author: agust
 */

#ifndef BSP_H_
#define BSP_H_

#include "stdint.h"

/** @defgroup STM32F4_DISCOVERY_LOW_LEVEL_Exported_Types STM32F4 DISCOVERY LOW LEVEL_Exported_Types
  * @{
  */
typedef enum
{
  LED_GREEN = 0,
  LED_ORANGE = 1,
  LED_RED = 2,
  LED_BLUE = 3
} Led_TypeDef;

typedef enum
{
  BUTTON_KEY = 0,
} Button_TypeDef;


/**
  * @}
  */


/** @defgroup STM32F4_DISCOVERY_LOW_LEVEL_Exported_Functions STM32F4 DISCOVERY LOW LEVEL Exported Functions
  * @{
  */
uint32_t	BSP_GetVersion(void);
void 		BSP_Init(void);
void     	BSP_LED_On(Led_TypeDef Led);
void     	BSP_LED_Off(Led_TypeDef Led);
void     	BSP_LED_Toggle(Led_TypeDef Led);
uint32_t 	BSP_PB_GetState(Button_TypeDef Button);
void		BSP_Delay(uint32_t ms);

/**
  * @}
  */



#endif /* BSP_H_ */
