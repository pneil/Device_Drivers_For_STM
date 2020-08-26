/*
 * stm32f407xx_gpio_driver.h
 *
 *  Created on: Jul 7, 2020
 *      Author: neilpatel
 */

#ifndef INC_STM32F407XX_GPIO_DRIVER_H_
#define INC_STM32F407XX_GPIO_DRIVER_H_

#include"stm32f407xx.h"

/*
 *  THIS IS A HANDLE STRUCTURE  FOR A GPIO PIN
 */
typedef struct
{
	uint8_t GPIO_PinNumber;
	uint8_t GPIO_PinMode;
	uint8_t GPIO_PinSpeed;
	uint8_t	GPIO_PinPuPdControl;
	uint8_t GPIO_PinOpType;
	uint8_t GPIO_PinAltFunMode;

}GPIO_PinConfig_t;


#define GPIO_MODE_IN 		0
#define GPIO_MODE_OUT		1
#define GPIO_MODE_ALTFEN	2
#define GPIO_MODE_ANALOG	3
#define GPIO_MODE_IT_FT		4
#define GPIO_MODE_IT_RT		5
#define GPIO_MODE_IT_RFT	6

#define GPIO_OP_TYPE_PP		0
#define GPIO_OP_TYPE_OP		1

#define GPIO_SPEED_LOW 		0
#define GPIO_SPEED_MEDIUM	1
#define GPIO_SPEED_FAST		2
#define GPIO_SPEED_HIGH		3

#define GPIO_NO_PUPD		0
#define GPIO_PU				1
#define GPIO_PD				2

#define GPIO_PIN_NO_0 	0
#define GPIO_PIN_NO_1 	1
#define GPIO_PIN_NO_2 	2
#define GPIO_PIN_NO_3 	3
#define GPIO_PIN_NO_4 	4
#define GPIO_PIN_NO_5 	5
#define GPIO_PIN_NO_6 	6
#define GPIO_PIN_NO_7 	7
#define GPIO_PIN_NO_8 	8
#define GPIO_PIN_NO_9 	9
#define GPIO_PIN_NO_10 10
#define GPIO_PIN_NO_11 11
#define GPIO_PIN_NO_12 12
#define GPIO_PIN_NO_13 13
#define GPIO_PIN_NO_14 14
#define GPIO_PIN_NO_15 15


typedef struct
{
	//pointer to  hold the base add
	GPIO_RegDef_t *pGPIOx;
	GPIO_PinConfig_t GPIO_PinConfig;

}GPIO_Handle_t;

/*********************************************************
*
*		APIs supported  by GPIO driver
*
**********************************************************/
void GPIO_Init(GPIO_Handle_t  *pGPIOHandle);
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx);


void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx,uint8_t EnorDi);

uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx);
void GPIO_WrtieToOutputPin(GPIO_RegDef_t *pGPIOx,uint8_t PinNumber,uint8_t value);
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx,uint16_t value);
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx,uint8_t PinNumber);

void GPIO_IRQConfig(uint8_t IRQNumber,uint8_t EnDi);
void GPIO_IRQHandling(uint8_t PinNumber);
void GPIO_IRQPriorityConfig(uint8_t IRQNumber,uint8_t IRQPriority);
#endif /* INC_STM32F407XX_GPIO_DRIVER_H_ */
