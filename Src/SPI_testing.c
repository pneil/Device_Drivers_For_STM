/*
 * SPI_testing.c
 *
 *  Created on: Aug 26, 2020
 *      Author: neilpatel
 */
//SS2
//PB14 -->MISO
//PB15 -->MOSI
//PB13 -->SCLK
//PB12 -->NSS
//ATL Function mode 5

// Debugging and  Testing of Developed driver were done using USB Logic Analyzer

#include "stm32f407xx.h"
#include<string.h>

void SPI2_GPIOInit(void)
{
	GPIO_Handle_t SPIPins;

	SPIPins.pGPIOx = GPIOB;
	SPIPins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFEN;
	SPIPins.GPIO_PinConfig.GPIO_PinAltFunMode = 5;
	SPIPins.GPIO_PinConfig.GPIO_PinOpType = GPIO_OP_TYPE_PP;
	SPIPins.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_13;
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_13;
	GPIO_Init(&SPIPins);
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_15;
	GPIO_Init(&SPIPins);
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_14;
	GPIO_Init(&SPIPins);
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
	GPIO_Init(&SPIPins);
}

void SPI2_Init()
{

	SPI_Handle_t SPI2Handle;

	SPI2Handle.pSPIx = SPI2;
	SPI2Handle.SPIConfig.SPI_BusConfig = SPI_BUSCONFIG_FD;
	SPI2Handle.SPIConfig.SPI_DeviceMode = SPI_DM_MASTER;
	SPI2Handle.SPIConfig.SPI_SclkSpeed = SPI_SPEED2;
	SPI2Handle.SPIConfig.SPI_DFF = SPI_DFF_8;
	SPI2Handle.SPIConfig.SPI_CPOL = SPI_CPOL_LOW;
	SPI2Handle.SPIConfig.SPI_CPHA = SPI_CPHA_LOW;
	SPI2Handle.SPIConfig.SPI_SSM = SPI_SSM_EN;

	SPI_Init(&SPI2Handle);

}
int main2(void)
{
	char userdata[] ="Hello World";
	SPI2_GPIOInit();
	SPI2_Init();
	SPI_SSIconfig(SPI2,ENABLE);
	//Enable SPI
	SPI_PeripheralControl(SPI2,ENABLE);
	SPI_SendData(SPI2,(uint8_t *)userdata,strlen(userdata));
	SPI_PeripheralControl(SPI2,DISABLE);

	while(1);
	return 0;
}
