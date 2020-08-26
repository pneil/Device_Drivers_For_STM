/*
 * stm32f4xx_spi_driver.h
 *
 *  Created on: Aug 25, 2020
 *      Author: neilpatel
 */

#ifndef INC_STM32F4XX_SPI_DRIVER_H_
#define INC_STM32F4XX_SPI_DRIVER_H_

#include"stm32f407xx.h"

// DEVICE MODE MACRO
#define SPI_DM_MASTER 	1
#define SPI_DM_SLAVE 	0

// DEVICE BUSCONFIGURE MACRO
#define SPI_BUSCONFIG_FD			1
#define SPI_BUSCONFIG_HD			2
#define SPI_BUSCONFIG_SIMPLEX_TX	3
#define SPI_BUSCONFIG_SIMPLEX_RX 	4

// SCLK SPEED MACRO
#define SPI_SPEED2			0
#define SPI_SPEED4			1
#define SPI_SPEED8			2
#define SPI_SPEED16			3
#define SPI_SPEED32			4
#define SPI_SPEED64			5
#define SPI_SPEED128		6
#define SPI_SPEED256		7

// DFF MACRO
#define SPI_DFF_8	0
#define SPI_DFF_16	1

//CPOL MACRO
#define SPI_CPOL_HIGH	1
#define SPI_CPOL_LOW	0

//CPHA MACRO
#define SPI_CPHA_HIGH	1
#define SPI_CPHA_LOW	0

//SSM MACRO
#define SPI_SSM_DI	0
#define SPI_SSM_EN	1

// STATUS FLAG MICRO
#define SPI_TX_FLAG		(1<<1)
#define SPI_RXNE_FLAG 	(1<<0)
#define SPI_BUSY_FLAG	(1<<7)
#define SPI_MODF_FLAG	(1<<5)
#define SPI_OVR_FLAG	(1<<6)
#define SPI_CRCERR_FLAG	(1<<4)
#define SPI_FRE_FLAG	(1<<8)

// SPI STATE MACRO
#define SPI_READY		0
#define SPI_BUSY_IN_TX	2
#define SPI_BUSY_IN_RX	1

// SPI EVENT
#define SPI_EVENT_TX_CMPLT	1
#define SPI_EVENT_RX_CMPLT	2
#define SPI_EVENT_OVR_ERR	3
/*
 * PERIPHERAL DEFINATION FOR SPI
 */

typedef struct
{
	uint8_t SPI_DeviceMode;
	uint8_t SPI_BusConfig;
	uint8_t SPI_SclkSpeed;
	uint8_t SPI_DFF;
	uint8_t SPI_CPOL;
	uint8_t SPI_CPHA;
	uint8_t SPI_SSM;

}SPI_Config_t;

/*
 * Handle struct for SPIx
 */

typedef struct
{
	SPI_RegDef_t  *pSPIx;
	SPI_Config_t  SPIConfig;
	uint8_t *pTxBuffer;
	uint8_t *pRxBuffer;
	uint32_t TxLen;
	uint32_t RxLen;
	uint8_t TxState;
	uint8_t rxState;

}SPI_Handle_t;


/**********************************
 * 								  *
 * 	API supported by  SPI driver  *
 * 								  *
 *********************************/

/*
 * PERIPHERAL Clock Setup
 */
void SPI_Init(SPI_Handle_t  *pSPIHandle);
void SPI_DeInit(SPI_RegDef_t *pSPIx);
void SPI_PeriClockControl(SPI_RegDef_t *pSPIx,uint8_t EnorDi);
void SPI_PeripheralControl(SPI_RegDef_t *pSPIx,uint8_t EnorDi);
void SPI_SSIconfig(SPI_RegDef_t *pSPIx,uint8_t EnorDi);

/*
 * DATA SEND AND RECEIVED
 */
void SPI_SendData(SPI_RegDef_t *pSPIx,uint8_t *pTxBuffer, uint32_t Len);
void SPI_ReceiveData(SPI_RegDef_t *pSPIx,uint8_t *pRxBuffer,uint32_t Len);

uint8_t SPI_SendDataIT(SPI_Handle_t  *pSPIHandle,uint8_t *pTxBuffer, uint32_t Len);
uint8_t SPI_ReceiveDataIT(SPI_Handle_t  *pSPIHandle,uint8_t *pRxBuffer,uint32_t Len);

/*
 * 	IRQ Handling API
 */
void SPI_IRQConfig(uint8_t IRQNumber,uint8_t EnDi);
void SPI_IRQHandling(SPI_Handle_t *pHandle);
void SPI_IRQPriorityConfig(uint8_t IRQNumber,uint8_t IRQPriority);

/*
 * Application call Back
 */
void SPI_ApplicationEventCallback(SPI_Handle_t *pHandle,uint8_t EventNum);
#endif /* INC_STM32F4XX_SPI_DRIVER_H_ */
