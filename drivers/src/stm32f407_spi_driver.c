/*
 * stm32f407_spi_driver.c
 *
 *  Created on: Aug 25, 2020
 *      Author: neilpatel
 */
/*
 * Peripharal Clock Setup
 */

#include "stm32f4xx_spi_driver.h"

void SPI_Init(SPI_Handle_t  *pSPIHandle)
{
	//FIRST CONFIG FOR SPI_CR1
	uint32_t temp = 0;

	// SSIx Clock Configuration
	SPI_PeriClockControl(pSPIHandle->pSPIx,ENABLE);

	temp |= pSPIHandle->SPIConfig.SPI_DeviceMode << 2;

	//BUS config

	if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUSCONFIG_FD)
	{
		temp &= ~(1<<15);
	}
	else if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUSCONFIG_HD)
	{
		temp |= (1<<15);
	}
	else if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUSCONFIG_SIMPLEX_RX)
	{
		temp &= ~(1<<15);
		temp |= (1<<10);
	}
	temp |= pSPIHandle->SPIConfig.SPI_SclkSpeed << 3;
	temp |= pSPIHandle->SPIConfig.SPI_DFF << 11;
	temp |= pSPIHandle->SPIConfig.SPI_CPOL << 1;
	temp |= pSPIHandle->SPIConfig.SPI_CPHA  << 0;

	pSPIHandle->pSPIx->CR1 = temp;
}

void SPI_DeInit(SPI_RegDef_t *pSPIx)
{
	if(pSPIx == SPI1)
	{
		SPI1_REG_RESET();
	}
	else if(pSPIx == SPI2)
	{
		SPI2_REG_RESET();
	}
	else if(pSPIx == SPI3)
	{
		SPI3_REG_RESET();
	}
}
void SPI_PeriClockControl(SPI_RegDef_t *pSPIx,uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		if(pSPIx == SPI1)
		{
			SPI1_PCLK_EN();
		}
		else if(pSPIx == SPI2)
		{
			SPI2_PCLK_EN();
		}
		else if(pSPIx == SPI3)
		{
			SPI3_PCLK_EN();
		}
	}
	else
	{
		{
			if(pSPIx == SPI1)
			{
				SPI1_PCLK_DI();
			}
			else if(pSPIx == SPI2)
			{
				SPI2_PCLK_DI();
			}
			else if(pSPIx == SPI3)
			{
				SPI3_PCLK_DI();
			}
	}
	}
}

uint8_t SPI_GetFlagStatus(SPI_RegDef_t  *pSPIx, uint32_t FlagName)
{
	if(pSPIx->SR & FlagName)
	{
		return FLAG_SET;
	}
	return  FLAG_RESET;
}

/*
 * DATA SEND AND RECEIVED
 */

void SPI_SendData(SPI_RegDef_t *pSPIx,uint8_t *pTxBuffer, uint32_t Len)
{
	while(Len > 0)
	{
		//Wait until TXE is set
		while(SPI_GetFlagStatus(pSPIx,SPI_TX_FLAG)  ==  FLAG_RESET);

		//Check the dff bit
		if(pSPIx->CR1 & (1<<11))
		{
			// 16  bit dff
			pSPIx->DR = *((uint16_t *)pTxBuffer);
			Len--;
			Len--;
			(uint16_t *)pTxBuffer++;
		}else
		{
			//8 bit dff
			pSPIx->DR = *(pTxBuffer);
			Len--;
			pTxBuffer++;
		}
	}
}

void SPI_PeripheralControl(SPI_RegDef_t *pSPIx,uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{	// ENABLE SPIx
		pSPIx->CR1  |= (1<<6);
	}
	else
	{	// DISABLE SPIx
		pSPIx->CR1  &= ~(1<<6);
	}
}

void SPI_SSIconfig(SPI_RegDef_t *pSPIx,uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{	// ENABLE THE SSI
		pSPIx->CR1  |= (1<<8);
	}
	else
	{	// DISABLE THE SSI
		pSPIx->CR1  &= ~(1<<8);
	}
}
void SPI_ReceiveData(SPI_RegDef_t *pSPIx,uint8_t *pRxBuffer,uint32_t Len)
{
	while(Len > 0)
	{
		//Wait until TXE is set
		while(SPI_GetFlagStatus(pSPIx,SPI_RXNE_FLAG)  ==  FLAG_RESET);

		//Check the dff bit
		if(pSPIx->CR1 & (1<<11))
		{
			// 16  bit dff
			*((uint16_t *)pRxBuffer) =  pSPIx->DR;
			Len--;
			Len--;
			(uint16_t *)pRxBuffer++;
		}else
		{
			//8 bit dff
			*(pRxBuffer) =pSPIx->DR;
			Len--;
			pRxBuffer++;
		}
	}
}

uint8_t SPI_SendDataIT(SPI_Handle_t  *pSPIHandle,uint8_t *pTxBuffer, uint32_t Len)
{
	uint8_t state = pSPIHandle->TxState;
	if(state != SPI_BUSY_IN_TX)
	{
	// Save TX buffer address  and len information
	pSPIHandle->pTxBuffer =  pTxBuffer;
	pSPIHandle->TxLen = Len;

	//	Mark SPI as Busy
	pSPIHandle->TxState = SPI_BUSY_IN_TX;

	//	Enable Interrupt flag - TXEIE
	pSPIHandle->pSPIx->CR2 |= (1<<7);
	}
	return state;

}
uint8_t SPI_ReceiveDataIT(SPI_Handle_t  *pSPIHandle,uint8_t *pRxBuffer,uint32_t Len)
{
	uint8_t state = pSPIHandle->TxState;
	if(state != SPI_BUSY_IN_RX)
	{
	// Save RX buffer address  and len information
	pSPIHandle->pRxBuffer =  pRxBuffer;
	pSPIHandle->RxLen = Len;

	//	Mark SPI as Busy
	pSPIHandle->rxState = SPI_BUSY_IN_RX;

	//	Enable Interrupt flag - RXEIE
	pSPIHandle->pSPIx->CR2 |= (1<<6);
	}
	return state;
}
/*
* 	IRQ Handling API
*/

void SPI_IRQConfig(uint8_t IRQNumber,uint8_t EnDi)
{
	if(EnDi == ENABLE)
	{
		if(IRQNumber <= 31)
		{
			*NVIC_ISER0 |= 1<<IRQNumber;
		}
		else if(IRQNumber >31 && IRQNumber < 64)
		{
			*NVIC_ISER1 |= (1<<(IRQNumber%32));
		}
		else if(IRQNumber >64 && IRQNumber < 96)
		{
			*NVIC_ISER2 |= (1<<(IRQNumber%64));
		}
	}
	else
	{
		if(IRQNumber <= 31)
		{
			*NVIC_ICER0 |= 1<<IRQNumber;
		}
		else if(IRQNumber >31 && IRQNumber < 64)
		{
			*NVIC_ICER1 |= (1<<(IRQNumber%32));
		}
		else if(IRQNumber >64 && IRQNumber < 96)
		{
			*NVIC_ICER2 |= (1<<(IRQNumber%64));
		}
	}
}

void SPI_IRQPriorityConfig(uint8_t IRQNumber,uint8_t IRQPriority)
{
	uint8_t iprx =  IRQNumber / 4;
	uint8_t	iprx_Section =  IRQNumber %4;
	uint8_t shift_amount = (8*iprx_Section) +(8 - NO_PR_BITS_IMPLEMENTED);
	*(NVIC_PR_BASEADDR + iprx*4) = (IRQPriority << shift_amount);
}

static void SPI_TXE_ISRHAndler(SPI_Handle_t *pHandle)
{
	//Check the dff bit
		if(pHandle->pSPIx->CR1 & (1<<11))
		{
			// 16  bit dff
			pHandle->pSPIx->DR = *((uint16_t *)pHandle->pTxBuffer);
			pHandle->TxLen-- ;
			pHandle->TxLen-- ;
			pHandle->pTxBuffer++;
		}else
		{
			//8 bit dff
			pHandle->pSPIx->DR = *(pHandle->pTxBuffer);
			pHandle->TxLen--;
			pHandle->pTxBuffer++;
		}
		if(!pHandle->TxLen)
		{	// CLEAR INTERRUPT
			pHandle->pSPIx->CR2 &= ~(SPI_TX_FLAG);
			pHandle->pTxBuffer = NULL;
			pHandle->TxLen = 0;
			pHandle->TxState = SPI_READY;
			SPI_ApplicationEventCallback(pHandle,SPI_EVENT_TX_CMPLT);
		}
}
static void SPI_RXE_ISRHAndler(SPI_Handle_t *pHandle)
{
	if(pHandle->pSPIx->CR1 & (1<<11))
	{
		// 16  bit dff
		*((uint16_t *)pHandle->pTxBuffer) = (uint16_t)pHandle->pSPIx->DR;
		pHandle->RxLen-- ;
		pHandle->RxLen-- ;
		pHandle->pRxBuffer--;
		pHandle->pRxBuffer--;
	}else
	{
		//8 bit dff
		*(pHandle->pRxBuffer) = (uint8_t)pHandle->pSPIx->DR;
		pHandle->RxLen--;
		pHandle->pRxBuffer--;
	}
	if(!pHandle->RxLen)
	{	// CLEAR INTERRUPT
		pHandle->pSPIx->CR2 &= ~(SPI_RXNE_FLAG);
		pHandle->pRxBuffer = NULL;
		pHandle->RxLen = 0;
		pHandle->rxState = SPI_READY;
		SPI_ApplicationEventCallback(pHandle,SPI_EVENT_RX_CMPLT);
	}
}


void SPI_IRQHandling(SPI_Handle_t *pHandle)
{
	uint8_t temp1,temp2;
	// Check for TXE
	temp1  = pHandle->pSPIx->SR & (SPI_TX_FLAG);
	temp2  = pHandle->pSPIx->SR & (1 << 7);

	if(temp1 && temp2)
	{
		//handle TXE
		SPI_TXE_ISRHAndler(pHandle);
	}
	temp1  = pHandle->pSPIx->SR & (SPI_RXNE_FLAG);
	temp2  = pHandle->pSPIx->SR & (1 << 6);
	if(temp1 && temp2)
	{
		SPI_RXE_ISRHAndler(pHandle);
	}
}

__weak void SPI_ApplicationEventCallback(SPI_Handle_t *pHandle,uint8_t EventNum)
{}

