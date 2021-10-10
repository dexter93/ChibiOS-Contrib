/******************** (C) COPYRIGHT 2017 SONiX *******************************
* COMPANY:			SONiX
* DATE:					2017/07
* AUTHOR:				SA1
* IC:						SN32F240B
* DESCRIPTION:	SPI0 related functions.
*____________________________________________________________________________
*	REVISION	Date				User		Description
*	1.0				2017/07/07	SA1			1. First release
*
*____________________________________________________________________________
* THE PRESENT SOFTWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
* WITH CODING INFORMATION REGARDING THEIR PRODUCTS TIME TO MARKET.
* SONiX SHALL NOT BE HELD LIABLE FOR ANY DIRECT, INDIRECT OR CONSEQUENTIAL 
* DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE CONTENT OF SUCH SOFTWARE
* AND/OR THE USE MADE BY CUSTOMERS OF THE CODING INFORMATION CONTAINED HEREIN 
* IN CONNECTION WITH THEIR PRODUCTS.
*****************************************************************************/

/*_____ I N C L U D E S ____________________________________________________*/
#include <SN32F240B.h>
#include "SPI.h"


/*_____ D E C L A R A T I O N S ____________________________________________*/


/*_____ D E F I N I T I O N S ______________________________________________*/
#ifndef   SPI_TEST_CNT
#define		SPI_TEST_CNT		32
#endif
uint16_t	hwSPI_Tx_Fifo[SPI_TEST_CNT];
uint16_t	hwSPI_Rx_Fifo[SPI_TEST_CNT];
/*_____ M A C R O S ________________________________________________________*/


/*_____ F U N C T I O N S __________________________________________________*/
uint32_t	wSPI_NBytes = 0;
uint32_t  wSPI_Send_Pointer = 0;
uint32_t  wSPI_Get_Pointer = 0;
/*****************************************************************************
* Function		: SPI0_Init
* Description	: Initialization of SPI0
* Input			: None
* Output		: None
* Return		: None
* Note			: None
*****************************************************************************/
void SPI0_Init(void)
{
	//Enable HCLK for SPI0
	SN_SYS1->AHBCLKEN_b.SPI0CLKEN = 1;							//Enable clock for SPI0.

	//SPI0 setting
	SN_SPI0->CTRL0_b.DL = SPI_DL_8;									//3 ~ 16 Data length
	SN_SPI0->CTRL0_b.MS = SPI_MS_MASTER_MODE;				//Master/Slave selection bit
	SN_SPI0->CTRL0_b.LOOPBACK = SPI_LOOPBACK_DIS; 	//Loop back mode
	SN_SPI0->CTRL0_b.SDODIS = SPI_SDODIS_EN; 				//Slave data output 
																									//(ONLY used in slave mode)
																									
	SN_SPI0->CLKDIV_b.DIV = (SPI_DIV/2) - 1;				//SPIn clock divider

	//SPI0 SPI mode
	SN_SPI0->CTRL1 = mskSPI_CPHA_FALLING_EDGE|					//Clock phase for edge sampling
									 mskSPI_CPOL_SCK_IDLE_LOW|					//Clock polarity selection bit
									 mskSPI_MLSB_MSB;									//MSB/LSB selection bit

	//SPI0 SEL0 setting
	SN_SPI0->CTRL0_b.SELDIS = SPI_SELDIS_DIS; 			//Auto-SEL disable bit
	SN_GPIO2->MODE_b.MODE9=1;											//SEL(P2.9) is outout high
	__SPI0_SET_SEL0;

	//SPI0 Fifo reset
	__SPI0_FIFO_RESET;
	
  SPI0_NvicEnable();	
	//__SPI0_DATA_FETCH_HIGH_SPEED;									//Enable if Freq. of SCK > 6MHz

	//SPI0 enable	
	SN_SPI0->CTRL0_b.SPIEN  = SPI_SPIEN_EN;    			//SPI enable bit	
}

/*****************************************************************************
* Function		: SPI0_Enable
* Description	: SPI0 enable setting
* Input			: None
* Output		: None
* Return		: None
* Note			: None
*****************************************************************************/
void SPI0_Enable(void)
{
	//Enable HCLK for SPI0
	SN_SYS1->AHBCLKEN |= (0x1 << 12);								//Enable clock for SPI0.

  SN_SPI0->CTRL0_b.SPIEN  = SPI_SPIEN_EN;    			//SPI enable bit
	__SPI0_FIFO_RESET;
}

/*****************************************************************************
* Function		: SPI0_Disable
* Description	: SPI0 disable setting
* Input			: None
* Output		: None
* Return		: None
* Note			: None
*****************************************************************************/
void SPI0_Disable(void)
{
  SN_SPI0->CTRL0_b.SPIEN  = SPI_SPIEN_DIS;    		//SPI disable bit

	//Disable HCLK for SPI0
	SN_SYS1->AHBCLKEN &=~ (0x1 << 12);							//Disable clock for SPI0.
}

/*****************************************************************************
* Function		: SPI0_Write
* Description	: SPI0 Write buffer
* Input			: None
* Output		: None
* Return		: None
* Note			: None
*****************************************************************************/
void SPI0_Write(unsigned char *p, int len)
{
    for (int i = 0; i < len; i++)
    {
        // while (!SN_SPI0->STAT_b.TX_EMPTY);
        while (SN_SPI0->STAT_b.TX_FULL);
        SN_SPI0->DATA_b.Data = *p++;
    }
    
    while (SN_SPI0->STAT_b.BUSY);
}

/*****************************************************************************
* Function		: SPI0_Read3
* Description	: SPI0 Read 3 bytes
* Input			: None
* Output		: None
* Return		: None
* Note			: None
*****************************************************************************/
void SPI0_Read3(unsigned char b1, unsigned char b2, unsigned char *b3)
{
    /* write first 2 bytes: header and address */
    while (!SN_SPI0->STAT_b.TX_EMPTY);
    SN_SPI0->DATA_b.Data = b1;
    SN_SPI0->DATA_b.Data = b2;

    /* read 1 byte data */
    while (SN_SPI0->STAT_b.BUSY);
    while (SN_SPI0->STAT_b.RX_EMPTY);
    *b3 = SN_SPI0->DATA_b.Data;
         
    while (SN_SPI0->STAT_b.BUSY);
}

/*****************************************************************************
* Function		: SPI0_NvicEnable
* Description	: Enable SPI0 interrupt
* Input			: None
* Output		: None
* Return		: None
* Note			: None
*****************************************************************************/
void	SPI0_NvicEnable (void)
{
	NVIC_ClearPendingIRQ(SPI0_IRQn);
	NVIC_EnableIRQ(SPI0_IRQn);
	//NVIC_SetPriority(SPI0,0);			// Set interrupt priority (default)
}

/*****************************************************************************
* Function		: SPI0_NvicDisable
* Description	: Enable SPI0 interrupt
* Input			: None
* Output		: None
* Return		: None
* Note			: None
*****************************************************************************/
void	SPI0_NvicDisable (void)
{
	NVIC_DisableIRQ(SPI0_IRQn);
}

/*****************************************************************************
* Function		: SPI0_IRQHandler
* Description	: None
* Input			: None
* Output		: None
* Return		: None
* Note			: None
*****************************************************************************/
void SPI0_IRQHandler(void)
{
	__SPI0_CLR_SEL0;										//SEL is low
	SN_SPI0->DATA = hwSPI_Tx_Fifo[wSPI_Send_Pointer++];
	if(!(SN_SPI0->STAT & mskSPI_RX_EMPTY))		//Check having any data in RXFIFO
	{	
		hwSPI_Rx_Fifo[wSPI_Get_Pointer++] = SN_SPI0->DATA;
	}	
	
	if(wSPI_Send_Pointer == wSPI_NBytes)
	{
		SN_SPI0->IE_b.TXFIFOTHIE = SPI_TXFIFOTHIE_DIS;	//TX FIFO threshold interrupt disable
	}		

	SN_SPI0->IC = mskSPI_TXFIFOTHIC;	//Clear overFlow flag
}

/*****************************************************************************
* Function		: SPI0_NBytesTxRxIrp
* Description	:  
* Input			: hwSPI_Tx_Fifo, 
* Output		: hwSPI_Rx_Fifo
* Return		: None
* Note			: Avoid all of the interrupts which may make FW can't fill in TX FIFO (SN_SPI0->DATA=...) in time.
*****************************************************************************/
void SPI0_NBytesTxRxIrp(uint32_t N_Bytes)
{
	wSPI_NBytes = N_Bytes; 

	SN_SPI0->IE_b.TXFIFOTHIE = SPI_TXFIFOTHIE_EN;

	while(wSPI_Send_Pointer != wSPI_NBytes);	

	while(1)
	{
		while(SN_SPI0->STAT & mskSPI_RX_EMPTY); //Get all remaining data				
		
		hwSPI_Rx_Fifo[wSPI_Get_Pointer++] = SN_SPI0->DATA;
			
		if(wSPI_Get_Pointer == wSPI_NBytes)
		{
			break;
		}
	}

	__SPI0_SET_SEL0;										//SEL is high

	//Reset Variable
	wSPI_Send_Pointer = 0;
	wSPI_Get_Pointer = 0;
}


/***************************************************************************************************************
* Function		: SIP0_NBytesTxRx
* Description	:
* Input			: hwSPI_Tx_Fifo
* Output		: hwSPI_Rx_Fifo
* Return		: None
* Note			: Avoid all of the interrupts which may make FW can't fill in TX FIFO (SN_SPI0->DATA=...) in time.
***************************************************************************************************************/
void SPI0_NBytesTxRx(uint32_t N_Bytes)
{
	uint32_t  wSPI_Send_Pointer = 0;
	uint32_t  wSPI_Get_Pointer = 0;

	while(wSPI_Send_Pointer != N_Bytes)
	{
		__SPI0_CLR_SEL0;																			//SEL is low
		SN_SPI0->DATA = hwSPI_Tx_Fifo[wSPI_Send_Pointer++];
		while (!(SN_SPI0->STAT & mskSPI_TXFIFOTHF));	//TX Half-Empty
		if(!(SN_SPI0->STAT & mskSPI_RX_EMPTY))								//Check having any data in RXFIFO
		{	
			hwSPI_Rx_Fifo[wSPI_Get_Pointer++] = SN_SPI0->DATA;
		}
	}	

	while(1)
	{
		while(SN_SPI0->STAT & mskSPI_RX_EMPTY); //Get all remaining data
		hwSPI_Rx_Fifo[wSPI_Get_Pointer++] = SN_SPI0->DATA;

		if(wSPI_Get_Pointer == N_Bytes)
		{
			break;
		}
	}
	while(SN_SPI0->STAT & mskSPI_BUSY);

	__SPI0_SET_SEL0;										//SEL is high
	__SPI0_FIFO_RESET;
}
