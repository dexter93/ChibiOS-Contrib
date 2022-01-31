/******************** (C) COPYRIGHT 2020 SONiX *******************************
* COMPANY:			SONiX
* DATE:					2020/06
* AUTHOR:				SA1
* IC:						SN32F280/SN32F290
* DESCRIPTION:	SPI0 related functions.
*____________________________________________________________________________
*	REVISION	Date				User		Description
*	1.0				2020/06/24	SA1			1. First release
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
#include "SPI.h"

/*_____ D E C L A R A T I O N S ____________________________________________*/

/*_____ D E F I N I T I O N S ______________________________________________*/

/*_____ M A C R O S ________________________________________________________*/

/*_____ F U N C T I O N S __________________________________________________*/

/*****************************************************************************
* Function		: SPI0_Init
* Description	: Initialization of SPI0
* Input			: None
* Output		: None
* Return		: None
* Note			: None
*****************************************************************************/
void SPI0_Init(void) {
	//Enable HCLK for SPI0
	sys1EnableSPI0();																//Enable clock for SPI0.

	//SPI0 setting
	SN_SPI0->CTRL0_b.DL = SPI_DL_8;									//3 ~ 16 Data length
#if defined(SN32_SPI_SLAVE_MODE)
	SN_SPI0->CTRL0_b.MS = SPI_MS_SLAVE_MODE;				//Master/Slave selection bit
#else
	SN_SPI0->CTRL0_b.MS = SPI_MS_MASTER_MODE;				//Master/Slave selection bit
#endif
	SN_SPI0->CTRL0_b.LOOPBACK = SPI_LOOPBACK_DIS; 	//Loop back mode
	SN_SPI0->CTRL0_b.SDODIS = SPI_SDODIS_EN; 				//Slave data output 
																									//(ONLY used in slave mode)
#if defined(SN32_SPI_RXFIFO_THRESHOLD)						//Override hw default of 0
	SN_SPI0->CTRL0_b.RXFIFOTH       = SN32_SPI_RXFIFO_THRESHOLD;
#endif
#if defined(SN32_SPI_TXFIFO_THRESHOLD)						//Override hw default of 0
	SN_SPI0->CTRL0_b.TXFIFOTH       = SN32_SPI_TXFIFO_THRESHOLD;
#endif
#if defined(SN32_SPI_DIVIDER)
	SN_SPI0->CLKDIV_b.DIV = SN32_SPI_DIVIDER;				//SPIn clock divider
#else
	SN_SPI0->CLKDIV_b.DIV = (SPI_DIV / 2) - 1;			//SPIn clock divider
#endif

	//SPI0 SPI mode
	SN_SPI0->CTRL1 = mskSPI_CPHA_FALLING_EDGE|			//Clock phase for edge sampling
									 mskSPI_CPOL_SCK_IDLE_LOW|			//Clock polarity selection bit
									 mskSPI_MLSB_MSB;								//MSB/LSB selection bit

	//SPI0 SEL0 setting
#if defined(SN32_SPI_ENABLE_AUTOSEL)
	SN_SPI0->CTRL0_b.SELDIS = SPI_SELDIS_DIS; 				//Auto-SEL disable bit
#else
	SN_SPI0->CTRL0_b.SELDIS = SPI_SELDIS_EN; 					//Auto-SEL disable bit
#endif
	//SN_GPIO2->MODE_b.MODE9 = 1;											//SEL(P2.9) is output high
	//__SPI0_SET_SEL0;

	//SPI0 Fifo reset
	__SPI0_FIFO_RESET;

	uint32_t spiClock = (SN32_HCLK / ((2* SN_SPI0->CLKDIV_b.DIV) + 2));
	if(spiClock >6000000){
	__SPI0_DATA_FETCH_HIGH_SPEED;									//Enable if Freq. of SCK > 6MHz
	}

	nvicDisableVector(SN32_SPI0_NUMBER);
#if defined(SN32_SPI_IRQ_PIN)
	palClearLine(SN32_SPI_IRQ_PIN);
#endif

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
void SPI0_Enable(void) {
	sys1EnableSPI0();																//Enable clock for SPI0.
  SN_SPI0->CTRL0_b.SPIEN = SPI_SPIEN_EN;    			//SPI enable bit
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
void SPI0_Disable(void) {
  SN_SPI0->CTRL0_b.SPIEN  = SPI_SPIEN_DIS;    		//SPI disable bit
	sys1DisableSPI0();															//Disable clock for SPI0.
}
/*****************************************************************************
* Function		: SPI0_Send_Init
* Description	: SPI0 Send Init function
* Input			: None
* Output		: None
* Return		: None
* Note			: None
*****************************************************************************/
void SPI0_Send_Init(void) {
#if defined(SN32_SPI_IRQ_PIN)
	palSetLine(SN32_SPI_IRQ_PIN);
#endif
}
/*****************************************************************************
* Function		: SPI0_Send_End
* Description	: SPI0 Send End function
* Input			: None
* Output		: None
* Return		: None
* Note			: None
*****************************************************************************/
void SPI0_Send_End(void) {
	while (!SN_SPI0->STAT_b.TX_EMPTY);
#if defined(SN32_SPI_IRQ_PIN)
	palClearLine(SN32_SPI_IRQ_PIN);
#endif
}
/*****************************************************************************
* Function		: SPI0_Flush
* Description	: SPI0 Flush function
* Input			: None
* Output		: None
* Return		: None
* Note			: None
*****************************************************************************/
void SPI0_Flush(void) {
	while (SN_SPI0->STAT_b.BUSY);
}
/*****************************************************************************
* Function		: SPI0_Write
* Description	: SPI0 Write data
* Input			: p pointer to data, length len
* Output		: None
* Return		: None
* Note			: None
*****************************************************************************/
void SPI0_Write(unsigned char *p, int len) {
	for (int i = 0; i < len; i++) {
		while (!SN_SPI0->STAT_b.TX_EMPTY);
		SN_SPI0->DATA_b.Data = *p++;
	}
	SPI0_Flush();
}
/*****************************************************************************
* Function		: SPI0_Write1
* Description	: SPI0 Write single packet
* Input			: data
* Output		: None
* Return		: None
* Note			: None
*****************************************************************************/
void SPI0_Write1(uint8_t data) {
	while (!SN_SPI0->STAT_b.TX_EMPTY);
	SN_SPI0->DATA_b.Data = data;
}
/*****************************************************************************
* Function		: SPI0_Read3
* Description	: SPI0 Request data at address
* Input			: header b1, address b2, data pointer b3
* Output		: None
* Return		: None
* Note			: None
*****************************************************************************/
void SPI0_Read3(unsigned char b1, unsigned char b2, unsigned char *b3) {
	/* write first 2 bytes: header and address */
	while (!SN_SPI0->STAT_b.TX_EMPTY);
	SN_SPI0->DATA_b.Data = b1;
	SN_SPI0->DATA_b.Data = b2;
	/* read 1 byte data */
	SPI0_Flush();
	while (SN_SPI0->STAT_b.RX_EMPTY);
	*b3 = SN_SPI0->DATA_b.Data;
	SPI0_Flush();
}
