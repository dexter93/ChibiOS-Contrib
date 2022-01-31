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
void SPI0_Init(void)
{
	//Enable HCLK for SPI0
	sys1EnableSPI0();																//Enable clock for SPI0.

	//SPI0 setting
	SN_SPI0->CTRL0_b.DL = SPI_DL_8;									//3 ~ 16 Data length
#if defined(SPI_MASTER_MODE)
	SN_SPI0->CTRL0_b.MS = SPI_MS_MASTER_MODE;				//Master/Slave selection bit
#elif defined(SPI_SLAVE_MODE)
	SN_SPI0->CTRL0_b.MS = SPI_MS_SLAVE_MODE;				//Master/Slave selection bit
#endif
	SN_SPI0->CTRL0_b.LOOPBACK = SPI_LOOPBACK_DIS; 	//Loop back mode
	SN_SPI0->CTRL0_b.SDODIS = SPI_SDODIS_EN; 				//Slave data output 
																									//(ONLY used in slave mode)
#if defined(SPI_DIVIDER)
	SN_SPI0->CLKDIV_b.DIV = SPI_DIVIDER;						//SPIn clock divider
#else
	SN_SPI0->CLKDIV_b.DIV = (SPI_DIV / 2) - 1;			//SPIn clock divider
#endif

	//SPI0 SPI mode
	SN_SPI0->CTRL1 = mskSPI_CPHA_FALLING_EDGE|			//Clock phase for edge sampling
									 mskSPI_CPOL_SCK_IDLE_LOW|			//Clock polarity selection bit
									 mskSPI_MLSB_MSB;								//MSB/LSB selection bit

	//SPI0 SEL0 setting
#if defined(SPI_ENABLE_AUTOSEL)
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
void SPI0_Disable(void)
{
  SN_SPI0->CTRL0_b.SPIEN  = SPI_SPIEN_DIS;    		//SPI disable bit
	sys1DisableSPI0();															//Disable clock for SPI0.
}
