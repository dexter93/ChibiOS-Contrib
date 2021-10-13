
#include <SN32F240B.h>
#include "SPI.h"
#include "SPI_master.h"
#ifndef SPI_MASTER_MODE
#define SPI_MASTER_MODE
#endif
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
