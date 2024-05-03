/*
    Copyright (C) 2023 Dimitris Mantzouranis

    Licensed under the Apache License, Version 2.0 (the "License");
    you may not use this file except in compliance with the License.
    You may obtain a copy of the License at

        http://www.apache.org/licenses/LICENSE-2.0

    Unless required by applicable law or agreed to in writing, software
    distributed under the License is distributed on an "AS IS" BASIS,
    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
    See the License for the specific language governing permissions and
    limitations under the License.
*/

/**
 * @file    UART/hal_serial_lld.c
 * @brief   SN32 low level serial driver code.
 *
 * @addtogroup SERIAL
 * @{
 */

#include "hal.h"
#include "matrix.h"
#include "print.h"
#include <math.h>
#if HAL_USE_SERIAL || defined(__DOXYGEN__)

/*===========================================================================*/
/* Driver local definitions.                                                 */
/*===========================================================================*/

/*===========================================================================*/
/* Driver exported variables.                                                */
/*===========================================================================*/

/** @brief UART0 serial driver identifier.*/
#if SN32_SERIAL_USE_UART0 || defined(__DOXYGEN__)
SerialDriver SD0;
#endif

/** @brief UART1 serial driver identifier.*/
#if SN32_SERIAL_USE_UART1 || defined(__DOXYGEN__)
SerialDriver SD1;
#endif

/** @brief UART2 serial driver identifier.*/
#if SN32_SERIAL_USE_UART2 || defined(__DOXYGEN__)
SerialDriver SD2;
#endif

/*===========================================================================*/
/* Driver local variables and types.                                         */
/*===========================================================================*/

/* Driver default configuration.*/
static const SerialConfig default_config = {SERIAL_DEFAULT_BITRATE,
                                            UART_WordLength_8b,
                                            UART_StopBits_One,
                                            UART_Parity_None,
                                            (UART_FIFO_Enable | UART_RxFIFOThreshold_1),
                                            UART_AutoBaudControl_None};

/*===========================================================================*/
/* Driver local functions.                                                   */
/*===========================================================================*/
void UART_divisor_CAL(uint32_t baudrate,uint32_t UART_PCLK,uint8_t Oversampling,uint8_t *DLM,uint8_t *DLL,uint8_t  *D_DIVADDVAL,uint8_t  *D_MULVAL)
{
  float expected_val;
  uint8_t DIVADDVAL[2],MULVAL[2];
  uint8_t divider_Index = 0;
  uint8_t f_divider_new=0;
  uint16_t  divisor=0;
  float   divider_plus,divider_minus,divider_expected;
  uint32_t  i,j,k;
  
  //Init
  for(i=0;i<2;i++)  {
    MULVAL[i] = 0;
    DIVADDVAL[i] = 0;
  }
  
  expected_val = (float)UART_PCLK/Oversampling/baudrate;
  
  if((int)expected_val == expected_val) {
    divisor = expected_val;
    // no fractional divider needed
    DIVADDVAL[0] = 0;
    MULVAL[0] = 1;
  } else {
    // we have to use the fractional divider
    // generate a lookup table
    uint8_t mulval_limit = 16;
    uint8_t divaddval_limit =15;
    float tab_D_div_M[mulval_limit][divaddval_limit]; 
    for (uint8_t i = 0; i < mulval_limit; i++) {
        for (uint8_t j = 0; j < divaddval_limit; j++) {
            tab_D_div_M[i][j] = (float)j / (i + 1);
        }
    }
    // go through the table until we have a match
    for(i=expected_val/2;i<expected_val;i++) {
      if(3<=i && i<0x100) {
        divider_expected = (expected_val)/i - 1;
        divider_plus = (divider_expected+1) *(1+0.011) - 1;
        divider_minus = (divider_expected+1) *(1-0.011) - 1;
        for(j=1;j<mulval_limit;j++) {
          for(k=0;k<divaddval_limit;k++) {
            if(j>k) {
              if(tab_D_div_M[j][k]>divider_minus && tab_D_div_M[j][k]<divider_plus) {
                if(MULVAL[divider_Index] == 0 && DIVADDVAL[divider_Index] == 0) {
                  MULVAL[divider_Index] = j;
                  DIVADDVAL[divider_Index] = k;
                  f_divider_new = 1;
                } else {
                  if((fabs)(tab_D_div_M[j][k]-divider_expected) < (fabs)(tab_D_div_M[MULVAL[divider_Index]][DIVADDVAL[divider_Index]]-divider_expected)) {
                    MULVAL[divider_Index] = j;
                    DIVADDVAL[divider_Index] = k;     
                    f_divider_new = 1;
                  }
                }
              }
            }
          }
        }
      }
      else {
        MULVAL[divider_Index] = 1;
        DIVADDVAL[divider_Index] = 0;
        f_divider_new = 1;        
      }
      if(f_divider_new == 1 ) {
        if(divider_Index == 0) {
          divider_Index++;
          divisor = i;
        } else {
          if((fabs)((tab_D_div_M[MULVAL[1]][DIVADDVAL[1]]+1)*i-expected_val) < (fabs)((tab_D_div_M[MULVAL[0]][DIVADDVAL[0]]+1)*divisor-expected_val)) {
            MULVAL[0] = MULVAL[1];
            DIVADDVAL[0] = DIVADDVAL[1];
            divisor = i;
          }
        }
      }
    }

  }
  // check the divisor is valid
  if(divisor != 0){
    *DLM = (divisor>>8)&0xff;
    *DLL = divisor&0xff;
    *D_MULVAL = MULVAL[0];
    *D_DIVADDVAL = DIVADDVAL[0];
  }
}
/**
 * @brief   UART initialization.
 * @details This function must be invoked with interrupts disabled.
 *
 * @param[in] sdp       pointer to a @p SerialDriver object
 * @param[in] config    the architecture-dependent serial driver configuration
 */
static void uart_init(SerialDriver *sdp, const SerialConfig *config) {
  uint32_t apbclock;
  uint8_t dlm, dll, divaddval, mulval, oversampling;
  sn32_uart_t *u = sdp->uart;

  apbclock = SN32_HCLK;

#if defined(UART_OVER8)
  oversampling = 8;
#else
  oversampling = 16;
#endif
  
  // Calculate divider
  UART_divisor_CAL(config->speed,apbclock,oversampling,&dlm,&dll,&divaddval,&mulval);

  // Update the registers
  u->LC = (config->UART_WordLength
          | config->UART_StopBits
          | config->UART_Parity
          | UART_Break_Control_Disable
          | UART_Divisor_Latch_Access_Enable);

  u->FD_b.MULVAL = mulval;
  u->FD_b.DIVADDVAL = divaddval;
  u->FD_b.OVER8 = (oversampling == 8) ? 1 : 0;
  u->DLM_b.DLM = dlm;
  u->DLL_b.DLL = dll;

  u->LC &= ~(UART_Divisor_Latch_Access_Enable);
  // Disable AutoBaud for serial - not useful
  u->ABCTRL = UART_AutoBaudControl_None;

  // Reset FIFO and enable
  // Set RX trigger level
  u->FIFOCTRL = (UART_FIFO_Enable
                | UART_RxFIFO_Reset
                | UART_TxFIFO_Reset
                | config->UART_FIFOControl);

  /* Enable Interrupts*/
  u->IE = (UART_ReceiveDataAvailable | UART_ReceiveLine);

  // Enable UART
  u->CTRL = (UART_Enable| UART_RxEnable | UART_TxEnable);
}

/**
 * @brief   UART de-initialization.
 * @details This function must be invoked with interrupts disabled.
 *
 * @param[in] u         pointer to an UART I/O block
 */
static void uart_deinit(sn32_uart_t *u) {
  // disable FIFOs
  u->FIFOCTRL_b.FIFOEN =0;
  // disable interrupts
  u->IE =0;
  // disable UART peripheral
  u->CTRL =0;
}

/**
 * @brief   Error handling routine.
 *
 * @param[in] sdp       pointer to a @p SerialDriver object
 * @param[in] ls        UART LS register value
*/
static void set_error(SerialDriver *sdp, uint8_t ls) {
  eventflags_t sts = 0;

  if(ls & UART_LineStatus_BI)
    sts |= SD_BREAK_DETECTED;
  if(ls & UART_LineStatus_OE)
    sts |= SD_OVERRUN_ERROR;
  if (ls & UART_LineStatus_PE)
    sts |= SD_PARITY_ERROR;
  if (ls & UART_LineStatus_FE)
    sts |= SD_FRAMING_ERROR;

  osalSysLockFromISR();
  chnAddFlagsI(sdp, sts);
  osalSysUnlockFromISR();
}

/**
 * @brief   Common IRQ handler.
 *
 * @param[in] sdp       communication channel associated to the UART
 */
static void serve_interrupt(SerialDriver *sdp) {
  sn32_uart_t *u = sdp->uart;
  uint32_t ii=u->II;

  while ((ii & UART_Interrupt_Status) == UART_Interrupt_Pending) {
    uint32_t int_status = (ii >> UART_Interrupt_Status);
    switch (int_status & UART_InterruptID_Status) {
    case UART_Interrupt_Pending:
      return;
    case UART_InterruptID_RLS:
      set_error(sdp, u->LS);
      break;
    case UART_InterruptID_CTI:
    case UART_InterruptID_RDA:
      osalSysLockFromISR();
      if (iqIsEmptyI(&sdp->iqueue))
        chnAddFlagsI(sdp, CHN_INPUT_AVAILABLE);
      osalSysUnlockFromISR();
      while (u->LS & UART_LineStatus_RDR) {
        osalSysLockFromISR();
        if (iqPutI(&sdp->iqueue, u->RB) < MSG_OK)
          chnAddFlagsI(sdp, SD_OVERRUN_ERROR);
        osalSysUnlockFromISR();
      }
      break;
    case UART_InterruptID_THRE:
      msg_t b;

      osalSysLockFromISR();
      b = oqGetI(&sdp->oqueue);
      osalSysUnlockFromISR();
      if (b < MSG_OK) {
        u->IE &= ~UART_TransmitterHoldingEmpty;
        osalSysLockFromISR();
        chnAddFlagsI(sdp, CHN_OUTPUT_EMPTY);
        osalSysUnlockFromISR();
        break;
      }
      u->TH = b;
      break;
    case UART_InterruptID_TEMT:
      osalSysLockFromISR();
      if (oqIsEmptyI(&sdp->oqueue)) {
          chnAddFlagsI(sdp, CHN_TRANSMISSION_END);
          u->IE &= ~(UART_TransmitterEmpty);
      }
      osalSysUnlockFromISR();
      break;
    default:
    }
    ii=u->II;
  }
}

static void load(SerialDriver *sdp) {
  sn32_uart_t *u = sdp->uart;
  if (u->LS & UART_LineStatus_THRE) {
    osalSysLock();
    msg_t b = oqGetI(&sdp->oqueue);
    if (b < MSG_OK) {
      chnAddFlagsI(sdp, CHN_OUTPUT_EMPTY);
      osalSysUnlock();
      return;
    }
    u->TH = b;
    osalSysUnlock();
  }
  u->IE |= (UART_TransmitterHoldingEmpty | UART_TransmitterEmpty);
}
#if SN32_SERIAL_USE_UART0 || defined(__DOXYGEN__)
static void notify0(io_queue_t *qp) {

  (void)qp;
  load(&SD0);
}
#endif

#if SN32_SERIAL_USE_UART1 || defined(__DOXYGEN__)
static void notify1(io_queue_t *qp) {

  (void)qp;
  load(&SD1);
}
#endif

#if SN32_SERIAL_USE_UART2 || defined(__DOXYGEN__)
static void notify2(io_queue_t *qp) {

  (void)qp;
  load(&SD2);
}
#endif
/*===========================================================================*/
/* Driver interrupt handlers.                                                */
/*===========================================================================*/

#if SN32_SERIAL_USE_UART0 || defined(__DOXYGEN__)
#if !defined(SN32_UART0_HANDLER)
#error "SN32_UART0_HANDLER not defined"
#endif
/**
 * @brief   UART0 interrupt handler.
 *
 * @isr
 */
OSAL_IRQ_HANDLER(SN32_UART0_HANDLER) {

  OSAL_IRQ_PROLOGUE();

  serve_interrupt(&SD0);

  OSAL_IRQ_EPILOGUE();
}
#endif

#if SN32_SERIAL_USE_UART1 || defined(__DOXYGEN__)
#if !defined(SN32_UART1_HANDLER)
#error "SN32_UART1_HANDLER not defined"
#endif
/**
 * @brief   UART1 interrupt handler.
 *
 * @isr
 */
OSAL_IRQ_HANDLER(SN32_UART1_HANDLER) {

  OSAL_IRQ_PROLOGUE();

  serve_interrupt(&SD1);

  OSAL_IRQ_EPILOGUE();
}
#endif

#if SN32_SERIAL_USE_UART2 || defined(__DOXYGEN__)
#if !defined(SN32_UART2_HANDLER)
#error "SN32_UART2_HANDLER not defined"
#endif
/**
 * @brief   UART2 interrupt handler.
 *
 * @isr
 */
OSAL_IRQ_HANDLER(SN32_UART2_HANDLER) {

  OSAL_IRQ_PROLOGUE();

  serve_interrupt(&SD2);

  OSAL_IRQ_EPILOGUE();
}
#endif

/*===========================================================================*/
/* Driver exported functions.                                                */
/*===========================================================================*/

/**
 * @brief   Low level serial driver initialization.
 *
 * @notapi
 */
void sd_lld_init(void) {

#if SN32_SERIAL_USE_UART0
  sdObjectInit(&SD0, NULL, notify0);
  SD0.uart = SN32_UART0;
#endif

#if SN32_SERIAL_USE_UART1
  sdObjectInit(&SD1, NULL, notify1);
  SD1.uart = SN32_UART1;
#endif

#if SN32_SERIAL_USE_UART2
  sdObjectInit(&SD2, NULL, notify2);
  SD2.uart = SN32_UART2;
#endif
}

/**
 * @brief   Low level serial driver configuration and (re)start.
 *
 * @param[in] sdp       pointer to a @p SerialDriver object
 * @param[in] config    the architecture-dependent serial driver configuration.
 *                      If this parameter is set to @p NULL then a default
 *                      configuration is used.
 *
 * @notapi
 */
void sd_lld_start(SerialDriver *sdp, const SerialConfig *config) {

  if (config == NULL)
    config = &default_config;

  if (sdp->state == SD_STOP) {
#if SN32_SERIAL_USE_UART0
    if (&SD0 == sdp) {
      /* UART0 clock enable.*/
      sys1EnableUART0();
      uart_init(sdp, config);
      nvicEnableVector(SN32_UART0_NUMBER, SN32_SERIAL_UART0_PRIORITY);
    }
#endif
#if SN32_SERIAL_USE_UART1
    if (&SD1 == sdp) {
      /* UART1 clock enable.*/
      sys1EnableUART1();
      uart_init(sdp, config);
      nvicEnableVector(SN32_UART1_NUMBER, SN32_SERIAL_UART1_PRIORITY);
    }
#endif
#if SN32_SERIAL_USE_UART2
    if (&SD2 == sdp) {
      /* UART2 clock enable.*/
      sys1EnableUART2();
      uart_init(sdp, config);
      nvicEnableVector(SN32_UART2_NUMBER, SN32_SERIAL_UART2_PRIORITY);
    }
#endif
  }
}

/**
 * @brief   Low level serial driver stop.
 * @details De-initializes the UART, stops the associated clock, resets the
 *          interrupt vector.
 *
 * @param[in] sdp       pointer to a @p SerialDriver object
 *
 * @notapi
 */
void sd_lld_stop(SerialDriver *sdp) {

  if (sdp->state == SD_READY) {
    uart_deinit(sdp->uart);
#if SN32_SERIAL_USE_UART0
    if (&SD0 == sdp) {
      /* UART0 DeInit.*/
      sys1DisableUART0();
      nvicDisableVector(SN32_UART0_NUMBER);
      return;
    }
#endif
#if SN32_SERIAL_USE_UART1
    if (&SD1 == sdp) {
      /* UART1 DeInit.*/
      sys1DisableUART1();
      nvicDisableVector(SN32_UART1_NUMBER);
      return;
    }
#endif
#if SN32_SERIAL_USE_UART2
    if (&SD2 == sdp) {
      /* UART2 DeInit.*/
      sys1DisableUART2();
      nvicDisableVector(SN32_UART2_NUMBER);
      return;
    }
#endif
  }
}

#endif /* HAL_USE_SERIAL */

/** @} */
