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

const float tab_D_div_M[16][15] = 
{
  {0,1,2,3,4,5,6,7,8,9,10,11,12,13,14},
  {0,1,2,3,4,5,6,7,8,9,10,11,12,13,14},
  {0,0.5,1,1.5,2,2.5,3,3.5,4,4.5,5,5.5,6,6.5,7},
  {0,0.333333333,0.666666667,1,1.333333333,1.666666667,2,2.333333333,2.666666667,3,3.333333333,3.666666667,4,4.333333333,4.666666667},
  {0,0.25,0.5,0.75,1,1.25,1.5,1.75,2,2.25,2.5,2.75,3,3.25,3.5},
  {0,0.2,0.4,0.6,0.8,1,1.2,1.4,1.6,1.8,2,2.2,2.4,2.6,2.8},
  {0,0.166666667,0.333333333,0.5,0.666666667,0.833333333,1,1.166666667,1.333333333,1.5,1.666666667,1.833333333,2,2.166666667,2.333333333},
  {0,0.142857143,0.285714286,0.428571429,0.571428571,0.714285714,0.857142857,1,1.142857143,1.285714286,1.428571429,1.571428571,1.714285714,1.857142857,2},
  {0,0.125,0.25,0.375,0.5,0.625,0.75,0.875,1,1.125,1.25,1.375,1.5,1.625,1.75},
  {0,0.111111111,0.222222222,0.333333333,0.444444444,0.555555556,0.666666667,0.777777778,0.888888889,1,1.111111111,1.222222222,1.333333333,1.444444444,1.555555556},
  {0,0.1,0.2,0.3,0.4,0.5,0.6,0.7,0.8,0.9,1,1.1,1.2,1.3,1.4},
  {0,0.090909091,0.181818182,0.272727273,0.363636364,0.454545455,0.545454545,0.636363636,0.727272727,0.818181818,0.909090909,1,1.090909091,1.181818182,1.272727273},
  {0,0.083333333,0.166666667,0.25,0.333333333,0.416666667,0.5,0.583333333,0.666666667,0.75,0.833333333,0.916666667,1,1.083333333,1.166666667},
  {0,0.076923077,0.153846154,0.230769231,0.307692308,0.384615385,0.461538462,0.538461538,0.615384615,0.692307692,0.769230769,0.846153846,0.923076923,1,1.076923077},
  {0,0.071428571,0.142857143,0.214285714,0.285714286,0.357142857,0.428571429,0.5,0.571428571,0.642857143,0.714285714,0.785714286,0.857142857,0.928571429,1},
  {0,0.066666667,0.133333333,0.2,0.266666667,0.333333333,0.4,0.466666667,0.533333333,0.6,0.666666667,0.733333333,0.8,0.866666667,0.933333333},
};
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
  
  if((int)expected_val == expected_val)
  {
    divisor = expected_val;
    DIVADDVAL[0] = 0;
    MULVAL[0] = 1;
//    return  1;
  }
  else
  {
    for(i=expected_val/2;i<expected_val;i++)
    {
      if(3<=i && i<0x100){
        divider_expected = (expected_val)/i - 1;
        divider_plus = (divider_expected+1) *(1+0.011) - 1;
        divider_minus = (divider_expected+1) *(1-0.011) - 1;
        for(j=1;j<16;j++)
        for(k=0;k<15;k++)
        {
          if(j>k)
          {
            if(tab_D_div_M[j][k]>divider_minus && tab_D_div_M[j][k]<divider_plus)
            {
              if(MULVAL[divider_Index] == 0 && DIVADDVAL[divider_Index] == 0)
              {
                MULVAL[divider_Index] = j;
                DIVADDVAL[divider_Index] = k;
                f_divider_new = 1;
              }
              else
              {
                if( (fabs)(tab_D_div_M[j][k]-divider_expected) < (fabs)(tab_D_div_M[MULVAL[divider_Index]][DIVADDVAL[divider_Index]]-divider_expected) )
                {
                  MULVAL[divider_Index] = j;
                  DIVADDVAL[divider_Index] = k;     
                  f_divider_new = 1;
                }
              }
              
            }
          }
        }
      }
      else{
        MULVAL[divider_Index] = 1;
        DIVADDVAL[divider_Index] = 0;
        f_divider_new = 1;        
      }
      if(f_divider_new == 1 )
      {
        if(divider_Index == 0){
          divider_Index++;
          divisor = i;
        }
        else{
          if( (fabs)((tab_D_div_M[MULVAL[1]][DIVADDVAL[1]]+1)*i-expected_val) < (fabs)((tab_D_div_M[MULVAL[0]][DIVADDVAL[0]]+1)*divisor-expected_val) ){
            MULVAL[0] = MULVAL[1];
            DIVADDVAL[0] = DIVADDVAL[1];
            divisor = i;
          }
          
        }
      }
      
    }

  }
  chDbgAssert(((divisor) == 0), "Invalid divider value");

    if(divisor != 0){
      *DLM = (divisor>>8)&0xff;
      *DLL = divisor&0xff;
      *D_MULVAL = MULVAL[0];
      *D_DIVADDVAL = DIVADDVAL[0];
      
      /*
      U_DLM = (divisor>>8)&0xff;
      U_DLL = divisor&0xff;
      U_MULVAL = MULVAL[0];
      U_DIVADDVALL = DIVADDVAL[0];
      U_Baudrate = (double)UART_PCLK/(Oversampling*divisor*(1+(float)DIVADDVAL[0]/MULVAL[0]));
      U_Deviation = (float)(U_Baudrate-baudrate)/baudrate;
     // return  1; */
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
//#define UART_OVER8
#if defined(UART_OVER8)
  oversampling = 8;
#else
  oversampling = 16;
#endif
/*
  // Calculate divider
  uint32_t uart_clock = config->speed * oversampling;

  // Check constraints based on oversampling value
  chDbgAssert(uart_clock <= apbclock / oversampling,
              "Invalid oversampling configuration for requested baud rate");

  uint32_t divider = (apbclock + uart_clock/2) / uart_clock;
  chDbgAssert((divider >= 1) && (divider <= 0xFFFF), "Invalid divider value");
  dlm = (0);
  dll = (17);
  divaddval = 8; // disable fractional divider
  mulval = 15;
  chDbgAssert((mulval >= 1) && (mulval <= 15), "Invalid Baud rate pre-scaler multiplier value");
  chDbgAssert(((mulval - divaddval) == 2), "Invalid mulval/divaddval");
*/
  UART_divisor_CAL(config->speed,apbclock,oversampling,&dlm,&dll,&divaddval,&mulval);

  // Update the registers
  u->LC = (config->UART_WordLength
          | config->UART_StopBits
          | config->UART_Parity
          | UART_Break_Control_Disable
          | UART_Divisor_Latch_Access_Enable);
  //u->FD = (UART_FD_MULVAL(mulval) | UART_FD_DIVADDVAL(divaddval));
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
 // u->IE = (UART_ReceiveDataAvailable | UART_ReceiveLine);

  // Enable UART
  //u->CTRL = (UART_Enable| UART_RxEnable | UART_TxEnable);
  //u->CTRL_b.TXEN=0;
  gpio_set_pin_output(C0);
  gpio_set_pin_output(C1);
  gpio_set_pin_output(C2);
  gpio_set_pin_output(C3);
  gpio_set_pin_output(C4);
  gpio_set_pin_output(C5);
  gpio_set_pin_output(C6);
  gpio_set_pin_output(C7);
  gpio_set_pin_output(C8);
  gpio_set_pin_output(C9);
  gpio_set_pin_output(C10);
  gpio_set_pin_output(C11);
  gpio_set_pin_output(C12);
  gpio_set_pin_output(C13);
  gpio_write_pin_low(C0);
  gpio_write_pin_low(C1);
  gpio_write_pin_low(C2);
  gpio_write_pin_low(C3);
  gpio_write_pin_low(C4);
  gpio_write_pin_low(C5);
  gpio_write_pin_low(C6);
  gpio_write_pin_low(C7);
  gpio_write_pin_low(C8);
  gpio_write_pin_low(C9);
  gpio_write_pin_low(C10);
  gpio_write_pin_low(C11);
  gpio_write_pin_low(C12);
  gpio_write_pin_low(C13);

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
  // disable UART peripheral
  u->CTRL =0;
  (void)u;
}
/*
static void debug_interrupt(uint8_t ls) {
 // uprintf("ls_p %d \n",ls);
  if (ls & UART_LineStatus_RDR) {
    gpio_write_pin_high(C0);
  } else {
    gpio_write_pin_low(C0);
  }
  if (ls & UART_LineStatus_THRE) {
    gpio_write_pin_high(C1);
  } else {
    gpio_write_pin_low(C1);
  }
  if (ls & UART_LineStatus_TEMT) {
    gpio_write_pin_high(C2);
  } else {
    gpio_write_pin_low(C2);
  }
  if (ls & UART_LineStatus_RxError) {
      gpio_write_pin_high(C3);
    } else {
      gpio_write_pin_low(C3);
  }
  if (ls & UART_LineStatus_TxError) {
    gpio_write_pin_high(C8);
  } else {
    gpio_write_pin_low(C8);
  }
  if (ls & UART_LineStatus_BI) {
    gpio_write_pin_high(C4);
  } else {
    gpio_write_pin_low(C4);
  }
  if (ls & UART_LineStatus_PE) {
    gpio_write_pin_high(C5);
  } else {
    gpio_write_pin_low(C5);
  }
  if (ls & UART_LineStatus_FE) {
    gpio_write_pin_high(C6);
  } else {
    gpio_write_pin_low(C6);
  }
  if (ls & UART_LineStatus_OE) {
    gpio_write_pin_high(C7);
  } else {
    gpio_write_pin_low(C7);
  }

} *//*
static void debug_ii(uint8_t int_id) {
  switch (int_id) {
  case UART_InterruptID_RDA:
      gpio_write_pin_high(C4);
      break;
  case UART_InterruptID_RLS:
      gpio_write_pin_high(C5);
      break;
  case UART_InterruptID_TEMT:
      gpio_write_pin_high(C6);
      break;
  case UART_InterruptID_THRE:
      gpio_write_pin_high(C7);
      break;
  case UART_InterruptID_CTI:
      gpio_write_pin_high(C8);
      break;
  default:
      gpio_write_pin_low(C4);
      gpio_write_pin_low(C5);
      gpio_write_pin_low(C6);
      gpio_write_pin_low(C7);
      gpio_write_pin_low(C8);
      break;
  }
}*/

/**
 * @brief   Error handling routine.
 *
 * @param[in] sdp       pointer to a @p SerialDriver object
 * @param[in] sr        UART SR register value
*/
static void set_error(SerialDriver *sdp, uint8_t ls) {
  eventflags_t sts = 0;

  if(ls & UART_LineStatus_BI){
    sts |= SD_BREAK_DETECTED;
  } 
  if(ls & UART_LineStatus_OE){
    sts |= SD_OVERRUN_ERROR;
  } 
  if (ls & UART_LineStatus_PE){
    sts |= SD_PARITY_ERROR;
  } 
  if (ls & UART_LineStatus_FE){
    sts |= SD_FRAMING_ERROR;
  } 
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
    // Debug isr start
    gpio_write_pin_high(C9);

    #define UART_LS_STATUS (UART_LineStatus_PE |UART_LineStatus_OE | UART_LineStatus_FE | UART_LineStatus_BI | UART_LineStatus_RxError)
    sn32_uart_t *u = sdp->uart;
    uint32_t ii_buf;

    uint32_t ls = u->LS;
        if (ls & UART_LineStatus_BI) {
            gpio_write_pin_high(C6);
            set_error(sdp, UART_LineStatus_BI);
            gpio_write_pin_low(C6);
        }
        if (ls & (UART_LineStatus_PE | UART_LineStatus_FE)) {
          gpio_write_pin_high(C11);
            set_error(sdp, ls);
            gpio_write_pin_low(C11);
        }

    while (ls & (UART_LineStatus_RDR)) {
        gpio_write_pin_high(C4);

        if (ls & UART_LineStatus_OE) {
            gpio_write_pin_high(C5);
            /*osalSysLockFromISR();
            set_error(sdp, UART_LineStatus_OE);
            osalSysUnlockFromISR();
            //u->FIFOCTRL |= UART_RxFIFO_Reset;
            ls = u->LS;
            gpio_write_pin_low(C4);*/
            set_error(sdp, UART_LineStatus_OE);

            gpio_write_pin_low(C5);
            gpio_write_pin_low(C4);
            break;
           // continue;
        }
        gpio_write_pin_high(C12);
        osalSysLockFromISR();
        if (iqIsEmptyI(&sdp->iqueue))
          chnAddFlagsI(sdp, CHN_INPUT_AVAILABLE);
        iqPutI(&sdp->iqueue, (uint8_t)u->RB);
        osalSysUnlockFromISR();
        gpio_write_pin_low(C12);
        ls = u->LS;
        gpio_write_pin_low(C4);
    }

    uint32_t ie = u->IE;

    if (ie & UART_TransmitterHoldingEmpty) {
        if (ls & UART_LineStatus_THRE) {
            msg_t b;
            osalSysLockFromISR();
            b = oqGetI(&sdp->oqueue);
            osalSysUnlockFromISR();
            if (b < MSG_OK) {
                gpio_write_pin_high(C7);
                osalSysLockFromISR();
                chnAddFlagsI(sdp, CHN_OUTPUT_EMPTY);
                osalSysUnlockFromISR();
                u->IE &= ~(UART_TransmitterHoldingEmpty);
                gpio_write_pin_low(C7);
            } else {
                gpio_write_pin_high(C8);
                osalSysLockFromISR();
                u->TH = b;
                osalSysUnlockFromISR();
                gpio_write_pin_low(C8);
            }

            //osalSysUnlockFromISR();
        }
    }

    if ((ie & UART_TransmitterEmpty) && (ls & UART_LineStatus_TEMT)) {
        osalSysLockFromISR();
        if (oqIsEmptyI(&sdp->oqueue)) {
            gpio_write_pin_high(C10);
            chnAddFlagsI(sdp, CHN_TRANSMISSION_END);
            u->IE &= ~(UART_TransmitterEmpty);
            gpio_write_pin_low(C10);
        }
        osalSysUnlockFromISR();
    }
    // Clear pending interrupts
    ii_buf = u->II;
    (void)ii_buf;
    // Debug isr end
    gpio_write_pin_low(C9);
}

static void load(SerialDriver *sdp) {
    sn32_uart_t *u = sdp->uart;
    // Debug load start
    gpio_write_pin_high(C0);
    //osalSysLock();
   // u->CTRL |= UART_TxEnable;
    //u->CTRL_b.TXEN=1;
    u->IE |= (UART_TransmitterHoldingEmpty | UART_TransmitterEmpty);
    //osalSysUnlock();
    // Debug load end
    gpio_write_pin_low(C0);
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
      sn32_uart_t *u = sdp->uart;
      u->CTRL = (UART_Enable| UART_RxEnable |UART_TxEnable);
      //u->CTRL_b.TXEN=0;
      u->IE = (UART_ReceiveDataAvailable | UART_ReceiveLine);
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
