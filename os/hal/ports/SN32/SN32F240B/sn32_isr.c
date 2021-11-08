/*
    ChibiOS - Copyright (C) 2006..2018 Giovanni Di Sirio

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
 * @file    SN32F24xx/sn32_isr.h
 * @brief   SN32F24xx ISR handler code.
 *
 * @{
 */

#include "hal.h"

/*===========================================================================*/
/* Driver local definitions.                                                 */
/*===========================================================================*/

/*===========================================================================*/
/* Driver exported variables.                                                */
/*===========================================================================*/

/*===========================================================================*/
/* Driver local variables.                                                   */
/*===========================================================================*/

/*===========================================================================*/
/* Driver local functions.                                                   */
/*===========================================================================*/

/*===========================================================================*/
/* Driver interrupt handlers.                                                */
/*===========================================================================*/


#if HAL_USE_GPT || HAL_USE_PWM || defined(__DOXYGEN__)
/**
 * @brief   CT16B0 interrupt handler.
 *
 * @isr
 */
OSAL_IRQ_HANDLER(Vector7C) {

  OSAL_IRQ_PROLOGUE();

#if HAL_USE_GPT
#if SN32_GPT_USE_CT16B0
  gpt_lld_serve_interrupt(&GPTD1);
#endif
#endif
  OSAL_IRQ_EPILOGUE();
}

/**
 * @brief   CT16B1 interrupt handler.
 *
 * @isr
 */
OSAL_IRQ_HANDLER(Vector80) {

  OSAL_IRQ_PROLOGUE();

#if HAL_USE_GPT
#if SN32_GPT_USE_CT16B1
  gpt_lld_serve_interrupt(&GPTD2);
#endif
#endif
#if HAL_USE_PWM
#if SN32_PWM_USE_CT16B1
  pwm_lld_serve_interrupt(&PWMD1);
#endif
#endif

  OSAL_IRQ_EPILOGUE();
}
#endif /* HAL_USE_GPT || HAL_USE_PWM */

/*===========================================================================*/
/* Driver exported functions.                                                */
/*===========================================================================*/

/**
 * @brief   Enables IRQ sources.
 *
 * @notapi
 */
void irqInit(void) {

#if HAL_USE_GPT || HAL_USE_PWM || defined(__DOXYGEN__)
#if SN32_GPT_USE_CT16B0
  nvicEnableVector(SN32_CT16B0_NUMBER,  SN32_GPT_CT16B0_IRQ_PRIORITY);
#endif
#if SN32_GPT_USE_CT16B1
  nvicEnableVector(SN32_CT16B1_NUMBER,  SN32_GPT_CT16B1_IRQ_PRIORITY);
#endif
#if SN32_PWM_USE_CT16B1
  nvicEnableVector(SN32_CT16B1_NUMBER,  SN32_PWM_CT16B1_IRQ_PRIORITY);
#endif
#endif

}

/**
 * @brief   Disables IRQ sources.
 *
 * @notapi
 */
void irqDeinit(void) {

#if HAL_USE_GPT || HAL_USE_PWM || defined(__DOXYGEN__)
  nvicDisableVector(SN32_CT16B0_NUMBER);
  nvicDisableVector(SN32_CT16B1_NUMBER);
#endif
}

/** @} */
