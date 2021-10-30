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
 * @file    CT/hal_pwm_lld.c
 * @brief   SN32 PWM subsystem low level driver header.
 *
 * @addtogroup PWM
 * @{
 */

#include "hal.h"

#if HAL_USE_PWM || defined(__DOXYGEN__)

/*===========================================================================*/
/* Driver local definitions.                                                 */
/*===========================================================================*/

/*===========================================================================*/
/* Driver exported variables.                                                */
/*===========================================================================*/

/**
 * @brief   PWMD1 driver identifier.
 * @note    The driver PWMD1 allocates the complex timer CT16B1 when enabled.
 */
#if SN32_PWM_USE_CT16B1 || defined(__DOXYGEN__)
PWMDriver PWMD1;
#endif

/*===========================================================================*/
/* Driver local variables and types.                                         */
/*===========================================================================*/

/*===========================================================================*/
/* Driver local functions.                                                   */
/*===========================================================================*/

/*===========================================================================*/
/* Driver interrupt handlers.                                                */
/*===========================================================================*/

#if SN32_PWM_USE_CT16B1 || defined(__DOXYGEN__)
#if !defined(SN32_CT16B1_SUPPRESS_ISR)
#if !defined(SN32_CT16B1_HANDLER)
#error "SN32_CT16B1_HANDLER not defined"
#endif
/**
 * @brief   CT16B1 interrupt handler.
 *
 * @isr
 */
OSAL_IRQ_HANDLER(SN32_CT16B1_HANDLER) {

  OSAL_IRQ_PROLOGUE();

  pwm_lld_serve_interrupt(&PWMD1);

  OSAL_IRQ_EPILOGUE();
}
#endif /* !defined(SN32_CT16B0_SUPPRESS_ISR) */
#endif /* SN32_PWM_USE_CT16B1 */

/*===========================================================================*/
/* Driver exported functions.                                                */
/*===========================================================================*/

/**
 * @brief   Low level PWM driver initialization.
 *
 * @notapi
 */
void pwm_lld_init(void) {

#if SN32_PWM_USE_CT16B1
  /* Driver initialization.*/
  pwmObjectInit(&PWMD1);
  PWMD1.channels = SN32_CT16B1_CHANNELS;
  PWMD1.ct = SN32_CT16B1;
#endif
}

/**
 * @brief   Configures and activates the PWM peripheral.
 * @note    Starting a driver that is already in the @p PWM_READY state
 *          disables all the active channels.
 *
 * @param[in] pwmp      pointer to a @p PWMDriver object
 *
 * @notapi
 */
void pwm_lld_start(PWMDriver *pwmp) {
  uint32_t psc;
  uint32_t pwmctrl;

  if (pwmp->state == PWM_STOP) {
    /* Clock activation and timer reset.*/
#if SN32_PWM_USE_CT16B1
    if (&PWMD1 == pwmp) {
      CT16B1_Init();
      CT16B1_Reset();
#if !defined(SN32_CT16B1_SUPPRESS_ISR)
      nvicEnableVector(SN32_CT16B1_NUMBER, SN32_PWM_CT16B1_IRQ_PRIORITY);
#endif
    pwmp->clock = SystemCoreClock;
    }
#endif

    /* All channels configured in PWM1 mode with preload enabled and will
       stay that way until the driver is stopped.*/
    pwmp->ct->PWMCTRL = 1;
    pwmp->ct->PWMCTRL2 = 1;
    pwmp->ct->PWMENB = 1;
    pwmp->ct->PWMIOENB = 1;
  }
  else {
    /* Driver re-configuration scenario, it must be stopped first.*/
    pwmp->ct->TMRCTRL = CT16_CEN_DIS;       /* Timer disabled.              */
    pwmp->ct->TC = 0;;                      /* Counter reset to zero.       */
  }

  /* Timer configuration.*/
  psc = (pwmp->clock / pwmp->config->frequency) - 1;
  osalDbgAssert((psc <= 0xFFFF) &&     /* Prescaler calulation.             */
                ((psc + 1) * pwmp->config->frequency) == pwmp->clock,
                "invalid frequency");
  pwmp->ct->PRE  = psc;
  pwmp->ct->CT   = pwmp->period - 1;

  /* Output enables and polarities setup.*/
  pwmctrl = 0;
  pwmctrl2 = 0;
  switch (pwmp->config->channels[0].mode & PWM_OUTPUT_MASK) {
  case PWM_OUTPUT_ACTIVE_LOW:
    pwmctrl |= CT16_PWM0MODE_1;
    /* Falls through.*/
  case PWM_OUTPUT_ACTIVE_HIGH:
    pwmctrl |= CT16_PWM0MODE_2;
    /* Falls through.*/
  default:
    ;
  }
  switch (pwmp->config->channels[1].mode & PWM_OUTPUT_MASK) {
  case PWM_OUTPUT_ACTIVE_LOW:
    pwmctrl |= CT16_PWM1MODE_1;
    /* Falls through.*/
  case PWM_OUTPUT_ACTIVE_HIGH:
    pwmctrl |= CT16_PWM1MODE_2;
    /* Falls through.*/
  default:
    ;
  }
  switch (pwmp->config->channels[2].mode & PWM_OUTPUT_MASK) {
  case PWM_OUTPUT_ACTIVE_LOW:
    pwmctrl |= CT16_PWM2MODE_1;
    /* Falls through.*/
  case PWM_OUTPUT_ACTIVE_HIGH:
    pwmctrl |= CT16_PWM2MODE_2;
    /* Falls through.*/
  default:
    ;
  }
  switch (pwmp->config->channels[3].mode & PWM_OUTPUT_MASK) {
  case PWM_OUTPUT_ACTIVE_LOW:
    pwmctrl |= CT16_PWM3MODE_1;
    /* Falls through.*/
  case PWM_OUTPUT_ACTIVE_HIGH:
    pwmctrl |= CT16_PWM3MODE_2;
    /* Falls through.*/
  default:
    ;
  }
  switch (pwmp->config->channels[4].mode & PWM_OUTPUT_MASK) {
  case PWM_OUTPUT_ACTIVE_LOW:
    pwmctrl |= CT16_PWM4MODE_1;
    /* Falls through.*/
  case PWM_OUTPUT_ACTIVE_HIGH:
    pwmctrl |= CT16_PWM4MODE_2;
    /* Falls through.*/
  default:
    ;
  }
  switch (pwmp->config->channels[5].mode & PWM_OUTPUT_MASK) {
  case PWM_OUTPUT_ACTIVE_LOW:
    pwmctrl |= CT16_PWM5MODE_1;
    /* Falls through.*/
  case PWM_OUTPUT_ACTIVE_HIGH:
    pwmctrl |= CT16_PWM5MODE_2;
    /* Falls through.*/
  default:
    ;
  }
  switch (pwmp->config->channels[6].mode & PWM_OUTPUT_MASK) {
  case PWM_OUTPUT_ACTIVE_LOW:
    pwmctrl |= CT16_PWM6MODE_1;
    /* Falls through.*/
  case PWM_OUTPUT_ACTIVE_HIGH:
    pwmctrl |= CT16_PWM6MODE_2;
    /* Falls through.*/
  default:
    ;
  }
  switch (pwmp->config->channels[7].mode & PWM_OUTPUT_MASK) {
  case PWM_OUTPUT_ACTIVE_LOW:
    pwmctrl |= CT16_PWM7MODE_1;
    /* Falls through.*/
  case PWM_OUTPUT_ACTIVE_HIGH:
    pwmctrl |= CT16_PWM7MODE_2;
    /* Falls through.*/
  default:
    ;
  }
  switch (pwmp->config->channels[8].mode & PWM_OUTPUT_MASK) {
  case PWM_OUTPUT_ACTIVE_LOW:
    pwmctrl |= CT16_PWM8MODE_1;
    /* Falls through.*/
  case PWM_OUTPUT_ACTIVE_HIGH:
    pwmctrl |= CT16_PWM8MODE_2;
    /* Falls through.*/
  default:
    ;
  }
  switch (pwmp->config->channels[9].mode & PWM_OUTPUT_MASK) {
  case PWM_OUTPUT_ACTIVE_LOW:
    pwmctrl |= CT16_PWM9MODE_1;
    /* Falls through.*/
  case PWM_OUTPUT_ACTIVE_HIGH:
    pwmctrl |= CT16_PWM9MODE_2;
    /* Falls through.*/
  default:
    ;
  }
  switch (pwmp->config->channels[10].mode & PWM_OUTPUT_MASK) {
  case PWM_OUTPUT_ACTIVE_LOW:
    pwmctrl |= CT16_PWM10MODE_1;
    /* Falls through.*/
  case PWM_OUTPUT_ACTIVE_HIGH:
    pwmctrl |= CT16_PWM10MODE_2;
    /* Falls through.*/
  default:
    ;
  }
  switch (pwmp->config->channels[11].mode & PWM_OUTPUT_MASK) {
  case PWM_OUTPUT_ACTIVE_LOW:
    pwmctrl |= CT16_PWM11MODE_1;
    /* Falls through.*/
  case PWM_OUTPUT_ACTIVE_HIGH:
    pwmctrl |= CT16_PWM11MODE_2;
    /* Falls through.*/
  default:
    ;
  }
  switch (pwmp->config->channels[12].mode & PWM_OUTPUT_MASK) {
  case PWM_OUTPUT_ACTIVE_LOW:
    pwmctrl |= CT16_PWM12MODE_1;
    /* Falls through.*/
  case PWM_OUTPUT_ACTIVE_HIGH:
    pwmctrl |= CT16_PWM12MODE_2;
    /* Falls through.*/
  default:
    ;
  }
  switch (pwmp->config->channels[13].mode & PWM_OUTPUT_MASK) {
  case PWM_OUTPUT_ACTIVE_LOW:
    pwmctrl |= CT16_PWM13MODE_1;
    /* Falls through.*/
  case PWM_OUTPUT_ACTIVE_HIGH:
    pwmctrl |= CT16_PWM13MODE_2;
    /* Falls through.*/
  default:
    ;
  }
  switch (pwmp->config->channels[14].mode & PWM_OUTPUT_MASK) {
  case PWM_OUTPUT_ACTIVE_LOW:
    pwmctrl |= CT16_PWM14MODE_1;
    /* Falls through.*/
  case PWM_OUTPUT_ACTIVE_HIGH:
    pwmctrl |= CT16_PWM14MODE_2;
    /* Falls through.*/
  default:
    ;
  }
  switch (pwmp->config->channels[15].mode & PWM_OUTPUT_MASK) {
  case PWM_OUTPUT_ACTIVE_LOW:
    pwmctrl |= CT16_PWM15MODE_1;
    /* Falls through.*/
  case PWM_OUTPUT_ACTIVE_HIGH:
    pwmctrl |= CT16_PWM15MODE_2;
    /* Falls through.*/
  default:
    ;
  }
  switch (pwmp->config->channels[16].mode & PWM_OUTPUT_MASK) {
  case PWM_OUTPUT_ACTIVE_LOW:
    pwmctrl2 |= CT16_PWM16MODE_1;
    /* Falls through.*/
  case PWM_OUTPUT_ACTIVE_HIGH:
    pwmctrl2 |= CT16_PWM16MODE_2;
    /* Falls through.*/
  default:
    ;
  }
  switch (pwmp->config->channels[17].mode & PWM_OUTPUT_MASK) {
  case PWM_OUTPUT_ACTIVE_LOW:
    pwmctrl2 |= CT16_PWM17MODE_1;
    /* Falls through.*/
  case PWM_OUTPUT_ACTIVE_HIGH:
    pwmctrl2 |= CT16_PWM17MODE_2;
    /* Falls through.*/
  default:
    ;
  }
  switch (pwmp->config->channels[18].mode & PWM_OUTPUT_MASK) {
  case PWM_OUTPUT_ACTIVE_LOW:
    pwmctrl2 |= CT16_PWM18MODE_1;
    /* Falls through.*/
  case PWM_OUTPUT_ACTIVE_HIGH:
    pwmctrl2 |= CT16_PWM18MODE_2;
    /* Falls through.*/
  default:
    ;
  }
  switch (pwmp->config->channels[19].mode & PWM_OUTPUT_MASK) {
  case PWM_OUTPUT_ACTIVE_LOW:
    pwmctrl2 |= CT16_PWM19MODE_1;
    /* Falls through.*/
  case PWM_OUTPUT_ACTIVE_HIGH:
    pwmctrl2 |= CT16_PWM19MODE_2;
    /* Falls through.*/
  default:
    ;
  }
  switch (pwmp->config->channels[20].mode & PWM_OUTPUT_MASK) {
  case PWM_OUTPUT_ACTIVE_LOW:
    pwmctrl2 |= CT16_PWM20MODE_1;
    /* Falls through.*/
  case PWM_OUTPUT_ACTIVE_HIGH:
    pwmctrl2 |= CT16_PWM20MODE_2;
    /* Falls through.*/
  default:
    ;
  }
  switch (pwmp->config->channels[21].mode & PWM_OUTPUT_MASK) {
  case PWM_OUTPUT_ACTIVE_LOW:
    pwmctrl2 |= CT16_PWM21MODE_1;
    /* Falls through.*/
  case PWM_OUTPUT_ACTIVE_HIGH:
    pwmctrl2 |= CT16_PWM21MODE_2;
    /* Falls through.*/
  default:
    ;
  }
  switch (pwmp->config->channels[22].mode & PWM_OUTPUT_MASK) {
  case PWM_OUTPUT_ACTIVE_LOW:
    pwmctrl2 |= CT16_PWM22MODE_1;
    /* Falls through.*/
  case PWM_OUTPUT_ACTIVE_HIGH:
    pwmctrl2 |= CT16_PWM22MODE_2;
    /* Falls through.*/
  default:
    ;
  }
  switch (pwmp->config->channels[23].mode & PWM_OUTPUT_MASK) {
  case PWM_OUTPUT_ACTIVE_LOW:
    pwmctrl2 |= CT16_PWM23MODE_1;
    /* Falls through.*/
  case PWM_OUTPUT_ACTIVE_HIGH:
    pwmctrl2 |= CT16_PWM23MODE_2;
    /* Falls through.*/
  default:
    ;
  }

  pwmp->ct->PWMCTRL  = pwmctrl;
  pwmp->ct->PWMCTRL  = pwmctrl2;
  pwmp->ct->IC       = 1;                   /* Clear pending IRQs.          */

  /* Timer configured and started.*/
  pwmp->ct->TMRCTRL |= mskCT16_CEN_EN;
}

/**
 * @brief   Deactivates the PWM peripheral.
 *
 * @param[in] pwmp      pointer to a @p PWMDriver object
 *
 * @notapi
 */
void pwm_lld_stop(PWMDriver *pwmp) {

  /* If in ready state then disables the PWM clock.*/
  if (pwmp->state == PWM_READY) {
    pwmp->ct->TMRCTRL = CT16_CEN_DIS;       /* Timer disabled.              */
    pwmp->ct->IC = 1;                       /* Clear pending IRQs.          */

#if SN32_PWM_USE_CT16B1
    if (&PWMD1 == pwmp) {
#if !defined(SN32_CT16B1_SUPPRESS_ISR)
      nvicDisableVector(SN32_CT16B1_NUMBER);
#endif
      SN_SYS1->AHBCLKEN_b.CT16B1CLKEN = DISABLE;
    }
#endif
  }
}

/**
 * @brief   Enables a PWM channel.
 * @pre     The PWM unit must have been activated using @p pwmStart().
 * @post    The channel is active using the specified configuration.
 * @note    The function has effect at the next cycle start.
 * @note    Channel notification is not enabled.
 *
 * @param[in] pwmp      pointer to a @p PWMDriver object
 * @param[in] channel   PWM channel identifier (0...channels-1)
 * @param[in] width     PWM pulse width as clock pulses number
 *
 * @notapi
 */
void pwm_lld_enable_channel(PWMDriver *pwmp,
                            pwmchannel_t channel,
                            pwmcnt_t width) {

  /* Changing channel duty cycle on the fly.*/
  pwmp->ct->MRchannel = width;
}

/**
 * @brief   Disables a PWM channel and its notification.
 * @pre     The PWM unit must have been activated using @p pwmStart().
 * @post    The channel is disabled and its output line returned to the
 *          idle state.
 * @note    The function has effect at the next cycle start.
 *
 * @param[in] pwmp      pointer to a @p PWMDriver object
 * @param[in] channel   PWM channel identifier (0...channels-1)
 *
 * @notapi
 */
void pwm_lld_disable_channel(PWMDriver *pwmp, pwmchannel_t channel) {

  pwmp->ct->MRchannel = 0;
  pwmp->ct->PWMIOENB |= mskCT16_PWMchannelIOEN_DIS;
}

/**
 * @brief   Enables the periodic activation edge notification.
 * @pre     The PWM unit must have been activated using @p pwmStart().
 * @note    If the notification is already enabled then the call has no effect.
 *
 * @param[in] pwmp      pointer to a @p PWMDriver object
 *
 * @notapi
 */
void pwm_lld_enable_periodic_notification(PWMDriver *pwmp) {
  /* Not supported */
}

/**
 * @brief   Disables the periodic activation edge notification.
 * @pre     The PWM unit must have been activated using @p pwmStart().
 * @note    If the notification is already disabled then the call has no effect.
 *
 * @param[in] pwmp      pointer to a @p PWMDriver object
 *
 * @notapi
 */
void pwm_lld_disable_periodic_notification(PWMDriver *pwmp) {
  /* Not supported */
}

/**
 * @brief   Enables a channel de-activation edge notification.
 * @pre     The PWM unit must have been activated using @p pwmStart().
 * @pre     The channel must have been activated using @p pwmEnableChannel().
 * @note    If the notification is already enabled then the call has no effect.
 *
 * @param[in] pwmp      pointer to a @p PWMDriver object
 * @param[in] channel   PWM channel identifier (0...channels-1)
 *
 * @notapi
 */
void pwm_lld_enable_channel_notification(PWMDriver *pwmp,
                                         pwmchannel_t channel) {
  /* Not supported */
}

/**
 * @brief   Disables a channel de-activation edge notification.
 * @pre     The PWM unit must have been activated using @p pwmStart().
 * @pre     The channel must have been activated using @p pwmEnableChannel().
 * @note    If the notification is already disabled then the call has no effect.
 *
 * @param[in] pwmp      pointer to a @p PWMDriver object
 * @param[in] channel   PWM channel identifier (0...channels-1)
 *
 * @notapi
 */
void pwm_lld_disable_channel_notification(PWMDriver *pwmp,
                                          pwmchannel_t channel) {
  /* Not supported */
}

/**
 * @brief   Common CT IRQ handler.
 * @note    It is assumed that the various sources are only activated if the
 *          associated callback pointer is not equal to @p NULL in order to not
 *          perform an extra check in a potentially critical interrupt handler.
 *
 * @param[in] pwmp      pointer to a @p PWMDriver object
 *
 * @notapi
 */
void pwm_lld_serve_interrupt(PWMDriver *pwmp) {
  uint32_t ris;

  ris  = pwmp->ct->RIS;
  if (((ris & mskCT16_MR0IF) != 0) &&
      (pwmp->config->channels[0].callback != NULL))
    pwmp->config->channels[0].callback(pwmp);
  if (((ris & mskCT16_MR1IF) != 0) &&
      (pwmp->config->channels[1].callback != NULL))
    pwmp->config->channels[1].callback(pwmp);
  if (((ris & mskCT16_MR2IF) != 0) &&
      (pwmp->config->channels[2].callback != NULL))
    pwmp->config->channels[2].callback(pwmp);
  if (((ris & mskCT16_MR3IF) != 0) &&
      (pwmp->config->channels[3].callback != NULL))
    pwmp->config->channels[3].callback(pwmp);
  if (((ris & mskCT16_MR4IF) != 0) &&
      (pwmp->config->channels[4].callback != NULL))
    pwmp->config->channels[4].callback(pwmp);
  if (((ris & mskCT16_MR5IF) != 0) &&
      (pwmp->config->channels[5].callback != NULL))
    pwmp->config->channels[5].callback(pwmp);
  if (((ris & mskCT16_MR6IF) != 0) &&
      (pwmp->config->channels[6].callback != NULL))
    pwmp->config->channels[6].callback(pwmp);
  if (((ris & mskCT16_MR7IF) != 0) &&
      (pwmp->config->channels[7].callback != NULL))
    pwmp->config->channels[7].callback(pwmp);
  if (((ris & mskCT16_MR8IF) != 0) &&
      (pwmp->config->channels[8].callback != NULL))
    pwmp->config->channels[8].callback(pwmp);
  if (((ris & mskCT16_MR9IF) != 0) &&
      (pwmp->config->channels[9].callback != NULL))
    pwmp->config->channels[9].callback(pwmp);
  if (((ris & mskCT16_MR10IF) != 0) &&
      (pwmp->config->channels[10].callback != NULL))
    pwmp->config->channels[10].callback(pwmp);
  if (((ris & mskCT16_MR11IF) != 0) &&
      (pwmp->config->channels[11].callback != NULL))
    pwmp->config->channels[11].callback(pwmp);
  if (((ris & mskCT16_MR12IF) != 0) &&
      (pwmp->config->channels[12].callback != NULL))
    pwmp->config->channels[12].callback(pwmp);
  if (((ris & mskCT16_MR13IF) != 0) &&
      (pwmp->config->channels[13].callback != NULL))
    pwmp->config->channels[13].callback(pwmp);
  if (((ris & mskCT16_MR14IF) != 0) &&
      (pwmp->config->channels[14].callback != NULL))
    pwmp->config->channels[14].callback(pwmp);
  if (((ris & mskCT16_MR15IF) != 0) &&
      (pwmp->config->channels[15].callback != NULL))
    pwmp->config->channels[15].callback(pwmp);
  if (((ris & mskCT16_MR16IF) != 0) &&
      (pwmp->config->channels[16].callback != NULL))
    pwmp->config->channels[16].callback(pwmp);
  if (((ris & mskCT16_MR17IF) != 0) &&
      (pwmp->config->channels[17].callback != NULL))
    pwmp->config->channels[17].callback(pwmp);
  if (((ris & mskCT16_MR18IF) != 0) &&
      (pwmp->config->channels[18].callback != NULL))
    pwmp->config->channels[18].callback(pwmp);
  if (((ris & mskCT16_MR19IF) != 0) &&
      (pwmp->config->channels[19].callback != NULL))
    pwmp->config->channels[19].callback(pwmp);
  if (((ris & mskCT16_MR20IF) != 0) &&
      (pwmp->config->channels[20].callback != NULL))
    pwmp->config->channels[20].callback(pwmp);
  if (((ris & mskCT16_MR21IF) != 0) &&
      (pwmp->config->channels[21].callback != NULL))
    pwmp->config->channels[21].callback(pwmp);
  if (((ris & mskCT16_MR22IF) != 0) &&
      (pwmp->config->channels[22].callback != NULL))
    pwmp->config->channels[22].callback(pwmp);
  if (((ris & mskCT16_MR23IF) != 0) &&
      (pwmp->config->channels[23].callback != NULL))
    pwmp->config->channels[23].callback(pwmp);
  if (((ris & gptp->ct->IC) != 0) && (pwmp->config->callback != NULL))
    pwmp->config->callback(pwmp);
}

#endif /* HAL_USE_PWM */

/** @} */
