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
 * @file    USBv1/hal_usb_lld.c
 * @brief   SN32 USB subsystem low level driver source.
 *
 * @addtogroup USB
 * @{
 */

#include <string.h>

#include "hal.h"

#if HAL_USE_USB || defined(__DOXYGEN__)

/*===========================================================================*/
/* Driver local definitions.                                                 */
/*===========================================================================*/


/*===========================================================================*/
/* Driver exported variables.                                                */
/*===========================================================================*/

/** @brief USB1 driver identifier.*/
#if SN32_USB_USE_USB1 || defined(__DOXYGEN__)
USBDriver USBD1;
#endif

/*===========================================================================*/
/* Driver local variables and types.                                         */
/*===========================================================================*/

/**
 * @brief   EP0 state.
 * @note    It is an union because IN and OUT endpoints are never used at the
 *          same time for EP0.
 */
static union {
  /**
   * @brief   IN EP0 state.
   */
  USBInEndpointState in;
  /**
   * @brief   OUT EP0 state.
   */
  USBOutEndpointState out;
} ep0_state;

/**
 * @brief   Buffer for the EP0 setup packets.
 */
static uint8_t ep0setup_buffer[8];

/**
 * @brief   EP0 initialization structure.
 */
static const USBEndpointConfig ep0config = {
  USB_EP_MODE_TYPE_CTRL,
  _usb_ep0setup,
  _usb_ep0in,
  _usb_ep0out,
  0x40,
  0x40,
  &ep0_state.in,
  &ep0_state.out,
  1,
  ep0setup_buffer
};

/*===========================================================================*/
/* Driver local functions.                                                   */
/*===========================================================================*/

/**
 * @brief   Resets the packet memory allocator.
 *
 * @param[in] usbp      pointer to the @p USBDriver object
 */
static void usb_pm_reset(USBDriver *usbp) {

  /* The first 64 bytes are reserved for the descriptors table. The effective
     available RAM for endpoint buffers is just 448 bytes.*/
  usbp->pmnext = 64;
}

/**
 * @brief   Resets the packet memory allocator.
 *
 * @param[in] usbp      pointer to the @p USBDriver object
 * @param[in] size      size of the packet buffer to allocate
 * @return              The packet buffer address.
 */
static uint32_t usb_pm_alloc(USBDriver *usbp, size_t size) {
  uint32_t next;

  next = usbp->pmnext;
  usbp->pmnext += (size + 1) & ~1;
  osalDbgAssert(usbp->pmnext <= SN32_USB_PMA_SIZE, "PMA overflow");
  return next;
}

/**
 * @brief   Reads from a dedicated packet buffer.
 *
 * @param[in] ep        endpoint number
 * @param[out] buf      buffer where to copy the packet data
 * @return              The size of the receivee packet.
 *
 * @notapi
 */
static size_t usb_packet_read_to_buffer(usbep_t ep, uint8_t *buf) {
  size_t i, n;
  sn32_usb_descriptor_t *udp;
  if(ep == 0) {
    udp = USB_GET_CTRL_DESCRIPTOR();
  }
  else {
    udp = USB_GET_DESCRIPTOR(ep);
  }
  sn32_usb_pma_t *pmap = USB_ADDR2PTR(udp->RXADDR0);
  n = (size_t)udp->RXCOUNT0 & RXCOUNT_COUNT_MASK;
  i = n;

#if SN32_USB_USE_FAST_COPY
  while (i >= 16) {
    uint32_t w;

    w = *(pmap + 0);
    *(buf + 0) = (uint8_t)w;
    *(buf + 1) = (uint8_t)(w >> 8);
    w = *(pmap + 1);
    *(buf + 2) = (uint8_t)w;
    *(buf + 3) = (uint8_t)(w >> 8);
    w = *(pmap + 2);
    *(buf + 4) = (uint8_t)w;
    *(buf + 5) = (uint8_t)(w >> 8);
    w = *(pmap + 3);
    *(buf + 6) = (uint8_t)w;
    *(buf + 7) = (uint8_t)(w >> 8);
    w = *(pmap + 4);
    *(buf + 8) = (uint8_t)w;
    *(buf + 9) = (uint8_t)(w >> 8);
    w = *(pmap + 5);
    *(buf + 10) = (uint8_t)w;
    *(buf + 11) = (uint8_t)(w >> 8);
    w = *(pmap + 6);
    *(buf + 12) = (uint8_t)w;
    *(buf + 13) = (uint8_t)(w >> 8);
    w = *(pmap + 7);
    *(buf + 14) = (uint8_t)w;
    *(buf + 15) = (uint8_t)(w >> 8);

    i -= 16;
    buf += 16;
    pmap += 8;
  }
#endif /* SN32_USB_USE_FAST_COPY */

  while (i >= 2) {
    uint32_t w = *pmap++;
    *buf++ = (uint8_t)w;
    *buf++ = (uint8_t)(w >> 8);
    i -= 2;
  }

  if (i >= 1) {
    *buf = (uint8_t)*pmap;
  }

  return n;
}

/**
 * @brief   Writes to a dedicated packet buffer.
 *
 * @param[in] ep        endpoint number
 * @param[in] buf       buffer where to fetch the packet data
 * @param[in] n         maximum number of bytes to copy. This value must
 *                      not exceed the maximum packet size for this endpoint.
 *
 * @notapi
 */
static void usb_packet_write_from_buffer(usbep_t ep,
                                         const uint8_t *buf,
                                         size_t n) {
  sn32_usb_descriptor_t *udp;
  if(ep == 0) {
    udp = USB_GET_CTRL_DESCRIPTOR();
  }
  else {
    udp = USB_GET_DESCRIPTOR(ep);
  }
  sn32_usb_pma_t *pmap = USB_ADDR2PTR(udp->TXADDR0);
  int i = (int)n;

  udp->TXCOUNT0 = (sn32_usb_pma_t)n;

#if SN32_USB_USE_FAST_COPY
  while (i >= 16) {
    uint32_t w;

    w  = *(buf + 0);
    w |= *(buf + 1) << 8;
    *(pmap + 0) = (sn32_usb_pma_t)w;
    w  = *(buf + 2);
    w |= *(buf + 3) << 8;
    *(pmap + 1) = (sn32_usb_pma_t)w;
    w  = *(buf + 4);
    w |= *(buf + 5) << 8;
    *(pmap + 2) = (sn32_usb_pma_t)w;
    w  = *(buf + 6);
    w |= *(buf + 7) << 8;
    *(pmap + 3) = (sn32_usb_pma_t)w;
    w  = *(buf + 8);
    w |= *(buf + 9) << 8;
    *(pmap + 4) = (sn32_usb_pma_t)w;
    w  = *(buf + 10);
    w |= *(buf + 11) << 8;
    *(pmap + 5) = (sn32_usb_pma_t)w;
    w  = *(buf + 12);
    w |= *(buf + 13) << 8;
    *(pmap + 6) = (sn32_usb_pma_t)w;
    w  = *(buf + 14);
    w |= *(buf + 15) << 8;
    *(pmap + 7) = (sn32_usb_pma_t)w;

    i -= 16;
    buf += 16;
    pmap += 8;
  }
#endif /* SN32_USB_USE_FAST_COPY */

  while (i > 0) {
    uint32_t w;

    w  = *buf++;
    w |= *buf++ << 8;
    *pmap++ = (sn32_usb_pma_t)w;
    i -= 2;
  }
}

/**
 * @brief   Common ISR code, serves the EP-related interrupts.
 *
 * @param[in] usbp      pointer to the @p USBDriver object
 * @param[in] ep        endpoint number
 *
 * @notapi
 */
static void usb_serve_endpoints(USBDriver *usbp, uint32_t ep) {
  size_t n;
  uint32_t cfg = SN32_USB->CFG;
  uint32_t status = SN32_USB->INSTS;
  uint8_t ep_out = (cfg & mskEPn_DIR(ep)) == mskEPn_DIR(ep);
  const USBEndpointConfig *epcp = usbp->epc[ep];

  if (status & (mskEP0_IN | mskEPn_NAK(ep) | mskEPn_ACK(ep))) {
    if((ep == 0 ) | (!ep_out)) {

      /* Special case for SetAddress for EP0 */
      if(ep == 0 && (((uint16_t)usbp->setup[0]<<8)|usbp->setup[1]) == 0x0500)
      {
        usbp->address = usbp->setup[2];
        usb_lld_set_address(usbp);
        _usb_isr_invoke_event_cb(usbp, USB_EVENT_ADDRESS);
        usbp->state = USB_SELECTED;
      }

      /* IN endpoint, transmission.*/
      USBInEndpointState *isp = epcp->in_state;

      isp->txcnt += isp->txlast;
      n = isp->txsize - isp->txcnt;
      if (n > 0) {
        /* Transfer not completed, there are more packets to send.*/
        if (n > epcp->in_maxsize)
          n = epcp->in_maxsize;

        /* Writes the packet from the defined buffer.*/
        isp->txbuf += isp->txlast;
        isp->txlast = n;
        usb_packet_write_from_buffer(ep, isp->txbuf, n);

        /* Starting IN operation.*/
        EPCTL_SET_STAT_TX(ep, n);
      }
      else {
        /* Transfer completed, invokes the callback.*/
        _usb_isr_invoke_in_cb(usbp, ep);
      }


      /* Clear the status register.*/
      if(ep == 0) {
        SN32_USB->INSTSC = (mskEP0_IN);
      }
      else if (status & mskEPn_NAK(ep)) {
        SN32_USB->INSTSC = (mskEPn_NAK(ep));
      }
      else if (status & mskEPn_ACK(ep)) {
        SN32_USB->INSTSC = (mskEPn_ACK(ep));
      }
    }
  }
  if (status & (mskEP0_SETUP | mskEP0_OUT | mskEP0_IN_STALL| mskEP0_OUT_STALL | mskEPn_NAK(ep) | mskEPn_ACK(ep))) {
        if((ep == 0) | (ep_out)) {
          /* OUT endpoint, receive.*/
          if(status & mskEP0_SETUP) {
              if (!(status & mskERR_SETUP)) {
                SN32_USB->INSTSC = (mskEP0_SETUP | mskEP0_PRESETUP | mskEP0_OUT_STALL | mskEP0_IN_STALL);
                /* Setup packets handling, setup packets are handled using a
                   specific callback.*/
                _usb_isr_invoke_setup_cb(usbp, 0);
              }
              else {
                SN32_USB->INSTSC = mskERR_SETUP;
                usb_lld_stall_out(usbp, 0);
              }
          }
          else {
            USBOutEndpointState *osp = epcp->out_state;

            /* Reads the packet into the defined buffer.*/
            n = usb_packet_read_to_buffer(ep, osp->rxbuf);
            osp->rxbuf += n;

            /* Transaction data updated.*/
            osp->rxcnt  += n;
            osp->rxsize -= n;
            osp->rxpkts -= 1;

            /* The transaction is completed if the specified number of packets
               has been received or the current packet is a short packet.*/
            if ((n < epcp->out_maxsize) || (osp->rxpkts == 0)) {
              /* Transfer complete, invokes the callback.*/
              _usb_isr_invoke_out_cb(usbp, ep);
            }
            else {
              /* Transfer not complete, there are more packets to receive.*/
              EPCTL_SET_STAT_RX(ep);
            }


            /* Clear the status register.*/
            if (status & mskEPn_NAK(ep)) {
              SN32_USB->INSTSC = (mskEPn_NAK(ep));
            }
            else if (status & mskEPn_ACK(ep)) {
              SN32_USB->INSTSC = (mskEPn_ACK(ep));
            }
          }
        }
    }
}

/*===========================================================================*/
/* Driver interrupt handlers.                                                */
/*===========================================================================*/

#if SN32_USB_USE_USB1 || defined(__DOXYGEN__)
/**
 * @brief   USB interrupt handler.
 *
 * @isr
 */
OSAL_IRQ_HANDLER(SN32_USB_HANDLER) {
  uint32_t insts;
  USBDriver *usbp = &USBD1;

  OSAL_IRQ_PROLOGUE();

  insts = SN32_USB->INSTS;

  /* USB bus reset condition handling.*/
  if (insts & mskBUS_RESET) {
    SN32_USB->INSTSC = mskBUS_RESET;

    _usb_reset(usbp);
  }

  /* USB bus SUSPEND condition handling.*/
  if (insts & mskBUS_SUSPEND) {
    SN32_USB->INSTSC = mskBUS_SUSPEND;
    SN32_USB->CFG &= ~(mskESD_EN|mskPHY_EN);
    _usb_suspend(usbp);
  }

  /* USB bus WAKEUP condition handling.*/
  if (insts & mskBUS_WAKEUP) {
    SN32_USB->CFG |= (mskESD_EN|mskPHY_EN);
    _usb_wakeup(usbp);
    SN32_USB->INSTSC = mskBUS_WAKEUP;
  }

  /* SOF handling.*/
  if (insts & mskUSB_SOF) {
    _usb_isr_invoke_sof_cb(usbp);
    SN32_USB->INSTSC = mskUSB_SOF;
  }

  /* Endpoint 0 events handling.*/
  if (insts & (mskEP0_IN | mskEP0_OUT | mskEP0_SETUP | mskEP0_PRESETUP | mskEP0_OUT_STALL | mskEP0_IN_STALL)) {
    usb_serve_endpoints(usbp, 0);
  }

  /* Endpoint 1-6 events handling.*/
  for (uint8_t ep=1; ep <= USB_ENDPOINTS_NUMBER; ep++) {
    while (insts & (mskEPn_NAK(ep) | mskEPn_ACK(ep))) {
      usb_serve_endpoints(usbp, ep);
    }
  }

  OSAL_IRQ_EPILOGUE();
}
#endif /* SN32_USB_USE_USB1 */

/*===========================================================================*/
/* Driver exported functions.                                                */
/*===========================================================================*/

/**
 * @brief   Low level USB driver initialization.
 *
 * @notapi
 */
void usb_lld_init(void) {

  /* Driver initialization.*/
  usbObjectInit(&USBD1);
}

/**
 * @brief   Configures and activates the USB peripheral.
 *
 * @param[in] usbp      pointer to the @p USBDriver object
 *
 * @notapi
 */
void usb_lld_start(USBDriver *usbp) {

  if (usbp->state == USB_STOP) {
    /* Clock activation.*/
#if SN32_USB_USE_USB1
    if (&USBD1 == usbp) {
      /* USB clock enabled.*/
          sys1EnableUSB();
      /* Powers up the transceiver while holding the USB in reset state.*/
      SN32_USB->SGCTL = mskBUS_J_STATE;
      SN32_USB->CFG = (mskVREG33_EN|mskPHY_EN|mskDPPU_EN|mskSIE_EN|mskESD_EN);
      /* Set up hardware configuration.*/
      SN32_USB->PHYPRM = 0x80000000;
      SN32_USB->PHYPRM2 = 0x00004004;
      /* Enabling the USB IRQ vectors, this also gives enough time to allow
         the transceiver power up (1uS).*/
      SN32_USB->INTEN = (mskBUS_IE|mskUSB_IE|mskEPnACK_EN|mskBUSWK_IE|mskUSB_SOF_IE);
      SN32_USB->INTEN |= mskEP1_NAK_EN;
      SN32_USB->INTEN |= mskEP2_NAK_EN;
      SN32_USB->INTEN |= mskEP3_NAK_EN;
      SN32_USB->INTEN |= mskEP4_NAK_EN;
#if (USB_ENDPOINTS_NUMBER > 4)
      SN32_USB->INTEN |= mskEP5_NAK_EN;
      SN32_USB->INTEN |= mskEP6_NAK_EN;
#endif /* (USB_ENDPOINTS_NUMBER > 4) */

      nvicEnableVector(SN32_USB_NUMBER, SN32_USB_IRQ_PRIORITY);
    }
#endif
    /* Reset procedure enforced on driver start.*/
    usb_lld_reset(usbp);
  }
}

/**
 * @brief   Deactivates the USB peripheral.
 *
 * @param[in] usbp      pointer to the @p USBDriver object
 *
 * @notapi
 */
void usb_lld_stop(USBDriver *usbp) {

  /* If in ready state then disables the USB clock.*/
  if (usbp->state != USB_STOP) {
#if SN32_USB_USE_USB1
    if (&USBD1 == usbp) {
      nvicDisableVector(SN32_USB_NUMBER);
      sys1DisableUSB();
    }
#endif
  }
}

/**
 * @brief   USB low level reset routine.
 *
 * @param[in] usbp      pointer to the @p USBDriver object
 *
 * @notapi
 */
void usb_lld_reset(USBDriver *usbp) {

  /* Post reset initialization.*/
  SN32_USB->INSTSC = (0xFFFFFFFF);
  SN32_USB->ADDR  = 0;
  usb_lld_stall_out(usbp, 0);
  /* Resets the packet memory allocator.*/
  usb_pm_reset(usbp);

  /* EP0 initialization.*/
  usbp->epc[0] = &ep0config;
  usb_lld_init_endpoint(usbp, 0);
}

/**
 * @brief   Sets the USB address.
 *
 * @param[in] usbp      pointer to the @p USBDriver object
 *
 * @notapi
 */
void usb_lld_set_address(USBDriver *usbp) {

  SN32_USB->ADDR = (uint32_t)(usbp->address) | mskUADDR;
}

/**
 * @brief   Enables an endpoint.
 *
 * @param[in] usbp      pointer to the @p USBDriver object
 * @param[in] ep        endpoint number
 *
 * @notapi
 */
void usb_lld_init_endpoint(USBDriver *usbp, usbep_t ep) {
  uint32_t cfg = SN32_USB->CFG;
  sn32_usb_descriptor_t *dp;
  const USBEndpointConfig *epcp = usbp->epc[ep];

  if(ep == 0) {
    dp = USB_GET_CTRL_DESCRIPTOR();
  }
  else {
    dp = USB_GET_DESCRIPTOR(ep);
  }

  /* IN endpoint handling.*/
  if (epcp->in_state != NULL) {
    dp->TXCOUNT0 = 0;
    dp->TXADDR0  = usb_pm_alloc(usbp, epcp->in_maxsize);

  }

  /* OUT endpoint handling.*/
  if (epcp->out_state != NULL) {
    uint16_t nblocks;

    /* Endpoint size and address initialization.*/
    if (epcp->out_maxsize > 62)
      nblocks = (((((epcp->out_maxsize - 1) | 0x1f) + 1) / 32) << 10) |
                0x8000;
    else
      nblocks = ((((epcp->out_maxsize - 1) | 1) + 1) / 2) << 10;
    dp->RXCOUNT0 = nblocks;
    dp->RXADDR0  = usb_pm_alloc(usbp, epcp->out_maxsize);

    cfg |= mskEPn_DIR(ep);
  }

  /* EPCTLx register setup.*/
  SN32_USB->CFG = cfg;
  EPCTL_SET_STAT_RX(ep);
  /* Resetting the data toggling bits for this endpoint.*/
  EPCTL_TOGGLE(ep);
}

/**
 * @brief   Disables all the active endpoints except the endpoint zero.
 *
 * @param[in] usbp      pointer to the @p USBDriver object
 *
 * @notapi
 */
void usb_lld_disable_endpoints(USBDriver *usbp) {
  unsigned i;

  /* Resets the packet memory allocator.*/
  usb_pm_reset(usbp);

  /* Disabling all endpoints.*/
  for (i = 1; i <= USB_ENDPOINTS_NUMBER; i++) {
    EPCTL_TOGGLE(i);
    SN32_USB->EPCTL[i] &= ~mskEPn_ENDP_STATE;
  }
}

/**
 * @brief   Returns the status of an OUT endpoint.
 *
 * @param[in] usbp      pointer to the @p USBDriver object
 * @param[in] ep        endpoint number
 * @return              The endpoint status.
 * @retval EP_STATUS_DISABLED The endpoint is not active.
 * @retval EP_STATUS_STALLED  The endpoint is stalled.
 * @retval EP_STATUS_ACTIVE   The endpoint is active.
 *
 * @notapi
 */
usbepstatus_t usb_lld_get_status_out(USBDriver *usbp, usbep_t ep) {

  (void)usbp;
  if (!(SN32_USB->EPCTL[ep] & mskEPn_ENDP_STATE)) {
    return EP_STATUS_DISABLED;
  }
  else if (SN32_USB->EPCTL[ep] & mskEPn_ENDP_STATE_STALL) {
    return EP_STATUS_STALLED;
  }
  else {
    return EP_STATUS_ACTIVE;
  }
}

/**
 * @brief   Returns the status of an IN endpoint.
 *
 * @param[in] usbp      pointer to the @p USBDriver object
 * @param[in] ep        endpoint number
 * @return              The endpoint status.
 * @retval EP_STATUS_DISABLED The endpoint is not active.
 * @retval EP_STATUS_STALLED  The endpoint is stalled.
 * @retval EP_STATUS_ACTIVE   The endpoint is active.
 *
 * @notapi
 */
usbepstatus_t usb_lld_get_status_in(USBDriver *usbp, usbep_t ep) {

  (void)usbp;
  if (!(SN32_USB->EPCTL[ep] & mskEPn_ENDP_STATE)) {
    return EP_STATUS_DISABLED;
  }
  else if (SN32_USB->EPCTL[ep] & mskEPn_ENDP_STATE_STALL) {
    return EP_STATUS_STALLED;
  }
  else {
    return EP_STATUS_ACTIVE;
  }
}

/**
 * @brief   Reads a setup packet from the dedicated packet buffer.
 * @details This function must be invoked in the context of the @p setup_cb
 *          callback in order to read the received setup packet.
 * @pre     In order to use this function the endpoint must have been
 *          initialized as a control endpoint.
 * @post    The endpoint is ready to accept another packet.
 *
 * @param[in] usbp      pointer to the @p USBDriver object
 * @param[in] ep        endpoint number
 * @param[out] buf      buffer where to copy the packet data
 *
 * @notapi
 */
void usb_lld_read_setup(USBDriver *usbp, usbep_t ep, uint8_t *buf) {
  sn32_usb_pma_t *pmap;
  sn32_usb_descriptor_t *udp;
  uint32_t n;

  (void)usbp;
  if(ep == 0) {
    udp = USB_GET_CTRL_DESCRIPTOR();
  }
  else {
    udp = USB_GET_DESCRIPTOR(ep);
  }
  pmap = USB_ADDR2PTR(udp->RXADDR0);
  for (n = 0; n < 4; n++) {
    *(uint16_t *)buf = (uint16_t)*pmap++;
    buf += 2;
  }
}

/**
 * @brief   Starts a receive operation on an OUT endpoint.
 *
 * @param[in] usbp      pointer to the @p USBDriver object
 * @param[in] ep        endpoint number
 *
 * @notapi
 */
void usb_lld_start_out(USBDriver *usbp, usbep_t ep) {
  USBOutEndpointState *osp = usbp->epc[ep]->out_state;

  /* Transfer initialization.*/
  if (osp->rxsize == 0)         /* Special case for zero sized packets.*/
    osp->rxpkts = 1;
  else
    osp->rxpkts = (uint16_t)((osp->rxsize + usbp->epc[ep]->out_maxsize - 1) /
                             usbp->epc[ep]->out_maxsize);

  EPCTL_SET_STAT_RX(ep);
}

/**
 * @brief   Starts a transmit operation on an IN endpoint.
 *
 * @param[in] usbp      pointer to the @p USBDriver object
 * @param[in] ep        endpoint number
 *
 * @notapi
 */
void usb_lld_start_in(USBDriver *usbp, usbep_t ep) {
  size_t n;
  USBInEndpointState *isp = usbp->epc[ep]->in_state;

  /* Transfer initialization.*/
  n = isp->txsize;
  if (n > (size_t)usbp->epc[ep]->in_maxsize)
    n = (size_t)usbp->epc[ep]->in_maxsize;

  isp->txlast = n;
  usb_packet_write_from_buffer(ep, isp->txbuf, n);

  EPCTL_SET_STAT_TX(ep, n);
}

/**
 * @brief   Brings an OUT endpoint in the stalled state.
 *
 * @param[in] usbp      pointer to the @p USBDriver object
 * @param[in] ep        endpoint number
 *
 * @notapi
 */
void usb_lld_stall_out(USBDriver *usbp, usbep_t ep) {

  (void)usbp;
  uint32_t presetup = (SN32_USB->INSTS & mskEP0_PRESETUP);
  if((ep ==0) && !presetup) {
    SN32_USB->EPCTL[ep] |= mskEPn_ENDP_STATE_STALL;
  }
  else if (ep !=0){
      SN32_USB->EPCTL[ep] |= mskEPn_ENDP_STATE_STALL;
  }
}

/**
 * @brief   Brings an IN endpoint in the stalled state.
 *
 * @param[in] usbp      pointer to the @p USBDriver object
 * @param[in] ep        endpoint number
 *
 * @notapi
 */
void usb_lld_stall_in(USBDriver *usbp, usbep_t ep) {

  (void)usbp;
  uint32_t presetup = (SN32_USB->INSTS & mskEP0_PRESETUP);
  if((ep ==0) && !presetup) {
    SN32_USB->EPCTL[ep] |= mskEPn_ENDP_STATE_STALL;
  }
  else if (ep !=0){
      SN32_USB->EPCTL[ep] |= mskEPn_ENDP_STATE_STALL;
  }

}

/**
 * @brief   Brings an OUT endpoint in the active state.
 *
 * @param[in] usbp      pointer to the @p USBDriver object
 * @param[in] ep        endpoint number
 *
 * @notapi
 */
void usb_lld_clear_out(USBDriver *usbp, usbep_t ep) {

  (void)usbp;

  /* Makes sure to not put to NAK an endpoint that is already
     transferring.*/
  if (SN32_USB->EPCTL[ep] & mskEPn_ENDP_STATE_ACK)
    SN32_USB->EPCTL[ep] = (mskEPn_ENDP_EN|mskEPn_ENDP_STATE_ACK);
}

/**
 * @brief   Brings an IN endpoint in the active state.
 *
 * @param[in] usbp      pointer to the @p USBDriver object
 * @param[in] ep        endpoint number
 *
 * @notapi
 */
void usb_lld_clear_in(USBDriver *usbp, usbep_t ep) {

  (void)usbp;

  /* Makes sure to not put to NAK an endpoint that is already
     transferring.*/
  if (SN32_USB->EPCTL[ep] & mskEPn_ENDP_STATE_ACK)
    EPCTL_SET_STAT_RX(ep);
}

#endif /* HAL_USE_USB */

/** @} */
