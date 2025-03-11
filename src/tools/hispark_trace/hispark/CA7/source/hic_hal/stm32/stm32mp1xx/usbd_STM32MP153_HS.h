/**
  * @copyright Copyright (c) HiSilicon (Shanghai) Technologies Co., Ltd. 2012-2023. All rights reserved.
  * Redistribution and use in source and binary forms, with or without modification, are permitted provided that the
  * following conditions are met:
  * 1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following
  * disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the
  * following disclaimer in the documentation and/or other materials provided with the distribution.
  * 3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote
  * products derived from this software without specific prior written permission.
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
  * INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
  * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
  * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE
  * USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  * @file    usbd_STM32MP153_HS.h
  * @author  MCU Driver Team
  * @brief   usbd hs for STM32MP15x
  */
#ifndef USBD_STM32MP153_HS_H
#define USBD_STM32MP153_HS_H

#include <stdint.h>
#include "stm32mp1xx.h"

#define RX_FIFO_SIZE    1024
#define TX0_FIFO_SIZE   1024
#define TX1_FIFO_SIZE   512
#define TX2_FIFO_SIZE   512
#define TX3_FIFO_SIZE   512
#define TX4_FIFO_SIZE   512
#define TX5_FIFO_SIZE   128
#define TX6_FIFO_SIZE   128
#define TX7_FIFO_SIZE   128
#define TX8_FIFO_SIZE   128

#define WORD_CNT(x) (((x) + 3) / 4)
#define OTG_INT_LEVEL  (1 << 3) /* priority = 1, low 3 bit is must be kept at reset value */
#define MHZ  1000000
#define USBD_DELAY_10MS 10

/* See register OTG_GRXSTSR field PKTSTS */
enum {
    OTG_GRXSTSR_PKTSTS_GLOBAL_OUT_NAK = 1,
    OTG_GRXSTSR_PKTSTS_OUT_DATA_PKT = 2,
    OTG_GRXSTSR_PKTSTS_OUT_TRANSFER_COMPLETED = 3,
    OTG_GRXSTSR_PKTSTS_SETUP_TRANSACTION_COMPLETED = 4,
    OTG_GRXSTSR_PKTSTS_SETUP_DATA_PKT = 6,
};

#define OTG             USB1_OTG_HS
#define USB1_OTG_HS_PERIPH_BASE (MCU_AHB2_PERIPH_BASE + 0x1000000) // (0x5800D000UL )
#define USBX_BASE       USB1_OTG_HS_PERIPH_BASE
#define USBX_DEVICE     ((USB_OTG_DeviceTypeDef *)(USBX_BASE + USB_OTG_DEVICE_BASE))

#define USBX_INEP(i) ((USB_OTG_INEndpointTypeDef *)(USBX_BASE + USB_OTG_IN_ENDPOINT_BASE + ((i)*USB_OTG_EP_REG_SIZE)))
#define USBX_OUTEP(i) \
    ((USB_OTG_OUTEndpointTypeDef *)(USBX_BASE + USB_OTG_OUT_ENDPOINT_BASE + ((i)*USB_OTG_EP_REG_SIZE)))
#define USBX_DFIFO(i) *(__IO uint32_t *)(USBX_BASE + USB_OTG_FIFO_BASE + ((i)*USB_OTG_FIFO_SIZE))

#define TX_FIFO(n)      *((__packed volatile uint32_t *)(USB1_OTG_HS + USB_OTG_FIFO_BASE + (n)*USB_OTG_FIFO_SIZE))
#define RX_FIFO         *((__packed volatile uint32_t *)(USB1_OTG_HS + USB_OTG_FIFO_BASE))

#define EP_IN_TYPE(num)     ((USBX_INEP(num)->DIEPCTL >> USB_OTG_DIEPCTL_EPTYP_Pos) & 3)
#define EP_OUT_TYPE(num)    ((USBX_OUTEP(num)->DOEPCTL >> USB_OTG_DIEPCTL_EPTYP_Pos) & 3)

#define HIGH_HALF_WORD(x)   ((x) << 16)
#define TXFIFO_WAIT_TIMEOUT_MS    (5)

#define PLLNDIV_SHIFT      0
#define PLLNDIV            0x7F
#define PLLFRACIN_SHIFT    10
#define PLLFRACIN          (0xFFFF << 10)
#define PLLFRACCTL         (1 << 29)
#define USBPHYC_PLLEN      (1 << 26)
#define USBPHYC_PLLDITHEN1 (1 << 31)
#define USBPHYC_PLLDITHEN0 (1 << 30)
#define USBPHYC_PLLSTRBYP  (1 << 28)

#define USBD_CLK_RATE        24000000U
#define CLEAR_FIFO_TRY_TIMES 200000U

typedef struct {
    uint8_t ndiv;
    uint16_t frac;
} PllParams;

#endif