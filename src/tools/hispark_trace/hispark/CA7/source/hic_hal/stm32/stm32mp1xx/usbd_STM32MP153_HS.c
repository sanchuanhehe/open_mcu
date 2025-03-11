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
  * @file    usbd_STM32MP153_HS.c
  * @author  MCU Driver Team
  * @brief   usbd hs for STM32MP15x
  */

#include <rl_usb.h>
#include "interrupt.h"
#include "usb_def.h"
#include "usb_config.c"
#include "usbd_STM32MP153_HS.h"

uint32_t g_outMaxPacketSize[5] = {USBD_MAX_PACKET0, 0, 0, 0, 0};
uint8_t g_outPacketCnt[5] = {1, 0, 0, 0, 0};
uint8_t g_inPacketCnt[5] = {1, 0, 0, 0, 0};

#if (USBD_HID_ENABLE == 1)
uint32_t g_hidIntInPacketData[WORD_CNT(USBD_HID_MAX_PACKET)];
#endif

#if (USBD_CDC_ACM_ENABLE == 1)
uint32_t g_cdcAcmIntInPacketData[WORD_CNT(USBD_CDC_ACM_MAX_PACKET)];
#endif

uint32_t *g_inPacketDataPtr[5] = {
/* endpoint 0 */
    0,
/* endpoint 1 */
#if ((USBD_HID_ENABLE == 1) && (USBD_HID_EP_INTIN == 1))
    g_hidIntInPacketData,
#elif ((USBD_CDC_ACM_ENABLE == 1) && (USBD_CDC_ACM_EP_INTIN == 1))
    g_cdcAcmIntInPacketData,
#else
    0,
#endif
/* endpoint 2 */
#if ((USBD_HID_ENABLE == 1) && (USBD_HID_EP_INTIN == 2))
    g_hidIntInPacketData,
#elif ((USBD_CDC_ACM_ENABLE == 1) && (USBD_CDC_ACM_EP_INTIN == 2))
    g_cdcAcmIntInPacketData,
#else
    0,
#endif
/* endpoint 3 */
#if ((USBD_HID_ENABLE == 1) && (USBD_HID_EP_INTIN == 3))
    g_hidIntInPacketData,
#elif ((USBD_CDC_ACM_ENABLE == 1) && (USBD_CDC_ACM_EP_INTIN == 3))
    g_cdcAcmIntInPacketData,
#else
    0,
#endif
/* endpoint 4 */
#if ((USBD_HID_ENABLE == 1) && (USBD_HID_EP_INTIN == 4))
    g_hidIntInPacketData,
#elif ((USBD_CDC_ACM_ENABLE == 1) && (USBD_CDC_ACM_EP_INTIN == 4))
    g_cdcAcmIntInPacketData,
#else
    0,
#endif
};

uint32_t g_inPacketDataCnt[5] = {0};
uint32_t g_inPacketDataReady = 0;
uint32_t g_syncWriteEp = 0;

/* Get USB Device Out packet counter
 *
 *   Return Value:    out packet counter
 */
static inline uint32_t USBD_OutPktCnt(uint32_t epNum)
{
    return (USBX_OUTEP(epNum)->DOEPTSIZ & USB_OTG_DOEPTSIZ_PKTCNT) >> USB_OTG_DOEPTSIZ_PKTCNT_Pos;
}

/* Get USB Device setup packet counter
 *
 *   Return Value:    setup packet counter
 */
static inline uint32_t USBD_SetupPktCnt(uint32_t epNum)
{
    return (USBX_OUTEP(epNum)->DOEPTSIZ & USB_OTG_DOEPTSIZ_STUPCNT) >> USB_OTG_DOEPTSIZ_STUPCNT_Pos;
}

/*
 *  USB Device Interrupt enable
 *   Called by USBD_Init to enable the USB Interrupt
 *    Return Value:    None
 */
void USBD_IntrEna(void)
{
    IRQ_Enable(OTG_IRQn, OTG_INT_LEVEL, OTG_HS_IRQHandler);
}

/*
 *  Get USB device phyc pll params
 *   Called by USBD_PhycPllInit
 *   Return Value:    None
 */
static void USBD_PhycGetPllParams(uint32_t clkRate, PllParams *pllParams)
{
#define PLL_FVCO 2880
#define FACTOR  (1 << 16)
    unsigned long long fvco;
    unsigned long long ndiv;
    unsigned long long frac;

    /*
     * | FVCO = INFF*2*(NDIV + FRACT/2^16 ) when DITHER_DISABLE[1] = 1
     * | FVCO = 2880MHz
     * | NDIV = integer part of input bits to set the LDF
     * | FRACT = fractional part of input bits to set the LDF
     * =>    PLLNDIV = integer part of (FVCO / (INFF*2))
     * =>    PLLFRACIN = fractional part of(FVCO / INFF*2) * 2^16
     * <=>  PLLFRACIN = ((FVCO / (INFF*2)) - PLLNDIV) * 2^16
     */
    fvco = (unsigned long long)PLL_FVCO * MHZ; /* In Hz */

    ndiv = fvco;
    ndiv /= (clkRate * 2); // multi * 2
    pllParams->ndiv = (uint8_t)ndiv;

    frac = fvco * FACTOR;
    frac /= (clkRate * 2); // multi * 2
    frac -= (ndiv * FACTOR);
    pllParams->frac = (uint16_t)frac;
}

/*
 *  Get USB device phyc pll Init
 *   Called by USBD_Init
 *   Return Value:    None
 */
static void USBD_PhycPllInit(void)
{
    const uint32_t clkRate = USBD_CLK_RATE;
    uint32_t  usbPhycPll;
    PllParams pllPara;

    USBD_PhycGetPllParams(clkRate, &pllPara);

    /* Disable pll */
    USBPHYC->PLL &= ~USBPHYC_PLLEN;

    /* Calc Pll */
    usbPhycPll = USBPHYC_PLLDITHEN1 | USBPHYC_PLLDITHEN0 | USBPHYC_PLLSTRBYP;
    usbPhycPll |= ((pllPara.ndiv << PLLNDIV_SHIFT) & PLLNDIV);
    if (pllPara.frac) {
        usbPhycPll |= PLLFRACCTL;
        usbPhycPll |= ((pllPara.frac << PLLFRACIN_SHIFT) & PLLFRACIN);
    }
    /* Set Pll */
    USBPHYC->PLL = usbPhycPll;
    HAL_Delay(1);

    /* Enable pll */
    USBPHYC->PLL |= USBPHYC_PLLEN;
    if (!(USBPHYC->PLL & USBPHYC_PLLEN)) {
        return;
    }
    HAL_Delay(1);
    return;
}

/*
 *  USB Device Rcc Config
 *   Called by the USBD_Init
 *   Return Value:    None
 */
static void USBD_RccConfig(void)
{
    RCC->USBCKSELR = RCC_USBCKSELR_USBOSRC; // USBPHYSRC=0, hse_ker_ck clock

    RCC->MP_APB4ENSETR |= RCC_MP_APB4ENSETR_USBPHYEN;
    HAL_Delay(10); /* Wait ~10 ms */
    RCC->APB4RSTSETR |= RCC_APB4RSTSETR_USBPHYRST;
    HAL_Delay(10);         /* Wait ~10 ms */
    RCC->APB4RSTCLRR |= RCC_APB4RSTCLRR_USBPHYRST; /* Reset OTG HS clock */
    HAL_Delay(10);         /* Wait ~10 ms */
    RCC->MP_APB4ENSETR |= RCC_MP_APB4ENSETR_USBPHYEN;
    HAL_Delay(10); /* Wait ~10 ms */
    RCC->MC_APB4ENSETR |= RCC_MC_APB4ENSETR_USBPHYEN;
    HAL_Delay(10);             /* Wait ~10 ms */
    RCC->MP_APB4LPENSETR |= RCC_MP_APB4LPENSETR_USBPHYLPEN;
    HAL_Delay(10);             /* Wait ~10 ms */

    RCC->MP_AHB2ENSETR |= RCC_MP_AHB2ENSETR_USBOEN;
    HAL_Delay(10);          /* Wait ~10 ms */
    RCC->AHB2RSTSETR |= RCC_AHB2RSTSETR_USBORST;
    HAL_Delay(10);                                                 /* Wait ~10 ms                        */
    RCC->AHB2RSTCLRR |= RCC_AHB2RSTCLRR_USBORST; /* Reset OTG HS clock                 */ /* H743, USB1OTGRST */
    HAL_Delay(10);                                                 /* Wait ~10 ms                        */

    RCC->MP_AHB2ENSETR |= RCC_MP_AHB2ENSETR_USBOEN;
    HAL_Delay(10);            /* Wait ~10 ms */
    RCC->MC_AHB2ENSETR |= RCC_MC_AHB2ENSETR_USBOEN;   // USBOLPEN
    HAL_Delay(10);            /* Wait ~10 ms */
    RCC->MP_AHB2LPENSETR |= RCC_MP_AHB2LPENSETR_USBOLPEN; // USBOLPEN
    HAL_Delay(10);            /* Wait ~10 ms */
}

/*
 *  Wait USB Device AHB master is idle
 *   Called by USBD_Init
 *   Return Value:    None
 */
static inline void USBD_WatiAhbIdle(void)
{
    int32_t tout = 1000; // Wait max 1000 ms for AHBIDL = 0
    while (!(OTG->GRSTCTL & USB_OTG_GRSTCTL_AHBIDL) && (tout-- > 0)) {
        HAL_Delay(1); // Wait 1 ms
    }
}

/*
 *  USB Device OTG reset
 *   Called by USBD_Init
 *   Return Value:    None
 */
static inline void USBD_OtgReset(void)
{
    int32_t tout = 1000; // Wait max 1000 ms for CRST = 0
    OTG->GRSTCTL |= USB_OTG_GRSTCTL_CSRST;
    while ((OTG->GRSTCTL & USB_OTG_GRSTCTL_CSRST) && (tout-- > 0)) {
        HAL_Delay(1); // Wait 1 ms
    }
    HAL_Delay(3); // Wait 3 ms
}

/*
 *  USB Device Wait Global out nak config effective
 *   Called by USBD_Init
 *   Return Value:    None
 */
static inline void USBD_WaitGlobalOutNakEffective(void)
{
    int32_t wcnt = 1000; /* Up to 1000 attempts */
    while (!(OTG->GINTSTS & USB_OTG_GINTSTS_BOUTNAKEFF)) {
        // wait until global NAK
        if ((wcnt--) == 0) {
            break;
        }
    }
}

/*
 *  USB Device Wait Global out nak config effective
 *  Called by USBD_DisableEP
 *  Parameters:      epNum: Device Endpoint Number
 *                       epNum.0..3: Address
 *                       epNum.7:    Dir
 *  Return Value:    None
 */
static inline void USBD_WaitEndpointInterruptDisabled(U32 epNum)
{
    int32_t wcnt = 1000;  /* Up to 1000 attempts */
    while (!(USBX_OUTEP(epNum)->DOEPINT & USB_OTG_DOEPINT_EPDISD)) {
        // wait until EP disabled
        if ((wcnt--) == 0) {
            break;
        }
    }
}

/*
 *  ReInit Tx/Rx/non-periodicc Tx Fifo Size
 *   Called by USBD_Init
 *   Return Value:    None
 */
static void USBD_FifoSizeReInit(void)
{
    uint32_t fifoOffset;
    uint16_t fifoSize[] = {
        TX1_FIFO_SIZE, TX2_FIFO_SIZE, TX3_FIFO_SIZE, TX4_FIFO_SIZE
    };

    /* Set Rx Fifo Size */
    OTG->GRXFSIZ = RX_FIFO_SIZE / sizeof(int);
    /* Set host non-periodic transmit FIFO */
    OTG->DIEPTXF0_HNPTXFSIZ = (RX_FIFO_SIZE / sizeof(int)) | HIGH_HALF_WORD(TX0_FIFO_SIZE / sizeof(int));

    OTG->DIEPTXF[0] =
        ((RX_FIFO_SIZE + TX0_FIFO_SIZE) / sizeof(int)) | (HIGH_HALF_WORD(TX1_FIFO_SIZE /sizeof(int)));

    /* DIEPTXF1 */
    OTG->DIEPTXF[1] = ((RX_FIFO_SIZE + TX0_FIFO_SIZE + TX1_FIFO_SIZE) / sizeof(int)) |
                      (HIGH_HALF_WORD(TX2_FIFO_SIZE / sizeof(int)));

    /* DIEPTXF2 */
    OTG->DIEPTXF[2] =
        ((RX_FIFO_SIZE + TX0_FIFO_SIZE + TX1_FIFO_SIZE + TX2_FIFO_SIZE) / sizeof(int)) |
        (HIGH_HALF_WORD(TX3_FIFO_SIZE / sizeof(int)));

    /* DIEPTXF3 */
    OTG->DIEPTXF[3] = ((RX_FIFO_SIZE + TX0_FIFO_SIZE + TX1_FIFO_SIZE +
                        TX2_FIFO_SIZE + TX3_FIFO_SIZE) /sizeof(int)) |
                      (HIGH_HALF_WORD(TX4_FIFO_SIZE / sizeof(int)));
}

/*
 *  Synchronous Write USB Device Endpoint Data to intermediate Buffer and synchronously
 *  transferred to FIFO on next NAK event.
 *    Parameters:      epNum: Device Endpoint Number
 *                        epNum.0..3: Address
 *                        epNum.7:    Dir
 *                     pData: Pointer to Data Buffer
 *                     cnt:   Number of bytes to write
 *    Return Value:    Number of bytes written
 */
static uint32_t USBD_SynWriteToBuffer(U32 epNum, U8 *pData, U32 cnt)
{
    /*
       get space in Ep TxFIFO
       Reset the IN endpoint if we can't get enough space. Usually this
       means something goes wrong on this endpoint and usb host stops
       sending IN token.
    */
    uint32_t startTs = HAL_GetTick();
    uint32_t curTs;
    uint32_t delta;

    /* Wait until the FIFO has sufficient space to receive data. */
    while ((cnt > 0) && ((USBX_INEP(epNum)->DTXFSTS * sizeof(int)) < cnt)) {
        curTs = HAL_GetTick();
        delta = (curTs >= startTs) ? (curTs - startTs) : 0xFFFFFFFF - startTs + curTs;
        if (delta >= TXFIFO_WAIT_TIMEOUT_MS) {
            USBD_ResetEP(epNum | USB_ENDPOINT_DIRECTION_MASK);
            return 0;
        }
    }

    /* set transfer size and packet count */
    USBX_INEP(epNum)->DIEPTSIZ = cnt |
        (g_inPacketCnt[epNum] << USB_OTG_DIEPTSIZ_PKTCNT_Pos) |
        (g_inPacketCnt[epNum] << USB_OTG_DIEPTSIZ_MULCNT_Pos);

    /* enable ep and clear NAK */
    USBX_INEP(epNum)->DIEPCTL |= USB_OTG_DIEPCTL_EPENA | USB_OTG_DIEPCTL_CNAK;
    if (cnt) {
        uint32_t val = WORD_CNT(cnt);
        while (val--) {
            /* copy data to endpoint TxFIFO */
            USBX_DFIFO((uint32_t)epNum) = __UNALIGNED_UINT32_READ(pData);
            pData += sizeof(int);
        }
    }
    g_inPacketDataReady &= ~(1 << epNum);
    return cnt;
}


/*
 *  Asynchronous Write USB Device Endpoint Data to intermediate Buffer and synchronously
 *  transferred to FIFO on next NAK event.
 *    Parameters:      epNum: Device Endpoint Number
 *                        epNum.0..3: Address
 *                        epNum.7:    Dir
 *                     pData: Pointer to Data Buffer
 *                     cnt:   Number of bytes to write
 *    Return Value:    Number of bytes written
 */
static uint32_t USBD_AsynWriteToBuffer(U32 epNum, U8 *pData, U32 cnt)
{
    U32 *ptr = g_inPacketDataPtr[epNum];
    uint32_t val = WORD_CNT(cnt);

    if ((g_inPacketDataReady & (1 << epNum)) != 0) { /* already, done */
        return 0;
    }
    g_inPacketDataCnt[epNum] = cnt; /* save Data size */
    while (val) {
        // save data to intermediate buffer
        *ptr++ = *((U32 *)pData);
        pData += sizeof(U32);
        --val;
    }
    g_inPacketDataReady |= 1 << epNum;
    // Set NAK to enable interrupt on NAK
    USBX_INEP(epNum)->DIEPCTL |= USB_OTG_DIEPCTL_SNAK;
    // INEPNEM = 1, IN EP NAK efective msk
    USBX_DEVICE->DIEPMSK |= USB_OTG_DIEPMSK_INEPNEM;
    return cnt;
}

/*
 *  USB Device Initialize Function
 *   Called by the User to initialize USB
 *   Return Value:    None
 */
void USBD_Init(void)
{
    USBD_RccConfig();
    USBD_IntrEna();
    USBD_PhycPllInit();
    USBPHYC->MISC = 0; // 0x0:No clock gating. PHY dedicated 60 MHz Port1/Port2 clocks are always delivered to
                       // target controller. PHY 48 MHz output clock is always delivered to target controller

#ifdef __OTG_HS_EMBEDDED_PHY
    OTG->GUSBCFG |= USB_OTG_GUSBCFG_PHYSEL;
#endif
    HAL_Delay(20); // Wait ~20 ms
    USBD_WatiAhbIdle();
    USBD_OtgReset();

    OTG->GAHBCFG &= ~USB_OTG_GAHBCFG_GINT; // Disable interrupts
    // No VBUS sensing
    // ToDo(elee): do we want this disabled?
    OTG->GCCFG &= ~USB_OTG_GCCFG_VBDEN;
    USBX_DEVICE->DCTL |= USB_OTG_DCTL_SDIS; // soft disconnect enabled

    // Force device mode
    OTG->GUSBCFG = USB_OTG_GUSBCFG_FDMOD |
                   USB_OTG_GUSBCFG_TRDT_3 |
                   USB_OTG_GUSBCFG_TRDT_0 |
                   USB_OTG_GUSBCFG_TOCAL;
    HAL_Delay(100); // Wait min 25 ms, we wait ~100 ms

#ifndef __OTG_HS_EMBEDDED_PHY
    USBX_DEVICE->DCFG &= ~USB_OTG_DCFG_DSPD; // High speed phy
#else
    USBX_DEVICE->DCFG &= ~USB_OTG_DCFG_DSPD; // Full speed phy
#endif
    HAL_Delay(2);  // Wait 2 ms

    OTG->GINTMSK =
        USB_OTG_GINTMSK_USBSUSPM |  // suspend int unmask
        USB_OTG_GINTMSK_USBRST |    // reset int unmask
        USB_OTG_GINTMSK_ENUMDNEM |  // enumeration done int unmask
        USB_OTG_GINTMSK_RXFLVLM |   // receive fifo non-empty int  unmask
        USB_OTG_GINTMSK_IEPINT |    // IN EP int unmask
        USB_OTG_GINTMSK_OEPINT |    // OUT EP int unmask
        USB_OTG_GINTMSK_WUIM |      // resume int unmask
        ((USBD_P_SOF_Event != 0) ? USB_OTG_GINTMSK_SOFM : 0); // SOF int unmask
    OTG->GAHBCFG |= USB_OTG_GAHBCFG_GINT |
                    USB_OTG_GAHBCFG_HBSTLEN_0 |
                    USB_OTG_GAHBCFG_HBSTLEN_1 |
                    USB_OTG_GAHBCFG_TXFELVL;
}

/*
 *  USB Device Connect Function
 *   Called by the User to Connect/Disconnect USB Device
 *    Parameters:      con:   Connect/Disconnect
 *    Return Value:    None
 */

void USBD_Connect(BOOL con)
{
    if (con) {
#ifdef __OTG_HS_EMBEDDED_PHY
        OTG->GCCFG &= ~USB_OTG_GCCFG_PWRDWN; // power down activated
#endif
        USBX_DEVICE->DCTL &= ~USB_OTG_DCTL_SDIS; // soft disconnect disabled
    } else {
        USBX_DEVICE->DCTL |= USB_OTG_DCTL_SDIS; // soft disconnect enabled
#ifdef __OTG_HS_EMBEDDED_PHY
        OTG->GCCFG |= USB_OTG_GCCFG_PWRDWN; // power down deactivated
#endif
    }
}

/*
 *  USB Device Reset Function
 *   Called automatically on USB Device Reset
 *    Return Value:    None
 */

void USBD_Reset(void)
{
    g_syncWriteEp = 0;
    g_inPacketDataReady = 0;
    USBX_DEVICE->DOEPMSK = 0;
    USBX_DEVICE->DIEPMSK = 0;

    for (uint32_t i = 0; i < (USBD_EP_NUM + 1); i++) {
        if (USBX_OUTEP(i)->DOEPCTL & USB_OTG_DOEPCTL_EPENA) {
            // OUT EP disable, Set NAK
            USBX_OUTEP(i)->DOEPCTL = USB_OTG_DOEPCTL_EPDIS | USB_OTG_DOEPCTL_SNAK;
        }
        if (USBX_INEP(i)->DIEPCTL & USB_OTG_DIEPCTL_EPENA) {
            // IN EP disable, Set NAK
            USBX_INEP(i)->DIEPCTL =
                USB_OTG_DIEPCTL_EPDIS | USB_OTG_DIEPCTL_SNAK;
        }

        USBX_INEP(i)->DIEPINT = 0x1B;
        USBX_OUTEP(i)->DOEPINT = 0x1B; // clear OUT Ep interrupts
    }

    USBD_SetAddress(0, 1);

    USBX_DEVICE->DAINTMSK = (1 << USB_OTG_DAINTMSK_OEPM_Pos) | // unmask IN&OUT EP0 interruts
                            (1 << USB_OTG_DAINTMSK_IEPM_Pos);
    USBX_DEVICE->DOEPMSK = USB_OTG_DOEPMSK_STUPM |  // setup phase done
                           USB_OTG_DOEPMSK_EPDM |   // endpoint disabled
                           USB_OTG_DOEPMSK_XFRCM;   // transfer complete
    USBX_DEVICE->DIEPMSK = USB_OTG_DIEPMSK_EPDM |   // endpoint disabled
                           USB_OTG_DIEPMSK_XFRCM;   // transfer completed

    USBD_FifoSizeReInit();  /* ReInit Fifo Size registers */

    USBX_OUTEP(0)->DOEPTSIZ = USB_OTG_DOEPTSIZ_STUPCNT_0 | // setup count = 1
                              (1 << USB_OTG_DOEPTSIZ_PKTCNT_Pos) |  // packet count
                              USBD_MAX_PACKET0;
}

/*
 *  USB Device Suspend Function
 *   Called automatically on USB Device Suspend
 *    Return Value:    None
 */

void USBD_Suspend(void) {}

/*
 *  USB Device Resume Function
 *   Called automatically on USB Device Resume
 *    Return Value:    None
 */

void USBD_Resume(void) {}

/*
 *  USB Device Remote Wakeup Function
 *   Called automatically on USB Device Remote Wakeup
 *    Return Value:    None
 */

void USBD_WakeUp(void)
{
    USBX_DEVICE->DCTL |= USB_OTG_DCTL_RWUSIG; // remote wakeup signaling
    HAL_Delay(5);                     // Wait ~5 ms
    USBX_DEVICE->DCTL &= ~USB_OTG_DCTL_RWUSIG;
}

/*
 *  USB Device Remote Wakeup Configuration Function
 *    Parameters:      cfg:   Device Enable/Disable
 *    Return Value:    None
 */

void USBD_WakeUpCfg(BOOL cfg)
{
}

/*
 *  USB Device Set Address Function
 *    Parameters:      adr:   USB Device Address
 *    Return Value:    None
 */

void USBD_SetAddress(U32 adr, U32 setup)
{
    if (setup) {
        USBX_DEVICE->DCFG = (USBX_DEVICE->DCFG & ~USB_OTG_DCFG_DAD) |
                            (adr << USB_OTG_DCFG_DAD_Pos);
    }
}

/*
 *  USB Device Flush IN Endpoint Transmit Fifo
 *    Parameters:      adr:   USB Device Address
 *    Return Value:    None
 */
static void USBD_FlushInEpFifo(uint32_t epNum)
{
    uint32_t wcnt;

    epNum &= ~USB_ENDPOINT_DIRECTION_MASK;
    OTG->GRSTCTL = (OTG->GRSTCTL & ~USB_OTG_GRSTCTL_TXFNUM) |
                   (epNum << USB_OTG_GRSTCTL_TXFNUM_Pos) |
                   USB_OTG_GRSTCTL_TXFFLSH;
    /* wait until fifo is flushed 10 cnt */
    wcnt = 10;
    while (OTG->GRSTCTL & USB_OTG_GRSTCTL_TXFFLSH) {
        if ((wcnt--) == 0) {
            break;
        }
    }
}

/*
 *  USB Device Configure Function
 *    Parameters:      cfg:   Device Configure/Deconfigure
 *    Return Value:    None
 */

void USBD_Configure(BOOL cfg)
{
    g_inPacketDataReady &= ~1UL;
}

/*
 *  Configure USB Device Endpoint according to Descriptor
 *    Parameters:      pEPD:  Pointer to Device Endpoint Descriptor
 *    Return Value:    None
 */

void USBD_ConfigEP(USB_ENDPOINT_DESCRIPTOR *pEPD)
{
    uint32_t num;
    uint32_t val;
    uint32_t type;

    num = pEPD->bEndpointAddress & ~(USB_ENDPOINT_DIRECTION_MASK);
    val = pEPD->wMaxPacketSize;
    type = pEPD->bmAttributes & USB_ENDPOINT_TYPE_MASK;

    if (pEPD->bEndpointAddress & USB_ENDPOINT_DIRECTION_MASK) {
        g_inPacketCnt[num] = 1;

        USBX_DEVICE->DAINTMSK |= (1 << num);     // unmask IN EP int
        USBX_INEP(num)->DIEPCTL = (num << USB_OTG_DIEPCTL_TXFNUM_Pos) |  // fifo number
                                  (type << USB_OTG_DIEPCTL_EPTYP_Pos) | // ep type
                                  (val & USB_OTG_DIEPCTL_MPSIZ);   // max packet size
        if ((type == USB_ENDPOINT_TYPE_BULK) || (type == USB_ENDPOINT_TYPE_INTERRUPT)) {
            // if interrupt or bulk EP
            USBX_INEP(num)->DIEPCTL |= USB_OTG_DIEPCTL_SD0PID_SEVNFRM;
        }
    } else {
        g_outMaxPacketSize[num] = val & USB_OTG_DOEPCTL_MPSIZ;
        g_outPacketCnt[num] = 1;

        USBX_DEVICE->DAINTMSK |= (1 << (num + USB_OTG_DAINTMSK_OEPM_Pos)); // unmask OUT EP int

        USBX_OUTEP(num)->DOEPCTL = (type << USB_OTG_DOEPCTL_EPTYP_Pos) | // EP type
                                   (val & USB_OTG_DOEPCTL_MPSIZ); // max packet size

        USBX_OUTEP(num)->DOEPTSIZ = (g_outPacketCnt[num] << USB_OTG_DOEPTSIZ_PKTCNT_Pos) | // packet count
                                    (val & USB_OTG_DOEPCTL_MPSIZ); // transfer size
        if ((type == USB_ENDPOINT_TYPE_BULK) || (type == USB_ENDPOINT_TYPE_INTERRUPT)) {
            // if int or bulk EP
            USBX_OUTEP(num)->DOEPCTL |= USB_OTG_DOEPCTL_SD0PID_SEVNFRM;
        }
    }
}

/*
 *  Set Direction for USB Device Control Endpoint
 *    Parameters:      dir:   Out (dir == 0), In (dir <> 0)
 *    Return Value:    None
 */

void USBD_DirCtrlEP(U32 dir)
{
}

/*
 *  Enable USB Device Endpoint
 *    Parameters:      epNum: Device Endpoint Number
 *                       epNum.0..3: Address
 *                       epNum.7:    Dir
 *    Return Value:    None
 */

void USBD_EnableEP(U32 epNum)
{
    if (epNum & USB_ENDPOINT_DIRECTION_MASK) {
        epNum &= ~USB_ENDPOINT_DIRECTION_MASK;
        USBX_INEP(epNum)->DIEPCTL |= USB_OTG_DIEPCTL_USBAEP | // EP active
                                     USB_OTG_DIEPCTL_SNAK;    // set EP NAK
        if (USBX_INEP(epNum)->DIEPCTL & USB_OTG_DIEPCTL_EPENA) {
            // disable EP
            USBX_INEP(epNum)->DIEPCTL |= USB_OTG_DIEPCTL_EPDIS;
        }

        g_inPacketDataReady &= ~(1 << epNum);
    } else {
        USBX_OUTEP(epNum)->DOEPCTL |= USB_OTG_DOEPCTL_USBAEP |  // EP active
                                      USB_OTG_DOEPCTL_EPENA |   // enable EP
                                      USB_OTG_DOEPCTL_CNAK;     // clear EP NAK
    }
}

/*
 *  Disable USB Endpoint
 *    Parameters:      epNum: Endpoint Number
 *                       epNum.0..3: Address
 *                       epNum.7:    Dir
 *    Return Value:    None
 */

void USBD_DisableEP(U32 epNum)
{
    // Disable IN Endpoint
    if (epNum & USB_ENDPOINT_DIRECTION_MASK) {
        epNum &= ~USB_ENDPOINT_DIRECTION_MASK;
        g_inPacketDataReady &= ~(1 << epNum);

        if (USBX_INEP(epNum)->DIEPCTL & USB_OTG_DIEPCTL_EPENA) {
            // disable EP
            USBX_INEP(epNum)->DIEPCTL |= USB_OTG_DIEPCTL_EPDIS;
        }
        // set EP NAK
        USBX_INEP(epNum)->DIEPCTL |= USB_OTG_DIEPCTL_SNAK;
        // deactivate EP
        USBX_INEP(epNum)->DIEPCTL &= ~USB_OTG_DIEPCTL_USBAEP;
    } else {
        // Disable OUT Endpoint
        // set global out nak
        USBX_DEVICE->DCTL |= USB_OTG_DCTL_SGONAK;

        USBD_WaitGlobalOutNakEffective();

        if (USBX_OUTEP(epNum)->DOEPCTL & USB_OTG_DOEPCTL_EPENA) {
            // disable EP
            USBX_OUTEP(epNum)->DOEPCTL |= USB_OTG_DOEPCTL_EPDIS;
        }
        // set EP NAK
        USBX_OUTEP(epNum)->DOEPCTL |= USB_OTG_DOEPCTL_SNAK;
        //  deactivate EP
        USBX_OUTEP(epNum)->DOEPCTL &= ~USB_OTG_DOEPCTL_USBAEP;

        USBD_WaitEndpointInterruptDisabled(epNum);
        // clear global nak
        USBX_DEVICE->DCTL |= USB_OTG_DCTL_CGONAK;
    }
}

/*
 *  Reset USB Device Endpoint
 *    Parameters:      epNum: Device Endpoint Number
 *                       epNum.0..3: Address
 *                       epNum.7:    Dir
 *    Return Value:    None
 */

void USBD_ResetEP(U32 epNum)
{
    // Reset IN Endpoint
    if (epNum & USB_ENDPOINT_DIRECTION_MASK) {
        epNum &= ~USB_ENDPOINT_DIRECTION_MASK;
        g_inPacketDataReady &= ~(1 << epNum);
        if (USBX_INEP(epNum)->DIEPCTL & USB_OTG_DIEPCTL_EPENA) {
            // disable EP
            USBX_INEP(epNum)->DIEPCTL |= USB_OTG_DIEPCTL_EPDIS;
        }
        // set EP NAK
        USBX_INEP(epNum)->DIEPCTL |= USB_OTG_DIEPCTL_SNAK;

        // Flush endpoint fifo
        USBD_FlushInEpFifo(epNum | USB_ENDPOINT_DIRECTION_MASK);
    }
}

/*
 *  Set Stall for USB Device Endpoint
 *    Parameters:      epNum: Device Endpoint Number
 *                       epNum.0..3: Address
 *                       epNum.7:    Dir
 *    Return Value:    None
 */

void USBD_SetStallEP(U32 epNum)
{
    if (!(epNum & USB_ENDPOINT_DIRECTION_MASK)) {
        // Stall OUT Endpoint

        // set global out nak
        USBX_DEVICE->DCTL |= USB_OTG_DCTL_SGONAK;

        USBD_WaitGlobalOutNakEffective(); // set global out nak

        if (USBX_OUTEP(epNum)->DOEPCTL & USB_OTG_DOEPCTL_EPENA) {
            // disable EP
            USBX_OUTEP(epNum)->DOEPCTL |= USB_OTG_DOEPCTL_EPDIS;
        }
        // set stall
        USBX_OUTEP(epNum)->DOEPCTL |= USB_OTG_DOEPCTL_STALL;
        USBD_WaitEndpointInterruptDisabled(epNum);

        // clear global nak
        USBX_DEVICE->DCTL |= USB_OTG_DCTL_CGONAK;
    } else {
        // Stall IN endpoint
        epNum &= ~USB_ENDPOINT_DIRECTION_MASK;
        if (USBX_INEP(epNum)->DIEPCTL & USB_OTG_DIEPCTL_EPENA) {
            // disable endpoint
            USBX_INEP(epNum)->DIEPCTL |= USB_OTG_DIEPCTL_EPDIS;
        }
        // set stall
        USBX_INEP(epNum)->DIEPCTL |= USB_OTG_DIEPCTL_STALL;

        USBD_FlushInEpFifo(epNum | USB_ENDPOINT_DIRECTION_MASK);
    }
}

/*
 *  Clear Stall for USB Device Endpoint
 *    Parameters:      epNum: Device Endpoint Number
 *                       epNum.0..3: Address
 *                       epNum.7:    Dir
 *    Return Value:    None
 */

void USBD_ClrStallEP(U32 epNum)
{
    if (!(epNum & USB_ENDPOINT_DIRECTION_MASK)) {
        // Clear OUT endpoint Stall
        if (EP_OUT_TYPE(epNum) > USB_ENDPOINT_TYPE_ISOCHRONOUS) {
            // if EP type Bulk or Interrupt
            // Set DATA0 PID
            USBX_OUTEP(epNum)->DOEPCTL |= USB_OTG_DOEPCTL_SD0PID_SEVNFRM;
            // Clear stall
            USBX_OUTEP(epNum)->DOEPCTL &= ~USB_OTG_DOEPCTL_STALL;
        }
    } else {
        // Clear IN Endpoint Stall
        epNum &= ~USB_ENDPOINT_DIRECTION_MASK;

        if (USBX_INEP(epNum)->DIEPCTL & USB_OTG_DIEPCTL_EPENA) {
            // disable endpoint
            USBX_INEP(epNum)->DIEPCTL |= USB_OTG_DIEPCTL_EPDIS;
        }

        // Flush endpoint fifo
        USBD_FlushInEpFifo(epNum | USB_ENDPOINT_DIRECTION_MASK);

        if (EP_IN_TYPE(epNum) > USB_ENDPOINT_TYPE_ISOCHRONOUS) {
            // if Interrupt or bulk EP, Set DATA0 PID
            USBX_INEP(epNum)->DIEPCTL |= USB_OTG_DIEPCTL_SD0PID_SEVNFRM;
        }

        // clear Stall
        USBX_INEP(epNum)->DIEPCTL &= ~USB_OTG_DIEPCTL_STALL;
    }
}

/*
 *  Clear USB Device Endpoint Buffer
 *    Parameters:      epNum: Device Endpoint Number
 *                       epNum.0..3: Address
 *                       epNum.7:    Dir
 *    Return Value:    None
 */
void USBD_ClearEPBuf(U32 epNum)
{
    if (epNum & USB_ENDPOINT_DIRECTION_MASK) {
        USBD_FlushInEpFifo(epNum | USB_OTG_GRSTCTL_RXFFLSH);
    } else {
        OTG->GRSTCTL |= USB_OTG_GRSTCTL_RXFFLSH;
        /* delay 4x4 nop */
        for (int i = 0; i < 4; i++) {
            __NOP();
        }
    }
}

/*
 *  Read USB Device Endpoint Data
 *    Parameters:      epNum: Device Endpoint Number
 *                       epNum.0..3: Address
 *                       epNum.7:    Dir
 *                     pData: Pointer to Data Buffer
 *    Return Value:    Number of bytes read
 */

uint32_t USBD_ReadEP(U32 epNum, U8 *pData, uint32_t bufsz)
{
    U32 val;
    U32 sz;

    if ((USBX_OUTEP(epNum)->DOEPCTL & USB_OTG_DOEPCTL_USBAEP) == 0) {
        // if Ep not active
        return (0);
    }

    sz = (OTG->GRXSTSP & USB_OTG_BCNT_Msk) >> USB_OTG_BCNT_Pos; // get available data size

    /* elee: copy from the f103 code, tbd, still required?
      Commit 5ffac262f,  and 3d1a68b768c57cb403bab9d5598f3e9f1d3506a2 (for the
      core) If smaller data is read here, is the rest of the data pulled in
      later? */
    if (sz > bufsz) {
        sz = bufsz;
    }

    // copy data from fifo if Isochronous Ep: data is copied to intermediate buffer
    for (val = 0; val < (uint32_t)WORD_CNT(sz); val++) {
        __UNALIGNED_UINT32_WRITE(pData, USBX_DFIFO(0U));
        pData += sizeof(int);
    }

    // wait RxFIFO non-empty (OUT transfer completed or Setup trans. completed)
    while ((OTG->GINTSTS & USB_OTG_GINTMSK_RXFLVLM) == 0) {
    }
    OTG->GRXSTSP;
    OTG->GINTMSK |= USB_OTG_GINTMSK_RXFLVLM;   // unmask RxFIFO non-empty interrupt

    return (sz);
}

/*
 *  Write USB Device Endpoint Data
 *  If write was requested synchronously from IRQ then data is written to FIFO
 *  directly else data is written to the intermediate buffer and synchronously
 *  transferred to FIFO on next NAK event.
 *    Parameters:      epNum: Device Endpoint Number
 *                        epNum.0..3: Address
 *                        epNum.7:    Dir
 *                     pData: Pointer to Data Buffer
 *                     cnt:   Number of bytes to write
 *    Return Value:    Number of bytes written
 */

uint32_t USBD_WriteEP(U32 epNum, U8 *pData, U32 cnt)
{
    epNum &= ~USB_ENDPOINT_DIRECTION_MASK;

    if ((USBX_INEP(epNum)->DIEPCTL & USB_OTG_DIEPCTL_USBAEP) == 0) {
        // if Ep not active
        return (0);
    }

    /* Asynchronous write to intermediate buffer */
    if (!g_syncWriteEp && g_inPacketDataPtr[epNum]) {
        return USBD_AsynWriteToBuffer(epNum, pData, cnt);
    } else {
        return USBD_SynWriteToBuffer(epNum, pData, cnt);
    }
}

/*
 *  Get USB Device Last Frame Number
 *    Parameters:      None
 *    Return Value:    Frame Number
 */

uint32_t USBD_GetFrame(void)
{
    return (USBX_DEVICE->DSTS  & USB_OTG_DSTS_FNSOF) >> USB_OTG_DSTS_FNSOF_Pos;
}

/*
 *  USB Device Interrupt Service Routine
 */
void OTG_HS_IRQHandler(void)
{
    GIC_DisableIRQ(OTG_IRQn);
    USBD_SignalHandler();
}

void USBD_ResetProcess(void)
{
    USBD_Reset();
    usbd_reset_core();
#ifdef __RTX
    if (USBD_RTX_DevTask) {
        isr_evt_set(USBD_EVT_RESET, USBD_RTX_DevTask);
    }
#else
    if (USBD_P_Reset_Event) {
        USBD_P_Reset_Event();
    }
#endif
    OTG->GINTSTS = USB_OTG_GINTSTS_USBRST;
}

static unsigned int FindNextEpIndex(unsigned int *start, unsigned int *mask)
{
    unsigned int i;
    unsigned int num = 0;

    for (i = *start; i < (USBD_EP_NUM + 1); i++) {
        if ((*mask & (1 << i)) != 0) {
            num = i;
            *mask &= ~(1 << i);
            break;
        }
    }
    *start = i;
    return num;
}

static void USBD_SuspendProcess(void)
{
    USBD_Suspend();
#ifdef __RTX
    if (USBD_RTX_DevTask) {
        isr_evt_set(USBD_EVT_SUSPEND, USBD_RTX_DevTask);
    }
#else
    if (USBD_P_Suspend_Event) {
        USBD_P_Suspend_Event();
    }
#endif
    OTG->GINTSTS = USB_OTG_GINTSTS_USBSUSP;
}

static void USBD_ResumeProcess(void)
{
    USBD_Resume();
#ifdef __RTX
    if (USBD_RTX_DevTask) {
        isr_evt_set(USBD_EVT_RESUME, USBD_RTX_DevTask);
    }
#else
    if (USBD_P_Resume_Event) {
        USBD_P_Resume_Event();
    }
#endif
    OTG->GINTSTS = USB_OTG_GINTSTS_WKUINT;
}

static void USBD_SpeedEnumDoneProcess(void)
{
    if (!((USBX_DEVICE->DSTS & USB_OTG_DSTS_ENUMSPD) >> USB_OTG_DSTS_ENUMSPD_Pos)) {
        USBD_HighSpeed = 1;
    }
    USBX_INEP(0)->DIEPCTL &= ~USB_OTG_DIEPCTL_MPSIZ;
    USBX_INEP(0)->DIEPCTL |= g_outMaxPacketSize[0]; /* EP0 max packet             */
    USBX_DEVICE->DCTL |= USB_OTG_DCTL_CGINAK;     // clear global IN NAK
    USBX_DEVICE->DCTL |= USB_OTG_DCTL_CGONAK;     // clear global OUT NAK
    OTG->GINTSTS |= USB_OTG_GINTSTS_ENUMDNE;
}

static void USBD_StartFrameProcess(void)
{
#ifdef __RTX
    if (USBD_RTX_DevTask) {
        isr_evt_set(USBD_EVT_SOF, USBD_RTX_DevTask);
    }
#else
    if (USBD_P_SOF_Event) {
        USBD_P_SOF_Event();
    }
#endif
    OTG->GINTSTS = USB_OTG_GINTSTS_SOF;
}

static void USBD_RxDataProcess(void)
{
    uint32_t val = OTG->GRXSTSR;
    uint32_t num = val & USB_OTG_GRXSTSP_EPNUM;

    switch ((val & USB_OTG_GRXSTSP_PKTSTS) >> USB_OTG_GRXSTSP_PKTSTS_Pos) {
        // setup packet
        case OTG_GRXSTSR_PKTSTS_SETUP_DATA_PKT:
            OTG->GINTMSK &= ~USB_OTG_GINTMSK_RXFLVLM;
            if (USBD_P_EP[num]) {
                USBD_P_EP[num](USBD_EVT_SETUP);
            }
            break;

        // OUT packet
        case OTG_GRXSTSR_PKTSTS_OUT_DATA_PKT:
            OTG->GINTMSK &= ~USB_OTG_GINTMSK_RXFLVLM;
            if (USBD_P_EP[num]) {
                USBD_P_EP[num](USBD_EVT_OUT);
            }
            break;
        default:
            OTG->GRXSTSP;
            break;
    }
}

static void USBD_OutEndPointInterruptProcess(void)
{
    unsigned int mask;
    unsigned int i = 0;

    /* process all endpoint with interrupt */
    mask = ((USBX_DEVICE->DAINT & USBX_DEVICE->DAINTMSK) & USB_OTG_DAINTMSK_OEPM) >> USB_OTG_DAINTMSK_OEPM_Pos;
    while (mask != 0) {
        unsigned int num = FindNextEpIndex(&i, &mask); /* Find EpNum */
        // Endpoint disabled
        if (USBX_OUTEP(num)->DOEPINT & USB_OTG_DOEPINT_EPDISD) {
            USBX_OUTEP(num)->DOEPINT |= USB_OTG_DOEPINT_EPDISD;
        }

        // Transfer complete interrupt
        if ((USBX_OUTEP(num)->DOEPINT & USB_OTG_DOEPINT_XFRC) | (USBX_OUTEP(num)->DOEPINT & USB_OTG_DOEPINT_STUP)) {
            USBX_OUTEP(num)->DOEPTSIZ =
                (g_outPacketCnt[num] << USB_OTG_DOEPTSIZ_PKTCNT_Pos) | /* packet count */
                (g_outMaxPacketSize[num]);    /* transfer size */
            if (num == 0) {
                USBX_OUTEP(0)->DOEPTSIZ |= USB_OTG_DOEPTSIZ_STUPCNT_0;
            }
            USBX_OUTEP(num)->DOEPCTL |= USB_OTG_DOEPCTL_EPENA | USB_OTG_DOEPCTL_CNAK;
            USBX_OUTEP(num)->DOEPINT |= USB_OTG_DOEPINT_XFRC;
        }
    }
}

static void USBD_InEndPointNakProcess(unsigned int epNum)
{
    g_syncWriteEp = 1;
    USBD_WriteEP(epNum, (uint8_t *)g_inPacketDataPtr[epNum],
        g_inPacketDataCnt[epNum]);
    g_syncWriteEp = 0;
    if (!g_inPacketDataReady) {
        // No more pending IN transfers and disable IN NAK interrupts
        USBX_DEVICE->DIEPMSK &= ~USB_OTG_DIEPMSK_INEPNEM;
    }
}

static void USBD_InEndPointInterruptProcess(void)
{
    unsigned int mask;
    unsigned int i = 0;

    /* process all endpoint with interrupt */
    mask = (USBX_DEVICE->DAINT & USBX_DEVICE->DAINTMSK & USB_OTG_DAINTMSK_IEPM);
    while (mask != 0) {
        unsigned int num = FindNextEpIndex(&i, &mask);
        // Endpoint disabled
        if (USBX_INEP(num)->DIEPINT & USB_OTG_DIEPINT_EPDISD) {
            USBX_INEP(num)->DIEPINT = USB_OTG_DIEPINT_EPDISD;
        }

        // IN endpoint NAK effective
        if (USBX_INEP(num)->DIEPINT & USB_OTG_DIEPINT_INEPNE) {
            if ((g_inPacketDataPtr[num] != 0) &&
                ((g_inPacketDataReady & (1 << num)) != 0)) {
                USBD_InEndPointNakProcess(num);
                continue;
            } else {
                USBX_INEP(num)->DIEPCTL |= USB_OTG_DIEPCTL_CNAK;
            }
            USBX_INEP(num)->DIEPINT = USB_OTG_DIEPINT_INEPNE;
        }

        // Transmit completed
        if (USBX_INEP(num)->DIEPINT & USB_OTG_DIEPINT_XFRC) {
            USBX_INEP(num)->DIEPINT = USB_OTG_DIEPINT_XFRC;
            g_syncWriteEp = 1;
            if (USBD_P_EP[num]) {
                USBD_P_EP[num](USBD_EVT_IN);
            }
            g_syncWriteEp = 0;
        }
    }
}

/*
 *  USB Handler Process
 */
void USBD_Handler(void)
{
    uint32_t istr = OTG->GINTSTS & OTG->GINTMSK;

    // reset interrupt
    if (istr & USB_OTG_GINTSTS_USBRST) {
        USBD_ResetProcess();
    }

    // suspend interrupt
    if (istr & USB_OTG_GINTSTS_USBSUSP) {
        USBD_SuspendProcess();
    }

    // resume interrupt
    if (istr & USB_OTG_GINTSTS_WKUINT) {
        USBD_ResumeProcess();
    }

    // speed enumeration completed
    if (istr & USB_OTG_GINTSTS_ENUMDNE) {
        USBD_SpeedEnumDoneProcess();
    }

    // Start Of Frame
    if (istr & USB_OTG_GINTSTS_SOF) {
        USBD_StartFrameProcess();
    }

    // RxFIFO non-empty
    if (istr & USB_OTG_GINTSTS_RXFLVL) {
        USBD_RxDataProcess();
    }

    // OUT Packet
    if (istr & USB_OTG_GINTSTS_OEPINT) {
        USBD_OutEndPointInterruptProcess();
    }

    // IN Packet
    if (istr & USB_OTG_GINTSTS_IEPINT) {
        USBD_InEndPointInterruptProcess();
    }

    IRQ_Enable(OTG_IRQn, OTG_INT_LEVEL, OTG_HS_IRQHandler);
}
