/* ---------------------------------------------------------------------------- */
/*                  Atmel Microcontroller Software Support                      */
/*                       SAM Software Package License                           */
/* ---------------------------------------------------------------------------- */
/* Copyright (c) 2015, Atmel Corporation                                        */
/*                                                                              */
/* All rights reserved.                                                         */
/*                                                                              */
/* Redistribution and use in source and binary forms, with or without           */
/* modification, are permitted provided that the following condition is met:    */
/*                                                                              */
/* - Redistributions of source code must retain the above copyright notice,     */
/* this list of conditions and the disclaimer below.                            */
/*                                                                              */
/* Atmel's name may not be used to endorse or promote products derived from     */
/* this software without specific prior written permission.                     */
/*                                                                              */
/* DISCLAIMER:  THIS SOFTWARE IS PROVIDED BY ATMEL "AS IS" AND ANY EXPRESS OR   */
/* IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF */
/* MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT ARE   */
/* DISCLAIMED. IN NO EVENT SHALL ATMEL BE LIABLE FOR ANY DIRECT, INDIRECT,      */
/* INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT */
/* LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,  */
/* OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF    */
/* LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING         */
/* NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, */
/* EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.                           */
/* ---------------------------------------------------------------------------- */

/**
 * \page usb_video USB Video Class Example
 *
 * \section Purpose
 *
 * The USB Video Example will help you to get familiar with the
 * USB Device Port(UDP) and ISI interface on SAMV7/E7 Microcontrollers.
 *
 * \section Requirements
 *
 * This package can be used with SAMV71 Xplained Ultra board or SAME70 Xplained board
 * - On-board ISI interface.
 * - External sensor, in the example, Omnivision OV2643/OV5640/OV7740/OV9740
 * sensor could be used.
 * It does data conversion, if necessary, before the storage in memory
 * through DMA. The ISI supports color CMOS image sensor and grayscale image
 * sensors with a reduced set of functionalities.
 * When an EK running this program connected to a host (PC for example), with
 * USB cable, the EK appears as a video camera for the host.
 *
 * \note
 * For the limitation of external memory size, this example only support for
 * QVGA format.
 * \section Description
 *
 * The USB video can help you to be familiar with the ISI (Image Sensor
 * Interface) to connects a CMOS-type image sensor to the processor and
 * provides image capture in various formats.
 *
 * \section Usage
 *
 * -# Build the program and download it inside the board.
 *    Please refer to the Getting Started with SAM V71/E70 Microcontrollers.pdf
 * -# On the computer, open and configure a terminal application
 *    (e.g. HyperTerminal on Microsoft Windows) with these settings:
 *   - 115200 baud rate
 *   - 8 bits of data
 *   - No parity
 *   - 1 stop bit
 *   - No flow control
 * -# Start the application.
 * -# In the terminal window, the following text should appear:
 *     \code
 *     -- USB Video Example xxx --
 *     -- SAMxxxxx-xx
 *     -- Compiled: xxx xx xxxx xx:xx:xx --
 *     \endcode
 * -# When connecting USB cable to windows, the
 *    new "xxx USB Device" appears in the
 *    hardware %device list.
 * -# Once the device is connected and configured on windows XP,
 *    "USB Video Device" will appear in "My Computer", you can double click
 *    it to preview with default resolution - QVGA.
 * -# Other video camera programs can also be used to monitor the capture
 *    output. The demo is tested on windows XP through "AmCap.exe".
 *
 * \section References
 * - usb_video_isi/main.c
 * - pio: PIO interface driver
 *    - pio.h
 *    - pio_it.h
 * - usb: USB Framework and UDP interface driver
 *    - \ref usbd_framework
 *       - \ref usbd_api
 *    - \ref usb_core
 *    \if usb_video_sim
 *    - \ref usb_video_sim
 *    \endif
 */

/**
 * \file
 *
 * This file contains all the specific code for the
 * usb_video_isi
 *
 * \section Contents
 *
 * The code can be roughly broken down as follows:
 *    - Configuration functions
 *       - Configure TWI
 *       - Configure pins for OV sensor
 *       - Configure ISI controller
 *    - Interrupt handlers
 *       - TWI_Handler
 *    - The main function, which implements the program behaviour
 */

/*----------------------------------------------------------------------------
 *        Headers
 *----------------------------------------------------------------------------*/

#include "board.h"

#include <USBDescriptors.h>
#include <USBRequests.h>
#include "USBD.h"
#include <USBD_HAL.h>
#include <USBDDriver.h>
#include <VIDEODescriptors.h>
#include <USBVideo.h>

#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <assert.h>

/*----------------------------------------------------------------------------
 *        Local definitions
 *----------------------------------------------------------------------------*/

/** TWI clock frequency in Hz. */
#define TWCK            400000

/** TWI Pins definition */
#define BOARD_PINS_TWI_ISI PINS_TWI0

/** TWI peripheral ID for Sensor configuration */
#define BOARD_ID_TWI_ISI        ID_TWIHS0
/** TWI base address for Sensor configuration */
#define BOARD_BASE_TWI_ISI      TWIHS0

/** ISI DMA buffer base address */
#define ISI_BASE    SDRAM_CS_ADDR

/** Frame Buffer Descriptors , it depends on size of external memory, more
        is better */
#define ISI_MAX_PREV_BUFFER    12

/*----------------------------------------------------------------------------
 *          External variables
 *----------------------------------------------------------------------------*/

extern const USBDDriverDescriptors usbdDriverDescriptors;

/*----------------------------------------------------------------------------
 *        Local variables
 *----------------------------------------------------------------------------*/

/** ISI pins to configure. */
const Pin pinsTWI[] = BOARD_PINS_TWI_ISI;
const Pin pin_ISI_RST = BOARD_ISI_RST;
const Pin pin_ISI_PWD = BOARD_ISI_PWD;
const Pin pPinsISI[] = {BOARD_ISI_PINS};

/** TWI driver instance.*/
static Twid twid;

COMPILER_WORD_ALIGNED
ISI_FrameBufferDescriptors  preBufDescList[ISI_MAX_PREV_BUFFER ];

COMPILER_WORD_ALIGNED
static uint8_t pXfrBuffers[FRAME_PACKET_SIZE_HS * (ISO_HIGH_BW_MODE + 1)][1];

/** Xfr Maximum packet size */
static uint32_t frmMaxPktSize = FRAME_PACKET_SIZE_HS * (ISO_HIGH_BW_MODE + 1);

/** Alternate interfaces */
static uint8_t bAlternateInterfaces[4];

/** Probe & Commit Controls */

static USBVideoProbeData viddProbeData = {
	0, /* bmHint: All parameters fixed: sent by host */
	0x01,   /* bFormatIndex: Format #1 */
	0x01,   /* bFrameIndex: Frame #1 */
	FRAME_INTERVALC(4), /* dwFrameInterval: in 100ns */
	0, /* wKeyFrameRate: not used */
	0, /* wPFrameRate: not used */
	10000, /* wCompQuality: highest */
	0, /* wCompWindowSize: ?K */
	100, /* wDelay: Internal VS latency in ms */
	FRAME_BUFFER_SIZEC(800, 600), /* dwMaxVideoFrameSize: in bytes */
	FRAME_PACKET_SIZE_FS /* dwMaxPayloadTransferSize: in bytes */
};

/** Buffer for USB requests data */

static uint8_t pControlBuffer[32];

/** Byte index in frame */
static uint32_t frmI = 0;
/** Frame count */
static uint32_t frmC;

/** Frame size: Width, Height */
static uint32_t frmW = 320, frmH = 240;

/** USB transferring frame data */
static uint8_t bFrameXfring = 0;

/** USB Streaming interface ON */
static volatile uint8_t bVideoON = 0;
static volatile uint8_t bVidON = 0;
static volatile uint8_t IsiPrevBuffIndex;
static volatile uint8_t UsbPrevBuffIndex;
static volatile uint32_t delay;
static volatile uint32_t displayFrameAddr;

/* Image size in preview mode */
static uint32_t wImageWidth, wImageHeight;

/* Image output format */
static sensorOutputFormat_t wImageFormat;

extern const sensorProfile_t ov2643Profile;
extern const sensorProfile_t ov5640Profile;
extern const sensorProfile_t ov7740Profile;
extern const sensorProfile_t ov9740Profile;

static const sensorProfile_t *sensorProfiles[] = {
	&ov2643Profile, &ov5640Profile, &ov7740Profile, &ov9740Profile
};

static const char sensorName[][8] = {
	"OV2643", "OV5640", "OV7740", "OV9740"
};

/** Video buffers */

static uint8_t *pVideoBufffers = (uint8_t *)ISI_BASE;

/*----------------------------------------------------------------------------
 *        Local functions
 *----------------------------------------------------------------------------*/
/**
 * \brief TWI interrupt handler. Forwards the interrupt to the TWI driver handler.
 */
void TWIHS0_Handler(void)
{
	TWID_Handler(&twid);
}

/**
 * \brief ISI interrupt handler.
 */
void ISI_Handler(void)
{
	/* If Preview DMA Transfer has Terminated */
	if (ISI_StatusRegister() & ISI_SR_PXFR_DONE) {
		/* Increase index, skip display locked one */
		displayFrameAddr = preBufDescList[IsiPrevBuffIndex].Current;
		IsiPrevBuffIndex++;

		if (IsiPrevBuffIndex == ISI_MAX_PREV_BUFFER)
			IsiPrevBuffIndex = 0;
	}
}

/**
 * \brief Set up Frame Buffer Descriptors(FBD) for preview path.
 */
static void _isi_AllocateFBD(void)
{
	uint32_t i;
	uint32_t framebufOffset;
	framebufOffset =  FRAME_BUFFER_SIZEC(frmW, frmH);

	for (i = 0; i < ISI_MAX_PREV_BUFFER; i++) {
		preBufDescList[i].Current = (uint32_t)ISI_BASE + (i * framebufOffset);
		preBufDescList[i].Control = ISI_DMA_P_CTRL_P_FETCH;
		preBufDescList[i].Next    = (uint32_t)&preBufDescList[i + 1];
	}

	preBufDescList[i - 1].Next = (uint32_t)&preBufDescList[0];
	SCB_CleanDCache_by_Addr((uint32_t *)&preBufDescList, sizeof(preBufDescList));
}

/**
 * \brief TWI initialization.
 */
static void _twiInit(void)
{
	/* Configure TWI pins. */
	PIO_Configure(pinsTWI, PIO_LISTSIZE(pinsTWI));
	/* Enable TWI peripheral clock */
	PMC_EnablePeripheral(BOARD_ID_TWI_ISI);
	/* Configure TWI */
	TWI_ConfigureMaster(BOARD_BASE_TWI_ISI, TWCK, BOARD_MCK);
	TWID_Initialize(&twid, BOARD_BASE_TWI_ISI);

	/* Configure TWI interrupts */
	NVIC_ClearPendingIRQ(TWIHS0_IRQn);
	NVIC_EnableIRQ(TWIHS0_IRQn);
}

/**
 * \brief ISI PCK initialization.
 */
static void _isiPckInit(void)
{
	/* Configure ISI pins. */
	PIO_Configure(pPinsISI, PIO_LISTSIZE(pPinsISI));

	/* Disable programmable clock 1 output */
	REG_PMC_SCDR = PMC_SCER_PCK0;
	/* Enable the DAC master clock */
	PMC->PMC_PCK[0] = PMC_PCK_CSS_MCK | (4 << 4);
	/* Enable programmable clock 0 output */
	REG_PMC_SCER = PMC_SCER_PCK0;

	/* Wait for the PCKRDY0 bit to be set in the PMC_SR register*/
	while ((REG_PMC_SR & PMC_SR_PCKRDY0) == 0);

	/* ISI PWD OFF*/
	PIO_Clear(&pin_ISI_PWD);
	PIO_Clear(&pin_ISI_RST);
}

/**
 * \brief ISI  initialization.
 */
static void _isiInit(void)
{
	/* Enable ISI peripheral clock */
	PMC_EnablePeripheral(ID_ISI);
	/* Set up Frame Buffer Descriptors(FBD) for preview path. */
	_isi_AllocateFBD();
	/* Reset ISI peripheral */
	ISI_Reset();
	/* Set the windows blank */
	ISI_SetBlank(0, 0);
	/* Set vertical and horizontal Size of the Image Sensor for preview path*/
	ISI_SetSensorSize(wImageWidth, wImageHeight);
	/*  Set data stream in YUV format.*/
	ISI_setInputStream(RGB_INPUT);

	/* Set preview size to fit LCD */
	ISI_setPreviewSize(frmW, frmH);
	/* calculate scaler factor automatically. */
	ISI_calcScalerFactor();
	/* Configure DMA for preview path. */
	ISI_setDmaInPreviewPath((uint32_t)&preBufDescList[0],
							ISI_DMA_P_CTRL_P_FETCH, (uint32_t)ISI_BASE);
	ISI_RgbPixelMapping(ISI_CFG2_RGB_CFG_MODE3);
}

/* Interrupt handler for TC0 interrupt. */
void TC0_Handler(void)
{
	REG_TC0_SR0;
}

/**
 *  Configure Timer Counter 0 to generate an interrupt every 250ms.
 */
static void _ConfigureTc(void)
{
	uint32_t div;
	uint32_t tcclks;

	/** Enable peripheral clock. */
	PMC_EnablePeripheral(ID_TC0);
	/** Configure TC for a 4Hz frequency and trigger on RC compare. */
	TC_FindMckDivisor(4, BOARD_MCK, &div, &tcclks, BOARD_MCK);

	TC_Configure(TC0, 0, tcclks | TC_CMR_CPCTRG);
	TC0->TC_CHANNEL[ 0 ].TC_RC = (BOARD_MCK / div) / 4;

	/* Configure and enable interrupt on RC compare */
	NVIC_ClearPendingIRQ(TC0_IRQn);
	NVIC_EnableIRQ(TC0_IRQn);

	TC0->TC_CHANNEL[ 0 ].TC_IER = TC_IER_CPCS;
	TC_Start(TC0, 0);
}

/*------------ USB Video Device Functions ------------*/

/**
 * Max packet size calculation for High bandwidth transfer\n
 * - Mode 1: last packet is <epSize+1> ~ <epSize*2> bytes\n
 * - Mode 2: last packet is <epSize*2+1> ~ <epSize*3> bytes
 */
static void VIDD_UpdateHighBWMaxPacketSize(void)
{

#if (ISO_HIGH_BW_MODE == 1 || ISO_HIGH_BW_MODE == 2)
	uint32_t frmSiz = frmW * frmH * 2 + FRAME_PAYLOAD_HDR_SIZE;
	uint32_t pktSiz = FRAME_PACKET_SIZE_HS * (ISO_HIGH_BW_MODE + 1);
	uint32_t nbLast = frmSiz % pktSiz;

	while (1) {
		nbLast = frmSiz % pktSiz;

		if (nbLast == 0 || nbLast > (FRAME_PACKET_SIZE_HS * ISO_HIGH_BW_MODE))
			break;

		pktSiz --;
	}

	frmMaxPktSize = pktSiz;
#else
	frmMaxPktSize = FRAME_PACKET_SIZE_HS; // EP size
#endif
}

/**
 * Send USB control status.
 */
static void VIDD_StatusStage(void)
{
	USBVideoProbeData *pProbe = (USBVideoProbeData *)pControlBuffer;
	viddProbeData.bFormatIndex = pProbe->bFormatIndex;
	viddProbeData.bFrameIndex  = pProbe->bFrameIndex;
	viddProbeData.dwFrameInterval = pProbe->dwFrameInterval;

	//viddProbeData.dwMaxVideoFrameSize = pProbe->dwMaxVideoFrameSize;
	switch (pProbe->bFrameIndex) {
	case 1: frmW = VIDCAMD_FW_1; frmH = VIDCAMD_FH_1; break;

	case 2: frmW = VIDCAMD_FW_2; frmH = VIDCAMD_FH_2; break;

	case 3: frmW = VIDCAMD_FW_3; frmH = VIDCAMD_FH_3; break;
	}

	VIDD_UpdateHighBWMaxPacketSize();
	USBD_Write(0, 0, 0, 0, 0);
}

/**
 * Handle SetCUR request for USB Video Device.
 */
static void VIDD_SetCUR(const USBGenericRequest *pReq)
{
	uint8_t bCS = USBVideoRequest_GetControlSelector(pReq);
	uint32_t len;
	TRACE_INFO_WP("SetCUR(%d) ", pReq->wLength);

	if (pReq->wIndex == VIDCAMD_StreamInterfaceNum) {
		TRACE_INFO_WP("VS ");

		switch (bCS) {
		case VS_PROBE_CONTROL:
			TRACE_INFO_WP("PROBE ");
			len = sizeof(USBVideoProbeData);

			if (pReq->wLength < len) len = pReq->wLength;

			USBD_Read(0, pControlBuffer, len, (TransferCallback)VIDD_StatusStage, 0);
			break;

		case VS_COMMIT_CONTROL:
			TRACE_INFO_WP("COMMIT ");
			len = sizeof(USBVideoProbeData);

			if (pReq->wLength < len) len = pReq->wLength;

			USBD_Read(0, pControlBuffer, len, (TransferCallback)VIDD_StatusStage, 0);

		default: USBD_Stall(0);
		}
	} else if (pReq->wIndex == VIDCAMD_ControlInterfaceNum){
		TRACE_INFO_WP("VC ");
    } else {
      USBD_Stall(0);
    }
}

/**
 * Handle GetCUR request for USB Video Device.
 */
static void VIDD_GetCUR(const USBGenericRequest *pReq)
{
	uint8_t bCS = USBVideoRequest_GetControlSelector(pReq);
	uint32_t len;
	TRACE_INFO_WP("GetCUR(%d) ", pReq->wLength);

	if (pReq->wIndex == VIDCAMD_StreamInterfaceNum) {
		TRACE_INFO_WP("VS ");

		switch (bCS) {
		case VS_PROBE_CONTROL:
			TRACE_INFO_WP("PROBE ");
			len = sizeof(USBVideoProbeData);

			if (pReq->wLength < len) len = pReq->wLength;

			USBD_Write(0, &viddProbeData, len, 0, 0);
			break;

		case VS_COMMIT_CONTROL: /* Returns current state of VS I/F */
			TRACE_INFO_WP("COMMIT ");
			USBD_Write(0, &bAlternateInterfaces[VIDCAMD_StreamInterfaceNum], 1, 0, 0);
			break;

		default: USBD_Stall(0);
		}
	} else if (pReq->wIndex == VIDCAMD_ControlInterfaceNum) {
      TRACE_INFO_WP("VC ");
    } else {
      USBD_Stall(0);
    }
}

/**
 * Handle GetDEF request for USB Video Device.
 */
static void VIDD_GetDEF(const USBGenericRequest *pReq)
{
	printf("GetDEF(%x,%x,%d)\n\r", pReq->wIndex, pReq->wValue, pReq->wLength);
}

/**
 * Handle GetINFO request for USB Video Device.
 */
static void VIDD_GetINFO(const USBGenericRequest *pReq)
{
	printf("GetINFO(%x,%x,%d)\n\r", pReq->wIndex, pReq->wValue, pReq->wLength);
}

/**
 * Handle GetMIN request for USB Video Device.
 */
static void VIDD_GetMIN(const USBGenericRequest *pReq)
{
	printf("GetMin(%x,%x,%d)\n\r", pReq->wIndex, pReq->wValue, pReq->wLength);
	VIDD_GetCUR(pReq);

}

/**
 * Handle GetMAX request for USB Video Device.
 */
static void VIDD_GetMAX(const USBGenericRequest *pReq)
{
	printf("GetMax(%x,%x,%d)\n\r", pReq->wIndex, pReq->wValue, pReq->wLength);
	VIDD_GetCUR(pReq);
}

/**
 * Handle GetRES request for USB Video Device.
 */
static void VIDD_GetRES(const USBGenericRequest *pReq)
{
	printf("GetRES(%x,%x,%d) ", pReq->wIndex, pReq->wValue, pReq->wLength);
}

/**
 * Callback that invoked when USB packet is sent.
 */
static void VIDD_PayloadSent(void *pArg, uint8_t bStat)
{
	uint32_t pktSize;
	pArg = pArg; bStat = bStat;
	uint8_t *pX = pXfrBuffers[0];

	uint32_t frmSize = FRAME_BUFFER_SIZEC(frmW, frmH);
	uint8_t *pV = pVideoBufffers;
	USBVideoPayloadHeader *pHdr = (USBVideoPayloadHeader *)pX;
	uint32_t maxPktSize = USBD_IsHighSpeed() ? (frmMaxPktSize)
						  : (FRAME_PACKET_SIZE_FS);
	pktSize = frmSize - frmI;
	pHdr->bHeaderLength     = FRAME_PAYLOAD_HDR_SIZE;
	pHdr->bmHeaderInfo.B    = 0;

	if (pktSize > maxPktSize - pHdr->bHeaderLength)
		pktSize = maxPktSize - pHdr->bHeaderLength;

	pV = &pV[frmI];
	frmI += pktSize;
	pHdr->bmHeaderInfo.bm.FID = (frmC & 1);

	if (frmI >= frmSize) {
		frmC ++;
		frmI = 0;
		pHdr->bmHeaderInfo.bm.EoF = 1;

		if (bFrameXfring)
			bFrameXfring = 0;
	} else {
		pHdr->bmHeaderInfo.bm.EoF = 0;

		if (bFrameXfring == 0) {
			bFrameXfring = 1;

			while (UsbPrevBuffIndex == IsiPrevBuffIndex)
				memory_barrier();

			UsbPrevBuffIndex = IsiPrevBuffIndex;
			pVideoBufffers = (uint8_t *)displayFrameAddr;
			memory_barrier();
		}
	}

	pHdr->bmHeaderInfo.bm.EOH =  1;

	USBD_HAL_WrWithHdr(VIDCAMD_IsoInEndpointNum, pHdr,
					   pHdr->bHeaderLength, pV, pktSize);
}

/*
 *   USB Device Driver Callbacks re-implementation
 */

/**
 * Configure USB settings for USB device
 */
static void _ConfigureUotghs(void)
{
	/* UTMI parallel mode, High/Full/Low Speed */
	/* UUSBCK not used in this configuration (High Speed) */
	PMC->PMC_SCDR = PMC_SCDR_USBCLK;
	/* USB clock register: USB Clock Input is UTMI PLL */
	PMC->PMC_USB = PMC_USB_USBS;
	/* Enable peripheral clock for USBHS */
	PMC_EnablePeripheral(ID_USBHS);
	USBHS->USBHS_CTRL = USBHS_CTRL_UIMOD_DEVICE;
	/* Enable PLL 480 MHz */
	PMC->CKGR_UCKR = CKGR_UCKR_UPLLEN | CKGR_UCKR_UPLLCOUNT(0xF);

	/* Wait that PLL is considered locked by the PMC */
	while (!(PMC->PMC_SR & PMC_SR_LOCKU));

	/* IRQ */
	NVIC_EnableIRQ(USBHS_IRQn);
}

/**
 *  Invoked whenever a SETUP request is received from the host. Forwards the
 *  request to the standard handler.
 */
void USBDCallbacks_RequestReceived(const USBGenericRequest *request)
{
	USBDDriver *pUsbd = USBD_GetDriver();

	/* STD requests */
	if (USBGenericRequest_GetType(request) != USBGenericRequest_CLASS) {
		USBDDriver_RequestHandler(pUsbd, request);
		return;
	}

	/* Video requests */
	TRACE_INFO_WP("Vid ");

	switch (USBGenericRequest_GetRequest(request)) {
	case VIDGenericRequest_SETCUR:  VIDD_SetCUR (request);  break;

	case VIDGenericRequest_GETCUR:  VIDD_GetCUR (request);  break;

	case VIDGenericRequest_GETDEF:  VIDD_GetDEF (request);  break;

	case VIDGenericRequest_GETINFO: VIDD_GetINFO(request);  break;

	case VIDGenericRequest_GETMIN:  VIDD_GetMIN (request);  break;

	case VIDGenericRequest_GETMAX:  VIDD_GetMAX (request);  break;

	case VIDGenericRequest_GETRES:  VIDD_GetRES (request);  break;

	default:
		TRACE_WARNING("REQ: %x %x %x %x\n\r",
					  USBGenericRequest_GetType(request),
					  USBGenericRequest_GetRequest(request),
					  USBGenericRequest_GetValue(request),
					  USBGenericRequest_GetLength(request));
		USBD_Stall(0);
	}

	TRACE_INFO_WP("\n\r");
}

/**
 * Invoked whenever the active setting of an interface is changed by the
 * host. Reset streaming interface.
 * \param interface Interface number.
 * \param setting Newly active setting.
 */
void USBDDriverCallbacks_InterfaceSettingChanged(uint8_t interface,
		uint8_t setting)
{
	if (interface != VIDCAMD_StreamInterfaceNum) return;

	if (setting) {
		bVideoON = 1;
		frmC = 0;   frmI = 0;
	} else {
		bVideoON = 0;
		bFrameXfring = 0;
	}

	memory_sync();
	USBD_HAL_ResetEPs(1 << VIDCAMD_IsoInEndpointNum, USBRC_CANCELED, 1);
}

/**
 * \brief Generic ISI & OV sensor initialization
 */
static void _PreviewMode(void)
{
	IsiPrevBuffIndex = 0;
	UsbPrevBuffIndex = 0;
	ISI_SetSensorSize(frmW, frmH);
	ISI_setPreviewSize(frmW, frmH);
	/* calculate scaler factor automatically. */
	ISI_calcScalerFactor();
	ISI_DisableInterrupt(0xFFFFFFFF);
	ISI_DmaChannelDisable(ISI_DMA_CHDR_C_CH_DIS);
	ISI_DmaChannelEnable(ISI_DMA_CHER_P_CH_EN);
	ISI_EnableInterrupt(ISI_IER_PXFR_DONE);
	/* Configure ISI interrupts */
	NVIC_ClearPendingIRQ(ISI_IRQn);
	NVIC_EnableIRQ(ISI_IRQn);
	ISI_Enable();
}

/*----------------------------------------------------------------------------
 *        Global functions
 *----------------------------------------------------------------------------*/

/**
 * \brief Application entry point for ISI USB video example.
 *
 * \return Unused (ANSI-C compatibility).
 */
extern int main(void)
{
	uint8_t i = 0;
	USBDDriver *pUsbd = USBD_GetDriver();

	/* Enable I and D cache */
	SCB_EnableICache();
	SCB_EnableDCache();

	TimeTick_Configure();
	/* Enable SDRAM for Frame buffer */
	BOARD_ConfigureSdram();

	WDT_Disable(WDT);

	/* Output example information */
	printf("-- USB VIDEO Example %s --\n\r", SOFTPACK_VERSION);
	printf("-- %s\n\r", BOARD_NAME);
	printf("-- Compiled: %s %s With %s--\n\r", __DATE__, __TIME__ ,
			COMPILER_NAME);

	__enable_irq();

	/* TWI Initialize */
	_twiInit();

	/* ISI PCK clock Initialize */
	_isiPckInit();
	PIO_Set(&pin_ISI_RST);

	while (1) {
		if (sensor_setup(&twid, sensorProfiles[i], QVGA) == SENSOR_OK) {
			printf("\n\r-I- Sensor %s setup succeed.\n\r", sensorName[i]);
			break;
		}

		if (i < (sizeof(sensorProfiles) / sizeof(sensorProfiles[0])))
			i++;
		else {
			printf("\n\r-E- Sensor setup failed.\n\r");

			while (1);
		}
	}

	/* Retrieve sensor output format and size */
	sensor_get_output(&wImageFormat, &wImageWidth, &wImageHeight, QVGA);

	if (wImageFormat == MONO_12_BIT) {
		printf("-I- Monochrome sensor do not support in this example!");

		while (1);
	}

	if (wImageFormat == RAW_BAYER_12_BIT || wImageFormat == RAW_BAYER_10_BIT)
		wImageFormat = (sensorOutputFormat_t)RGB_INPUT;
	else
		wImageFormat = (sensorOutputFormat_t)YUV_INPUT;

	/* ISI Initialize */
	_isiInit();

	/* Interrupt priority */
	NVIC_SetPriority(ISI_IRQn, 0);
	NVIC_SetPriority(USBHS_IRQn, 2);

	/* Initialize all USB power (off) */
	_ConfigureUotghs();

	/* USB Driver Initialize */
	USBDDriver_Initialize(pUsbd, &usbdDriverDescriptors, bAlternateInterfaces);
	USBD_Init();

	/* Start USB stack to authorize VBus monitoring */
	USBD_Connect();

	_ConfigureTc();

	while (1) {
		if (USBD_GetState() < USBD_STATE_CONFIGURED)
			continue;

		if (bVidON && !bVideoON) {
			bVidON = 0;
			ISI_Disable();
			printf("vidE \n\r");
		}

		if (!bVidON && bVideoON) {
			bVidON = 1;
			NVIC_ClearPendingIRQ(USBHS_IRQn);
			NVIC_DisableIRQ(USBHS_IRQn);
			_PreviewMode();
			/* Start USB Streaming */
			USBD_HAL_SetTransferCallback(VIDCAMD_IsoInEndpointNum,
										 (TransferCallback)VIDD_PayloadSent, 0);
			NVIC_EnableIRQ(USBHS_IRQn);
			VIDD_PayloadSent(NULL, USBD_STATUS_SUCCESS);
			printf("vidS\n\r");
		}
	}
}
