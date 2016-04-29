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
#include <UVCDriver.h>
#include <UVCFunction.h>

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
static ISI_FrameBufferDescriptors  preBufDescList[ISI_MAX_PREV_BUFFER ];
COMPILER_WORD_ALIGNED
static uint8_t pXfrBuffers[FRAME_PACKET_SIZE_HS * (ISO_HIGH_BW_MODE + 1)][1];


volatile uint8_t bVidON = 0;

static volatile uint8_t IsiPrevBuffIndex;
static volatile uint8_t UsbPrevBuffIndex;
static volatile uint32_t displayFrameAddr;

/* Image size in preview mode */
static uint32_t wImageWidth, wImageHeight;

/* Image output format */
static sensorOutputFormat_t wImageFormat;

extern const sensorProfile_t ov2643Profile;
extern const sensorProfile_t ov5640Profile;
extern const sensorProfile_t ov7740Profile;
extern const sensorProfile_t ov9740Profile;
extern const sensorProfile_t ov7670Profile;


static const sensorProfile_t *sensorProfiles[] = {
	&ov2643Profile, &ov5640Profile, &ov7740Profile, &ov9740Profile,&ov7670Profile
};

static const char sensorName[][8] = {
	"OV2643", "OV5640", "OV7740", "OV9740","OV7670"
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
	framebufOffset =  FRAME_BUFFER_SIZEC(VIDEO_WIDTH, VIDEO_HEIGHT);

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
	ISI_setPreviewSize(wImageWidth,wImageHeight);

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


/*---------------------------------------------------------------------------
 *         VBus monitoring (optional)
 *---------------------------------------------------------------------------*/
#ifdef VBUS_DETECTION

/** VBus pin instance. */
static const Pin pinVbus = PIN_USB_VBUS;
/**
 * Handles interrupts coming from PIO controllers.
 */
static void ISR_Vbus(const Pin *pPin)
{
	/* Check current level on VBus */
	if (PIO_Get(&pinVbus)) {

		TRACE_INFO("VBUS conn\n\r");
		USBD_Connect();
	}
	else {

		TRACE_INFO("VBUS discon\n\r");
		USBD_Disconnect();
	}
}
#endif


/**
 * Configures the VBus pin to trigger an interrupt when the level on that pin
 * changes if it exists.
 */
static void VBus_Configure( void )
{
	TRACE_INFO("VBus configuration\n\r");

#ifdef VBUS_DETECTION
	/* Configure PIO */
	PIO_Configure(&pinVbus, PIO_LISTSIZE(pinVbus));
	PIO_ConfigureIt(&pinVbus, ISR_Vbus);
	PIO_EnableIt(&pinVbus);

	/* Check current level on VBus */
	if (PIO_Get(&pinVbus)) {

		/* if VBUS present, force the connect */
		TRACE_INFO("conn\n\r");
		USBD_Connect();
	}
	else {
		USBD_Disconnect();
	}
#else
	printf("No VBus Monitor\n\r");
	USBD_Connect();

#endif

}


/**
 * Callback that invoked when USB packet is sent.
 */
void VIDD_PayloadSent(void *pArg, uint8_t bStat)
{
	/*dummy*/
	pArg = pArg;
	bStat = bStat;
	uint32_t pktSize;
	uint8_t *pX = pXfrBuffers[0];
	struct _uvc_driver *pUVC_driver = UVC_get_driver();
	uint32_t frmSize = FRAME_BUFFER_SIZEC(pUVC_driver->frm_width,
			pUVC_driver->frm_height);
	uint8_t *pV = (uint8_t*)pUVC_driver->buf;
	USBVideoPayloadHeader *pHdr = (USBVideoPayloadHeader *)pX;
	uint32_t maxPktSize = USBD_IsHighSpeed() ? (UVC_get_frmMaxPktSize())
						: (FRAME_PACKET_SIZE_FS);
	pktSize = frmSize - pUVC_driver->frm_index;
	pHdr->bHeaderLength     = FRAME_PAYLOAD_HDR_SIZE;
	pHdr->bmHeaderInfo.B    = 0;

	if (pktSize > maxPktSize - pHdr->bHeaderLength)
		pktSize = maxPktSize - pHdr->bHeaderLength;

	pV = &pV[pUVC_driver->frm_index];
	pUVC_driver->frm_index += pktSize;
	pHdr->bmHeaderInfo.bm.FID = (pUVC_driver->frm_count & 1);

	if (pUVC_driver->frm_index >= frmSize) {
		pUVC_driver->frm_count ++;
		pUVC_driver->frm_index = 0;
		pHdr->bmHeaderInfo.bm.EoF = 1;

		if (pUVC_driver->is_frame_xfring) {
			pUVC_driver->is_frame_xfring = 0;
		}
	} else {
		pHdr->bmHeaderInfo.bm.EoF = 0;

		if (pUVC_driver->is_frame_xfring == 0) {
			pUVC_driver->is_frame_xfring = 1;

			while (UsbPrevBuffIndex == (uint8_t)(IsiPrevBuffIndex))
				memory_barrier();

			UsbPrevBuffIndex = (uint8_t)(IsiPrevBuffIndex);
			pUVC_driver->buf = (uint32_t *)(displayFrameAddr);
			memory_barrier();
		}
	}

	pHdr->bmHeaderInfo.bm.EOH = 1;

	USBD_HAL_WrWithHdr(VIDCAMD_IsoInEndpointNum, pHdr,
					pHdr->bHeaderLength, pV, pktSize);
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

	/* UVC Driver Initialize */
	UVCDriver_Init(&usbdDriverDescriptors, (uint32_t *)pVideoBufffers);

	/* Start USB stack to authorize VBus monitoring */
	VBus_Configure();

	_ConfigureTc();

	while (1) {
		if (USBD_GetState() < USBD_STATE_CONFIGURED)
			continue;

		if (bVidON && !UVC_is_video_on()) {
			bVidON = 0;
			ISI_Disable();
			printf("vidE \n\r");
		}

		if (!bVidON && UVC_is_video_on()) {
			bVidON = 1;
			NVIC_ClearPendingIRQ(USBHS_IRQn);
			NVIC_DisableIRQ(USBHS_IRQn);
			IsiPrevBuffIndex = 0;
			UsbPrevBuffIndex = 0;
			_PreviewMode(UVC_frm_width(),UVC_frm_height());
			/* Start USB Streaming */
			USBD_HAL_SetTransferCallback(VIDCAMD_IsoInEndpointNum,
							 (TransferCallback)VIDD_PayloadSent, NULL);
			NVIC_EnableIRQ(USBHS_IRQn);
			VIDD_PayloadSent(NULL, USBD_STATUS_SUCCESS);
			printf("vidS\n\r");
		}
	}
}
