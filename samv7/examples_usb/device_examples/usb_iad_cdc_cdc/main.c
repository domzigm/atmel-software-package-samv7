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

/** \cond usb_iad_cdc_cdc
 * \page usb_iad_cdc_cdc USB DUAL CDC Serial Port Example
 *
 * \section Purpose
 *
 * The USB DUALCDC Project will help you to get familiar with the
 * USB Device Port(UDP)interface . Also it can help you to be familiar
 * with the USB Framework that is used for rapid development of USB-compliant class
 * drivers such as USB Communication Device class (CDC), and how to combine
 * two USB functions to a single composite device (such as Dual CDC port).
 *
 * \section Requirements
 *
 * This package can be used with SAMV71 Xplained Ultra board or SAME70 Xplained board
 * that have UDP interface, depending on the functions included.
 *
 *  \section win_drv_update Windows Driver Update
 *
 * The composite device is generally supported by Microsoft windows, but some
 * patches are needed for muti-interface functions such as CDC & Audio.
 *
 * \section Description
 *
 * This demo simulates 2 USB to RS-232 Serial Port Converter.
 *
 * When the board running this program connected to a host (PC for example), with
 * USB cable, host will notice the attachment of a USB device. No device
 * driver offered for the device now.
 *
 * \section Usage
 *
 *  -# Build the program and download it inside the board.
 *     Please refer to the Getting Started with SAM V71/E70 Microcontrollers.pdf
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
 *     -- USB Dual CDC Device Project xxx --
 *     -- SAMxxxxx-xx
 *     -- Compiled: xxx xx xxxx xx:xx:xx --
 *     \endcode
 * -# When connecting USB cable to windows, the host reports a new USB device
 *  attachment.
 * -# You can use the inf file
 *    libraries\\usb\\device\\composite\\drv\\CompositeCDCSerial.inf
 *    to install the serial  port. Then new
 *    "AT91 USB to Serial Converter (COMx)" appears in the
 *    hardware device list.
 * -# You can run hyperterminal to send data to the port. And it can be seen
 *    at the other hyperterminal connected to the USART port of the board or
 *    another USB serial port.
 *
 * \section References
 * - usb_iad_cdc_cdc/main.c
 * - pio: Pin configurations and peripheral configure.
 * - usb: USB Device Framework, USB CDC driver and UDP interface driver
 *    - \ref usbd_framework
 *        - \ref usbd_api
 *    - \ref usbd_composite "composite"
 *       - \ref usbd_composite_drv
 *    - \ref usbd_cdc "cdc-serial"
 *       - \ref usbd_cdc_serial_drv
 * - projects:
 *    - \ref usb_cdc_serial
 *
 */

/**
 * \file
 *
 * This file contains all the specific code for the
 * usb_iad_cdc_cdc
 *
 */

/*----------------------------------------------------------------------------
 *        Headers
 *----------------------------------------------------------------------------*/

#include "board.h"

#include <USBD_Config.h>

#include <DUALCDCDDriver.h>

#include <string.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

/*---------------------------------------------------------------------------
 *      Definitions
 *---------------------------------------------------------------------------*/

/** Size in bytes of the packet used for reading data from USB */
#define DATAPACKETSIZE \
	CHIP_USB_ENDPOINTS_MAXPACKETSIZE(CDCDSerialDriverDescriptors_DATAIN)

/** Size in bytes of the buffer used for reading data from the USB & USART */
#define DATABUFFERSIZE (DATAPACKETSIZE+2)

/** Pins used for USART transfer */
#define PINS_USART      PIN_USART1_TXD, PIN_USART1_RXD
/** Register base for USART operation */
#define BASE_USART      USART1
/** USART ID */
#define ID_USART        ID_USART1

/*----------------------------------------------------------------------------
 *      External variables
 *----------------------------------------------------------------------------*/

extern const USBDDriverDescriptors dualcdcdDriverDescriptors;

/*---------------------------------------------------------------------------
 *      Internal variables
 *---------------------------------------------------------------------------*/

/** Global DMA driver for all transfer */
static sXdmad dmad;

/** DMA channel for RX */
static uint32_t usartDmaRxChannel;
/** DMA channel for TX */
static uint32_t usartDmaTxChannel;

/** USART link list for data RX */
COMPILER_ALIGNED(32) static LinkedListDescriporView1 dmaRxLinkList[2];

/*- CDC */
/** List of pins that must be configured for use by the application. */
static const Pin pinsUsart[] = {PINS_USART};

/** Double-buffer for storing incoming USART data. */
COMPILER_ALIGNED(32) static uint8_t usartBuffers[2][DATABUFFERSIZE];

/** Current USART buffer index. */
static uint8_t usartCurrentBuffer = 0;

/** Buffer for storing incoming USB data for serial port 0. */
COMPILER_ALIGNED(32) static uint8_t usbSerialBuffer0[DATABUFFERSIZE];
/** Buffer for storing incoming USB data for serial port 1. */
COMPILER_ALIGNED(32) static uint8_t usbSerialBuffer1[DATABUFFERSIZE];

/** Usart opened */
static uint8_t isUsartON = 0;
/** Serial port 0 opened */
static uint8_t isSerialPort0ON = 0;
/** Serial port 1 opened */
static uint8_t isSerialPort1ON = 0;



/*-----------------------------------------------------------------------------
 *         Callback re-implementation
 *-----------------------------------------------------------------------------*/
/**
 * Invoked when the configuration of the device changes. Parse used endpoints.
 * \param cfgnum New configuration number.
 */
void USBDDriverCallbacks_ConfigurationChanged(uint8_t cfgnum)
{
	DUALCDCDDriver_ConfigurationChangeHandler(cfgnum);
}

/**
 * Invoked when a new SETUP request is received from the host. Forwards the
 * request to the Mass Storage device driver handler function.
 * \param request  Pointer to a USBGenericRequest instance.
 */
void USBDCallbacks_RequestReceived(const USBGenericRequest *request)
{
	DUALCDCDDriver_RequestHandler(request);
}

/*----------------------------------------------------------------------------
 *         Internal functions
 *---------------------------------------------------------------------------*/

/**
 * ISR for xDMA interrupt
 */
void XDMAC_Handler(void)
{
	XDMAD_Handler(&dmad);
}

/**
 *  \brief Send single buffer data through xDMA
 */
static void _UsartDmaTx(uint32_t dwDestAddr, void *pBuffer, uint16_t wSize)
{
	sXdmad *pDmad = &dmad;
	/* Setup transfer */
	sXdmadCfg xdmadCfg;
	xdmadCfg.mbr_ubc = wSize;
	xdmadCfg.mbr_sa = (uint32_t) pBuffer;
	xdmadCfg.mbr_da = (uint32_t) dwDestAddr;
	xdmadCfg.mbr_cfg = XDMAC_CC_TYPE_PER_TRAN
					   | XDMAC_CC_MEMSET_NORMAL_MODE
					   | XDMAC_CC_DSYNC_MEM2PER
					   | XDMAC_CC_CSIZE_CHK_1
					   | XDMAC_CC_DWIDTH_BYTE
					   | XDMAC_CC_SIF_AHB_IF1
					   | XDMAC_CC_DIF_AHB_IF1
					   | XDMAC_CC_SAM_INCREMENTED_AM
					   | XDMAC_CC_DAM_FIXED_AM
					   | XDMAC_CC_PERID(XDMAIF_Get_ChannelNumber(
											ID_USART, XDMAD_TRANSFER_TX));
	xdmadCfg.mbr_bc = 0;
	XDMAD_ConfigureTransfer(pDmad, usartDmaTxChannel, &xdmadCfg, 0, 0, (
								 XDMAC_CIE_BIE   |
								 XDMAC_CIE_DIE   |
								 XDMAC_CIE_FIE   |
								 XDMAC_CIE_RBIE  |
								 XDMAC_CIE_WBIE  |
								 XDMAC_CIE_ROIE));
	SCB_CleanDCache_by_Addr((uint32_t *)pBuffer, wSize);
	XDMAD_StartTransfer(pDmad, usartDmaTxChannel);
}

/**
 *  \brief Prepare link list for USART RX
 *  Ringed link list initialized for 2 USART buffer.
 */
static void _UsartDmaRxSetup(void)
{
	uint8_t i;
	Usart *pUs = BASE_USART;

	for (i = 0; i < 2; i++) {
		dmaRxLinkList[i].mbr_ubc = XDMA_UBC_NVIEW_NDV1
								   | XDMA_UBC_NDE_FETCH_EN
								   | XDMA_UBC_NSEN_UPDATED
								   | XDMAC_CUBC_UBLEN(DATAPACKETSIZE);
		dmaRxLinkList[i].mbr_sa  = (uint32_t)&pUs->US_RHR;
		dmaRxLinkList[i].mbr_da = (uint32_t)usartBuffers[i];
		dmaRxLinkList[i].mbr_nda = (uint32_t)&dmaRxLinkList[i];
	}
}

/**
 *  \brief Start waiting USART data
 *  Start DMA, the 1st DMA buffer is free USART buffer assigned.
 */
static void _UsartDmaRx(uint32_t startBuffer)
{
	sXdmad *pDmad = &dmad;
	sXdmadCfg xdmadUsartRxCfg;
	uint32_t xdmaUsartRxCndc;
	xdmadUsartRxCfg.mbr_cfg = XDMAC_CC_TYPE_PER_TRAN
							  | XDMAC_CC_MBSIZE_SINGLE
							  | XDMAC_CC_DSYNC_PER2MEM
							  | XDMAC_CC_CSIZE_CHK_1
							  | XDMAC_CC_DWIDTH_BYTE
							  | XDMAC_CC_SIF_AHB_IF1
							  | XDMAC_CC_DIF_AHB_IF1
							  | XDMAC_CC_SAM_FIXED_AM
							  | XDMAC_CC_DAM_INCREMENTED_AM
							  | XDMAC_CC_PERID(XDMAIF_Get_ChannelNumber(
										  ID_USART, XDMAD_TRANSFER_RX));
	xdmaUsartRxCndc = XDMAC_CNDC_NDVIEW_NDV1
					  | XDMAC_CNDC_NDE_DSCR_FETCH_EN
					  | XDMAC_CNDC_NDSUP_SRC_PARAMS_UPDATED
					  | XDMAC_CNDC_NDDUP_DST_PARAMS_UPDATED;
	XDMAD_ConfigureTransfer(pDmad, usartDmaRxChannel, &xdmadUsartRxCfg,
							 xdmaUsartRxCndc, (uint32_t)&dmaRxLinkList[startBuffer], XDMAC_CIE_LIE);
	SCB_CleanDCache_by_Addr((uint32_t *)dmaRxLinkList, sizeof(dmaRxLinkList));
	XDMAD_StartTransfer(pDmad, usartDmaRxChannel);
}

/**
 * Handles interrupts coming from Timer #0.
 */
void TC0_Handler(void)
{
	Tc *pTc0 = TC0;
	sXdmad *pDmad = &dmad;
	uint8_t size;
	uint32_t nextBuffer = 1 - usartCurrentBuffer;
	uint32_t dmaChannel = usartDmaRxChannel;
	uint8_t iChannel = (dmaChannel) & 0xFF;
	volatile uint32_t status = pTc0->TC_CHANNEL[0].TC_SR;

	if ((status & TC_SR_CPCS) != 0) {
		/* Serial Timer */
		if (isUsartON) {
			/* Get Received size */
			size = XDMAC_GetChDestinationAddr(pDmad->pXdmacs, iChannel)
				   - (uint32_t)(usartBuffers[usartCurrentBuffer]);

			if (size == 0) {
				pTc0->TC_CHANNEL[0].TC_CCR = TC_CCR_CLKEN | TC_CCR_SWTRG;
				return;
			}

			/* Stop DMA */
			XDMAD_StopTransfer(pDmad, dmaChannel);
			/* Restart DMA in next buffer */
			_UsartDmaRx(nextBuffer);

			/* Send current buffer through the USB */
			if (isSerialPort0ON) {
				CDCDSerialPort_Write(DUALCDCDDriver_GetSerialPort(0),
									 usartBuffers[usartCurrentBuffer],
									 size, 0, 0);
			}

			if (isSerialPort1ON) {
				CDCDSerialPort_Write(DUALCDCDDriver_GetSerialPort(1),
									 usartBuffers[usartCurrentBuffer],
									 size, 0, 0);
			}

			usartCurrentBuffer = nextBuffer;
			pTc0->TC_CHANNEL[0].TC_CCR = TC_CCR_CLKEN | TC_CCR_SWTRG;
		}
	}
}

/**
 * Callback invoked when data has been received on the USB.
 * For USB CDC Serial Port 0
 */
static void _UsbDataReceived0(uint32_t unused,
							  uint8_t status,
							  uint32_t received,
							  uint32_t remaining)
{
	Usart *pUs = BASE_USART;
	unused = unused; /*dummy */

	/* Check that data has been received successfully */
	if (status == USBD_STATUS_SUCCESS) {
		/* Send data through USBSerial 1 */
		while (CDCDSerialPort_Write(DUALCDCDDriver_GetSerialPort(1),
									usbSerialBuffer0,
									received, 0, 0) != USBD_STATUS_SUCCESS);

		/* Send data through USART */
		_UsartDmaTx((uint32_t)&pUs->US_THR, usbSerialBuffer0, received);

		/* Check if bytes have been discarded */
		if ((received == DATABUFFERSIZE) && (remaining > 0)) {
			TRACE_WARNING(
				"_UsbDataReceived0: %u bytes discarded\n\r",
				(unsigned int)remaining);
		}
	} else {
		TRACE_WARNING("_UsbDataReceived0: Transfer error\n\r");
	}
}

/**
 * Callback invoked when data has been received on the USB.
 * For USB CDC Serial Port 1
 */
static void _UsbDataReceived1(uint32_t unused,
							  uint8_t status,
							  uint32_t received,
							  uint32_t remaining)
{
	Usart *pUs = BASE_USART;
	unused = unused; /*dummy */

	/* Check that data has been received successfully */
	if (status == USBD_STATUS_SUCCESS) {
		/* Send data through USBSerial 0 */
		while (CDCDSerialPort_Write(DUALCDCDDriver_GetSerialPort(0),
									usbSerialBuffer1,
									received, 0, 0) != USBD_STATUS_SUCCESS);

		/* Send data through USART */
		_UsartDmaTx((uint32_t)&pUs->US_THR, usbSerialBuffer1, received);

		/* Check if bytes have been discarded */
		if ((received == DATABUFFERSIZE) && (remaining > 0)) {
			TRACE_WARNING(
				"_UsbDataReceived1: %u bytes discarded\n\r",
				(unsigned int)remaining);
		}
	} else {
		TRACE_WARNING("_UsbDataReceived1: Transfer error\n\r");
	}
}

/**
 * \brief DMA RX callback function
 */
static void _UsDmaRxCallback(uint32_t channel, void *pArg)
{
    channel = channel;
	pArg = pArg; /* dummy */
	/* Disable timer */
	TC_Stop(TC0, 0);

	/* Send data through USBSerial 0 */
	if (isSerialPort0ON) {
		while (CDCDSerialPort_Write(DUALCDCDDriver_GetSerialPort(0),
									usartBuffers[usartCurrentBuffer],
									DATAPACKETSIZE, 0, 0) != USBD_STATUS_SUCCESS);
	}

	/* Send data through USBSerial 1 */
	if (isSerialPort1ON) {
		while (CDCDSerialPort_Write(DUALCDCDDriver_GetSerialPort(1),
									usartBuffers[usartCurrentBuffer],
									DATAPACKETSIZE, 0, 0) != USBD_STATUS_SUCCESS);
	}

	/* Restart read on buffer */
	usartCurrentBuffer = 1 - usartCurrentBuffer;
	_UsartDmaRx(usartCurrentBuffer);
	/* Restart timer */
	TC_Start(TC0, 0);
}

/**
 * \brief DMA TX callback function
 */
static void _UsDmaTxCallback(uint32_t channel, void *pArg)
{
  /* dummy */
	pArg = pArg;
    channel = channel;

	/* Restart USB read */
	if (isSerialPort0ON) {
		CDCDSerialPort_Read(DUALCDCDDriver_GetSerialPort(0),
							usbSerialBuffer0,
							DATAPACKETSIZE,
							(TransferCallback) _UsbDataReceived0,
							0);
	}

	if (isSerialPort1ON) {
		CDCDSerialPort_Read(DUALCDCDDriver_GetSerialPort(1),
							usbSerialBuffer1,
							DATAPACKETSIZE,
							(TransferCallback) _UsbDataReceived1,
							0);
	}
}

/*----------------------------------------------------------------------------
 * Handles interrupts coming from USART
 *----------------------------------------------------------------------------*/
void USART1_Handler(void)
{
	Usart *pUs = BASE_USART;
	uint32_t status;
	uint16_t serialState;

	status  = USART_GetStatus(pUs);
	status &= USART_GetItMask(pUs);

	/* If USB device is not configured, do nothing */
	if (!isUsartON) {
		USART_DisableIt(pUs, 0xFFFFFFFF);
		return;
	}

	/* Errors */
	serialState = CDCDSerialPort_GetSerialState(
					  DUALCDCDDriver_GetSerialPort(0));

	/* Overrun */
	if ((status & US_CSR_OVRE) != 0) {
		TRACE_WARNING("USART1_Handler: Overrun\n\r");
		serialState |= CDCSerialState_OVERRUN;
	}

	/* Framing error */
	if ((status & US_CSR_FRAME) != 0) {
		TRACE_WARNING("USART1_Handler: Framing error\n\r");
		serialState |= CDCSerialState_FRAMING;
	}

	CDCDSerialPort_SetSerialState(
		DUALCDCDDriver_GetSerialPort(0), serialState);
}

/*---------------------------------------------------------------------------
 *         Internal functions
 *---------------------------------------------------------------------------*/

/**
 * \brief DMA driver configuration
 */
static void _ConfigureDma(void)
{
	sXdmad *pDmad = &dmad;
	/* Driver initialize */
	XDMAD_Initialize(&dmad, 0);
	/* IRQ configure */
	NVIC_EnableIRQ(XDMAC_IRQn);

	/* Allocate DMA channels for USART */
	usartDmaTxChannel = XDMAD_AllocateChannel(pDmad, XDMAD_TRANSFER_MEMORY,
						ID_USART);
	usartDmaRxChannel = XDMAD_AllocateChannel(pDmad, ID_USART,
						XDMAD_TRANSFER_MEMORY);

	/* Set RX callback */
	XDMAD_SetCallback(pDmad, usartDmaRxChannel,
					  (XdmadTransferCallback)_UsDmaRxCallback
					  , 0);
	/* Set TX callback */
	XDMAD_SetCallback(pDmad, usartDmaTxChannel,
					  (XdmadTransferCallback)_UsDmaTxCallback
					  , 0);
	XDMAD_PrepareChannel(pDmad, usartDmaRxChannel);
	XDMAD_PrepareChannel(pDmad, usartDmaTxChannel);

}

/**
 * Configure USART to work at 115200
 */
static void _ConfigureUsart(void)
{
	PIO_Configure(pinsUsart, PIO_LISTSIZE(pinsUsart));
	PMC_EnablePeripheral(ID_USART);
	USART_DisableIt(BASE_USART, 0xFFFFFFFF);
	USART_Configure(BASE_USART,
					USART_MODE_ASYNCHRONOUS,
					115200,
					BOARD_MCK);

	USART_SetTransmitterEnabled(BASE_USART, 1);
	USART_SetReceiverEnabled(BASE_USART, 1);
	NVIC_EnableIRQ(USART1_IRQn);
}

/**
 * Configure TC0 to generate an interrupt every 4ms
 */
static void _ConfigureTc0(void)
{
	uint32_t div, tcclks;

	/* Enable TC0 peripheral */
	PMC_EnablePeripheral(ID_TC0);
	/* Configure TC0 for 250Hz frequency and trigger on RC compare */
	TC_FindMckDivisor(250, BOARD_MCK, &div, &tcclks, BOARD_MCK);
	TC_Configure(TC0, 0, tcclks | TC_CMR_CPCTRG);
	TC0->TC_CHANNEL[0].TC_RC = (BOARD_MCK / div) / 250;
	/* Configure and enable interrupt on RC compare */
	NVIC_EnableIRQ(TC0_IRQn);
	TC0->TC_CHANNEL[0].TC_IER = TC_IER_CPCS;
	/* Start TC when USB connected & RX enabled */
}

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
/*---------------------------------------------------------------------------
 *         Exported function
 *---------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------
 *          Main
 *---------------------------------------------------------------------------*/

/**
 * Initializes drivers and start the USB Dual CDC device.
 */
int main(void)
{
	uint8_t usbConnected = 0;
	uint8_t serial0ON = 0, serial1ON = 0, usartON = 0;

	/* Disable watchdog */
	WDT_Disable(WDT);

	SCB_EnableICache();
	SCB_EnableDCache();

	printf("-- USB Dual CDC Device Project %s --\n\r", SOFTPACK_VERSION);
	printf("-- %s\n\r", BOARD_NAME);
	printf("-- Compiled: %s %s With %s--\n\r", __DATE__, __TIME__ ,
			COMPILER_NAME);

	/* If they are present, configure Vbus & Wake-up pins */
	PIO_InitializeInterrupts(0);

	/* If there is on board power, switch it off */
	_ConfigureUotghs();

	/* Configure DMA */
	_ConfigureDma();

	/* Configure timer 0 */
	_ConfigureTc0();

	/* ----- CDC Function Initialize */
	/* Configure USART */
	_ConfigureUsart();
	/* Configure DMA for USART */
	_UsartDmaRxSetup();

	/* USB DualCDC driver initialization */
	DUALCDCDDriver_Initialize(&dualcdcdDriverDescriptors);

	/* connect if needed */
	USBD_Connect();

	/* Driver loop */
	while (1) {
		/* Device is not configured */
		if (USBD_GetState() < USBD_STATE_CONFIGURED) {
			if (usbConnected) {
				printf("-I- USB Disconnect/Suspend\n\r");
				usbConnected = 0;
				/* Serial port closed */
				isSerialPort0ON = 0;
				isSerialPort1ON = 0;
				isUsartON = 0;
			}
		} else {
			isSerialPort0ON = CDCDSerialPort_GetControlLineState(
								  DUALCDCDDriver_GetSerialPort(0))
							  & CDCControlLineState_DTR;
			isSerialPort1ON = CDCDSerialPort_GetControlLineState(
								  DUALCDCDDriver_GetSerialPort(1))
							  & CDCControlLineState_DTR;

			if (usbConnected == 0) {
				printf("-I- USB Connect\n\r");
				usbConnected = 1;
			}

			isUsartON = isSerialPort0ON || isSerialPort1ON;

			if (!usartON && isUsartON) {
				usartON = 1;
				printf("-I- USART ON\n\r");
				/* Start receiving data on the USART */
				usartCurrentBuffer = 0;
				TC_Start(TC0, 0);
				_UsartDmaRx(usartCurrentBuffer);
				USART_EnableIt(BASE_USART, US_CSR_FRAME | US_CSR_OVRE);
			} else if (usartON && !isUsartON) {
				usartON = 0;
				printf("-I- USART OFF\n\r");
			}

			if (!serial0ON && isSerialPort0ON) {
				serial0ON = 1;
				printf("-I- SerialPort0 ON\n\r");
				/* Start receiving data on the USB */
				CDCDSerialPort_Read(DUALCDCDDriver_GetSerialPort(0),
									usbSerialBuffer0,
									DATAPACKETSIZE,
									(TransferCallback) _UsbDataReceived0,
									0);
			} else if (serial0ON && !isSerialPort0ON) {
				serial0ON = 0;
				printf("-I- SeriaoPort0 OFF\n\r");
			}

			if (!serial1ON && isSerialPort1ON) {
				serial1ON = 1;
				printf("-I- SerialPort1 ON\n\r");
				/* Start receiving data on the USB */
				CDCDSerialPort_Read(DUALCDCDDriver_GetSerialPort(1),
									usbSerialBuffer1,
									DATAPACKETSIZE,
									(TransferCallback) _UsbDataReceived1,
									0);
			} else if (serial1ON && !isSerialPort1ON) {
				serial1ON = 0;
				printf("-I- SeriaoPort1 OFF\n\r");
			}
		}
	}
}
/** \endcond */
