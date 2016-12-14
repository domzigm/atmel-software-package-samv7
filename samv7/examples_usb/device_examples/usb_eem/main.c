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

/** \cond usb_eem
 *  \page usb_eem USB Example for Ethernet Emulation Model Devices
 *
 *  \section Purpose
 *
 *  The USB communication device class Example will help you to get familiar with the
 *  USB communication device class on SAMV7/E7 Microcontrollers. Also
 *  it can help you to be familiar with the USB Framework that is used for
 *  rapid development of USB-compliant class drivers such as USB communication device
 *  class.
 *
 *  \section Requirements
 *
 *  This package can be used with SAMV71 Xplained Ultra board or SAME70 Xplained board.
 *  The device uses the USB communication device class (CDC) drivers to take advantage of the installed PC RS-232
 *  software to talk over the USB.
 *
 *  \section Usage
 *
 *  -# Build the program and download it inside the board.
 *     Please refer to the Getting Started with SAM V71/E70 Microcontrollers.pdf
 *  -# On the computer, open and configure a terminal application
 *     (e.g. HyperTerminal on Microsoft Windows) with these settings:
 *    - 115200 baud rate
 *    - 8 bits of data
 *    - No parity
 *    - 1 stop bit
 *    - No flow control
 *  -# Start the application.
 *  -# In the terminal window, the following text should appear:
 *  \code
 *  -- USB Device CDC EEM Project xxx --
 *  -- SAMxxxxx-xx
 *  -- Compiled: xxx xx xxxx xx:xx:xx --
 *  \endcode
 *
 *  \section References
 *  - usb_eem/main.c
 *  - usb: USB Framework, USB communication device class driver
 *      - \ref usbd_framework
 *      - \ref usbd_api
 *      - \ref usbd_cdc
 */

/**
 *  \file
 *
 *  This file contains all the specific code for the
 *  usb_eem example.
 */
/*----------------------------------------------------------------------------
 *         Headers
 *----------------------------------------------------------------------------*/

#include "board.h"

#include "USBD.h"
#include "CDCDEEMDriver.h"

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

/*----------------------------------------------------------------------------
 *      Definitions
 *----------------------------------------------------------------------------*/

/** Size in bytes of the packet used for reading data from USB */
#define MAXPACKETSIZE       1536
#if defined   (__CC_ARM) &&  defined   (sram)
	#define ETH_TX_BUFFERS       32     /** Must be a power of 2 */
	#define ETH_RX_BUFFERS       32     /** Must be a power of 2 */
#else
	#define ETH_TX_BUFFERS       64     /** Must be a power of 2 */
	#define ETH_RX_BUFFERS       64     /** Must be a power of 2 */
#endif

#define ETH_BUFF_SIZE        1536

#define DUMMY_BUFFERS        2
#define DUMMY_BUFF_SIZE      128

/*----------------------------------------------------------------------------
 *      External variables
 *----------------------------------------------------------------------------*/

extern const USBDDriverDescriptors cdcdEEMDriverDescriptors;

/*----------------------------------------------------------------------------
 *      Internal variables
 *----------------------------------------------------------------------------*/

static uint8_t gUsbConnected = 0;

/** Buffer for storing incoming USB data. */
static uint8_t gUsbRxBuffer[MAXPACKETSIZE];

/** The PINs for GMAC */
static const Pin gmacPins[]      = { BOARD_GMAC_RUN_PINS };
static const Pin gmacResetPins[] = { BOARD_GMAC_RESET_PIN };

/** The MAC address used for demo */
static uint8_t gMacAddress[6] = { 0x3a, 0x1f, 0x34, 0x08, 0x54, 0x54 };

/** The GMAC driver instance */
static sGmacd gGmacd;

/** The MACB driver instance */
static GMacb gGmacb;

/** TX descriptors list */
COMPILER_SECTION(".ram_nocache")
COMPILER_ALIGNED(8)
static sGmacTxDescriptor gTxDs[ETH_TX_BUFFERS], gDummyTxDs[DUMMY_BUFFERS];

/** TX callbacks list */
COMPILER_ALIGNED(8) static fGmacdTransferCallback gTxCbs[ETH_TX_BUFFERS],
				 gDummyTxCbs[DUMMY_BUFFERS];

/** RX descriptors list */
COMPILER_SECTION(".ram_nocache")
COMPILER_ALIGNED(8)
static sGmacRxDescriptor gRxDs[ETH_RX_BUFFERS], gDummyRxDs[DUMMY_BUFFERS];

/** Send Buffer */
/* Section 3.6 of AMBA 2.0 spec states that burst should not cross 1K Boundaries.
   Receive buffer manager writes are burst of 2 words => 3 lsb bits of the address
   shall be set to 0 */
COMPILER_ALIGNED(8)
static uint8_t gTxBuffer[ETH_TX_BUFFERS * ETH_BUFF_SIZE],
gTxDummyBuffer[DUMMY_BUFFERS * DUMMY_BUFF_SIZE];

/** Receive Buffer */
COMPILER_ALIGNED(8)
static uint8_t gRxBuffer[ETH_RX_BUFFERS * ETH_BUFF_SIZE],
gRxDummyBuffer[DUMMY_BUFFERS * DUMMY_BUFF_SIZE];


/*----------------------------------------------------------------------------
 *         Internal Prototypes
 *----------------------------------------------------------------------------*/

/*----------------------------------------------------------------------------
 *  Interrupt handlers
 *----------------------------------------------------------------------------*/

/**
 * Gmac interrupt handler
 */
void GMAC_Handler(void)
{
	GMACD_Handler(&gGmacd, GMAC_QUE_0);
}

/*-----------------------------------------------------------------------------
 *         Callback re-implementation
 *-----------------------------------------------------------------------------*/

/**
 * Invoked when the configuration of the device changes. Parse used endpoints.
 * \param cfgnum New configuration number.
 */
void USBDDriverCallbacks_ConfigurationChanged(unsigned char cfgnum)
{
	CDCDEEMDriver_ConfigurationChangedHandler(cfgnum);
}

/**
 * Invoked when a new SETUP request is received from the host. Forwards the
 * request to the Mass Storage device driver handler function.
 * \param request  Pointer to a USBGenericRequest instance.
 */
void USBDCallbacks_RequestReceived(const USBGenericRequest *request)
{
	CDCDEEMDriver_RequestHandler(request);
}

/*----------------------------------------------------------------------------
 *         Internal functions
 *----------------------------------------------------------------------------*/

/**
 * Configure USBHS settings for USB device
 */
static void _ConfigureUotghs(void)
{
	/* UTMI parallel mode, High/Full/Low Speed */
	/* UOTGCK not used in this configuration (High Speed) */
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
}

/*----------------------------------------------------------------------------
 * Callback invoked when data has been received on the USB.
 *----------------------------------------------------------------------------*/
static void _UsbDataReceived(void *unused,
							 uint8_t status,
							 uint32_t received,
							 uint32_t remaining)
{
	// unused args
	(void)unused;
	(void)remaining;

	/* Check that data has been received successfully */
	if (status == USBD_STATUS_SUCCESS) {
		TRACE_INFO("%u USB_RECV(%u)\n\r",
				   (unsigned int)GetTicks(), (unsigned int)received);

		/* Send packet through GMAC */
		if (GMACD_Send(&gGmacd, gUsbRxBuffer, received, NULL, GMAC_QUE_0)
			!= GMACD_OK)
			TRACE_WARNING("_UsbDataReceived: GMAC send overflow\n\r");
	} else {
		TRACE_WARNING("_UsbDataReceived: Transfer error\n\r");
	}

	/* receive next packet on USB */
	CDCDEEMDriver_Read(gUsbRxBuffer, MAXPACKETSIZE, _UsbDataReceived, 0);
}

/*----------------------------------------------------------------------------
 * Callback invoked when data has been received on the GMAC.
 *----------------------------------------------------------------------------*/
static void _EthDataReceived(uint32_t status)
{
	uint8_t buffer[MAXPACKETSIZE];
	uint32_t frmSize;

	// unused args
	(void)status;

	TRACE_INFO("%u ETH_RXCB(%u)\n\r", (unsigned int)GetTicks(),
			   (unsigned int)status);

	/* Handle ethernet data if any */
	while (
		GMACD_OK == GMACD_Poll(&gGmacd, buffer, sizeof(buffer), &frmSize, GMAC_QUE_0)) {
		TRACE_INFO("%u ETH_RECV(%u)\n\r", (unsigned int)GetTicks(),
				   (unsigned int)frmSize);

		/* Send packet through USB */
		if (CDCDEEMDriver_Write(buffer, frmSize, NULL, 0) != USBD_STATUS_SUCCESS) {
			TRACE_WARNING("_EthDataReceived: USB send overflow\n\r");
		}
	}

	/* receive next packet on GMAC */
	GMACD_SetRxCallback(&gGmacd, _EthDataReceived, GMAC_QUE_0);
}

/*_____ E X P O R T E D   F U N C T I O N S ________________________________*/

/*! \brief Main function. Execution starts here.
 */
int main(void)
{
	sGmacInit Queue0, Queue12;

	/* Disable watchdog */
	WDT_Disable(WDT);

	SCB_EnableICache();
	SCB_EnableDCache();

	/* Output example information */
	printf("-- USB Device CDC EEM Project %s --\n\r", SOFTPACK_VERSION);
	printf("-- %s\n\r", BOARD_NAME);
	printf("-- Compiled: %s %s  With %s--\n\r", __DATE__, __TIME__ ,
			COMPILER_NAME);

	/* Initialize OTG clocks */
	_ConfigureUotghs();

	/* Configure systick for 1 ms. */
	TimeTick_Configure();

	/* Initialize GMAC driver structure for queue 0 */
	memset(&Queue0, 0, sizeof(Queue0));
	Queue0.bIsGem = 1;
	Queue0.bDmaBurstLength = 4;
	Queue0.pRxBuffer = gRxBuffer;
	Queue0.pRxD = gRxDs;
	Queue0.wRxBufferSize = ETH_BUFF_SIZE;
	Queue0.wRxSize = ETH_RX_BUFFERS;
	Queue0.pTxBuffer = gTxBuffer;
	Queue0.pTxD = gTxDs;
	Queue0.wTxBufferSize = ETH_BUFF_SIZE;
	Queue0.wTxSize = ETH_TX_BUFFERS;
	Queue0.pTxCb = gTxCbs;

	/* Initialize dummy GMAC driver structure for queues 1 and 2 */
	memset(&Queue12, 0, sizeof(Queue12));
	Queue12.bIsGem = 1;
	Queue12.bDmaBurstLength = 4;
	Queue12.pRxBuffer = gRxDummyBuffer;
	Queue12.pRxD = gDummyRxDs;
	Queue12.wRxBufferSize = DUMMY_BUFF_SIZE;
	Queue12.wRxSize = DUMMY_BUFFERS;
	Queue12.pTxBuffer = gTxDummyBuffer;
	Queue12.pTxD = gDummyTxDs;
	Queue12.wTxBufferSize = DUMMY_BUFF_SIZE;
	Queue12.wTxSize = DUMMY_BUFFERS;
	Queue12.pTxCb = gDummyTxCbs;

	/* Initialize GMAC */
	GMACD_Init(&gGmacd, GMAC, ID_GMAC, 1, 0);
	GMACD_InitTransfer(&gGmacd, &Queue12,  GMAC_QUE_2);
	GMACD_InitTransfer(&gGmacd, &Queue12,  GMAC_QUE_1);
	GMACD_InitTransfer(&gGmacd, &Queue0, GMAC_QUE_0);
	GMAC_SetAddress(gGmacd.pHw, 0, gMacAddress);

	/* Setup interrupts */
	NVIC_ClearPendingIRQ(GMAC_IRQn);
	NVIC_EnableIRQ(GMAC_IRQn);
	NVIC_ClearPendingIRQ(USBHS_IRQn);
	NVIC_EnableIRQ(USBHS_IRQn);

	/* PHY initialize */
	GMACB_Init(&gGmacb, &gGmacd, BOARD_GMAC_PHY_ADDR);
	GMACB_ResetPhy(&gGmacb);

	if (!GMACB_InitPhy(
			&gGmacb, BOARD_MCK, gmacResetPins, 1, gmacPins, PIO_LISTSIZE(gmacPins))) {
		TRACE_ERROR("PHY Initialize ERROR!\n\r");
		return 0;
	}

	if (!GMACB_AutoNegotiate(&gGmacb)) {
		TRACE_ERROR("Auto Negotiate ERROR!\n\r");
		return 0;
	}

	/* CDC EEM driver initialization */
	CDCDEEMDriver_Initialize(&cdcdEEMDriverDescriptors);

	/* Start USB stack to authorize VBus monitoring */
	USBD_Connect();

	/* Driver loop */
	while (1) {
		/* Device is not configured */
		if (USBD_GetState() < USBD_STATE_CONFIGURED)
			gUsbConnected = 0;
		else if (!gUsbConnected) {
			gUsbConnected = 1;

			/* Start receiving data on the USB */
			CDCDEEMDriver_Read(gUsbRxBuffer, MAXPACKETSIZE, _UsbDataReceived, 0);

			/* Start receiving data on GMAC */
			GMACD_SetRxCallback(&gGmacd, _EthDataReceived, GMAC_QUE_0);
		}
	}
}
