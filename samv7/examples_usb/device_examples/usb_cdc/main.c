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

/** \cond usb_cdc
 *  \page usb_cdc USB communication device class Example
 *
 *  \section Purpose
 *
 *  The USB communication device class Example will help you to get familiar
 *  with the USB communication device class on SAMV7/E7 Microcontrollers. Also
 *  it can help you to be familiar with the USB Framework that is used for
 *  rapid development of USB-compliant class drivers such as USB communication
 *  device class.
 *
 *  \section Requirements
 *
 *  This package can be used with SAMV71 Xplained Ultra board or SAME70 Xplained
 *  board.
 *  The device uses the USB communication device class (CDC) drivers to take
 *  advantage of the installed PC RS-232 software to talk over the USB.
 *  The example is a bridge between a USART (USART2) from the main microchip
 *  and the USB host CDC interface.
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
 *  -- USB Device CDC Serial Project xxx --
 *  -- SAMxxxxx-xx
 *  -- Compiled: xxx xx xxxx xx:xx:xx --
 *  \endcode
 *
 *  \section References
 *  - usb_cdc/main.c
 *  - usb: USB Framework, USB communication device class driver
 *      - \ref usbd_framework
 *      - \ref usbd_api
 *      - \ref usbd_cdc
 */

/**
 *  \file
 *
 *  This file contains all the specific code for the
 *  usb_cdc example.
 */
/*----------------------------------------------------------------------------
 *         Headers
 *----------------------------------------------------------------------------*/

#include "board.h"

#include "USBD.h"
#include "CDCDSerialDriver.h"

#include <stdbool.h>
#include <stdint.h>

/*----------------------------------------------------------------------------
 *      Definitions
 *----------------------------------------------------------------------------*/

/** Size in bytes of the packet used for reading data from the USB & USART */
#define DATAPACKETSIZE (128)

/** Size in bytes of the buffer used for reading data from the USB & USART */
#define DATABUFFERSIZE (DATAPACKETSIZE+2)

/** Pins used for USART transfer */
#define PINS_USART      PIN_USART2_TXD, PIN_USART2_RXD
/** Register base for USART operation */
#define BASE_USART      USART2
/** USART ID */
#define ID_USART        ID_USART2

#define USART_TIMEOUT           115200
/** test buffer size */
#define TEST_BUFFER_SIZE    (2*1024)
/** write loop count */
#define TEST_COUNT          (30)

/*----------------------------------------------------------------------------
 *      External variables
 *----------------------------------------------------------------------------*/

extern const USBDDriverDescriptors cdcdSerialDriverDescriptors;

/*----------------------------------------------------------------------------
 *      Internal variables
 *----------------------------------------------------------------------------*/

/** Global DMA driver for all transfer */
static sXdmad dmad;

/** DMA channel for RX */
static uint32_t usartDmaRxChannel;
/** DMA channel for TX */
static uint32_t usartDmaTxChannel;

/** USART link list for data RX */
static LinkedListDescriporView1 dmaRxLinkList;

/** List of pins that must be configured for use by the application. */
static const Pin pins[] = {PINS_USART};

/** Double-buffer for storing incoming USART data. */
COMPILER_ALIGNED(32) static uint8_t usartBuffers[2][DATABUFFERSIZE];

/** Current USART buffer index. */
static uint8_t usartCurrentBuffer = 0;

/** Buffer for storing incoming USB data. */
COMPILER_ALIGNED(32) static uint8_t usbBuffer[DATABUFFERSIZE];

/** Serial Port ON/OFF */
static uint8_t isCdcSerialON = 0;

/** CDC Echo back ON/OFF */
static uint8_t isCdcEchoON = 0;

/** USB Tx flag */
static volatile uint8_t txDoneFlag = 0;
/** Test buffer */
COMPILER_ALIGNED(32) static uint8_t testBuffer[TEST_BUFFER_SIZE];
static void _UsartDmaRx(void);
void *memset(void *pBuffer, int value, size_t num);

/*----------------------------------------------------------------------------
 *  Interrupt handlers
 *----------------------------------------------------------------------------*/
/**
 * ISR for xDMA interrupt
 */
void XDMAC_Handler(void)
{
	XDMAD_Handler(&dmad);
}

/**
 * USART interrupt handler
 */
void USART2_Handler(void)
{
	Usart *pUs = BASE_USART;
	uint32_t status;
	uint16_t serialState;
	uint32_t count;

	status  = USART_GetStatus(pUs);
	status &= USART_GetItMask(pUs);

	/* If USB device is not configured, do nothing */
	if (!isCdcSerialON) {
		USART_DisableIt(pUs, 0xFFFFFFFF);
		return;
	}

	if (status & US_CSR_TIMEOUT) {
		/*Clear TIMEOUT Flag and Start Time-out After Next Character Received*/
		USART_AcknowledgeRxTimeOut(BASE_USART, 0);
		/* Flush the DMA FIFO */
		XDMAC_SoftwareFlushReq(dmad.pXdmacs, usartDmaRxChannel);
		/* Transfer the last pack through USB */
		count = dmad.pXdmacs->XDMAC_CHID[usartDmaRxChannel].XDMAC_CUBC;
		SCB_InvalidateDCache_by_Addr((uint32_t *)usartBuffers, DATAPACKETSIZE - count);

		while (CDCDSerialDriver_Write(usartBuffers, DATAPACKETSIZE - count, 0, 0)
			   != USBD_STATUS_SUCCESS);

		/*Reset DMA transfer*/
		XDMAD_StopTransfer(&dmad, usartDmaRxChannel);
		_UsartDmaRx();
	} else {
		/* Errors */
		serialState = CDCDSerialDriver_GetSerialState();

		/* Overrun */
		if ((status & US_CSR_OVRE) != 0) {
			TRACE_WARNING("USART_IrqHandler: Overrun\n\r");
			serialState |= CDCSerialState_OVERRUN;
		}

		/* Framing error */
		if ((status & US_CSR_FRAME) != 0) {
			TRACE_WARNING("USART_IrqHandler: Framing error\n\r");
			serialState |= CDCSerialState_FRAMING;
		}

		CDCDSerialDriver_SetSerialState(serialState);
	}
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
	CDCDSerialDriver_ConfigurationChangedHandler(cfgnum);
}

/**
 * Invoked when a new SETUP request is received from the host. Forwards the
 * request to the Mass Storage device driver handler function.
 * \param request  Pointer to a USBGenericRequest instance.
 */
void USBDCallbacks_RequestReceived(const USBGenericRequest *request)
{
	CDCDSerialDriver_RequestHandler(request);
}

/*----------------------------------------------------------------------------
 *         Internal functions
 *----------------------------------------------------------------------------*/


/**
 *  \brief Send single buffer data through DMA
 */
static void _UsartDmaTx(uint32_t dwDestAddr,
						 void *pBuffer, uint16_t wSize)
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
					   | XDMAC_CC_PERID(XDMAIF_Get_ChannelNumber(ID_USART, XDMAD_TRANSFER_TX));
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
	Usart *pUs = BASE_USART;
	dmaRxLinkList.mbr_ubc = XDMA_UBC_NVIEW_NDV1
							| XDMA_UBC_NDE_FETCH_EN
							| XDMA_UBC_NSEN_UPDATED
							| XDMAC_CUBC_UBLEN(DATAPACKETSIZE);
	dmaRxLinkList.mbr_sa  = (uint32_t)&pUs->US_RHR;
	dmaRxLinkList.mbr_da = (uint32_t)usartBuffers[0];

}

/**
 *  \brief Start waiting USART data
 *  Start DMA, the 1st DMA buffer is free USART buffer assigned.
 */
static void _UsartDmaRx()
{
	sXdmad *pDmad = &dmad;
	sXdmadCfg xdmadUsartRxCfg;
	uint32_t xdmaUsartRxCndc, xdmaInt;
	xdmadUsartRxCfg.mbr_cfg = XDMAC_CC_TYPE_PER_TRAN
							  | XDMAC_CC_MBSIZE_SINGLE
							  | XDMAC_CC_DSYNC_PER2MEM
							  | XDMAC_CC_CSIZE_CHK_1
							  | XDMAC_CC_DWIDTH_BYTE
							  | XDMAC_CC_SIF_AHB_IF1
							  | XDMAC_CC_DIF_AHB_IF1
							  | XDMAC_CC_SAM_FIXED_AM
							  | XDMAC_CC_DAM_INCREMENTED_AM
							  | XDMAC_CC_PERID(XDMAIF_Get_ChannelNumber(ID_USART, XDMAD_TRANSFER_RX));
	xdmadUsartRxCfg.mbr_sa = dmaRxLinkList.mbr_sa;
	xdmadUsartRxCfg.mbr_da = dmaRxLinkList.mbr_da;
	xdmadUsartRxCfg.mbr_ubc = DATAPACKETSIZE;
	xdmadUsartRxCfg.mbr_bc = 0;
	xdmaUsartRxCndc = 0;


	xdmaInt = XDMAC_CIE_BIE;

	XDMAD_ConfigureTransfer(pDmad, usartDmaRxChannel, &xdmadUsartRxCfg,
							 xdmaUsartRxCndc, (uint32_t)&dmaRxLinkList, xdmaInt);
	XDMAD_StartTransfer(pDmad, usartDmaRxChannel);
}

/**
 * DBGU help dump
 */
static void _DebugHelp(void)
{
	printf("\n\r==========================================================\n\r");
	printf("-- ESC to Enable/Disable ECHO on cdc serial --\n\r");
	printf("-- Press 't' to test transfer --\n\r");
	printf("\n\r==========================================================\n\r");
}

/**
 * Callback invoked when data has been sent.
 */
static void _UsbDataSent(void)
{
	txDoneFlag = 1;
}
/*----------------------------------------------------------------------------
 * Callback invoked when data has been received on the USB.
 *----------------------------------------------------------------------------*/
static void _UsbDataReceived(uint32_t unused,
							 uint8_t status,
							 uint32_t received,
							 uint32_t remaining)
{
	unused = unused;
	Usart *pUs = BASE_USART;

	/* Check that data has been received successfully */
	if (status == USBD_STATUS_SUCCESS) {

		SCB_InvalidateDCache_by_Addr((uint32_t *)usbBuffer, received);

		/* Send back CDC data */
		if (isCdcEchoON) {
			while (CDCDSerialDriver_Write(usbBuffer, received, 0, 0)
				   != USBD_STATUS_SUCCESS);
		}

		/* Send data through USART */
		if (isCdcSerialON)
			_UsartDmaTx((uint32_t)&pUs->US_THR, usbBuffer, received);

		/* Check if bytes have been discarded */
		if ((received == DATAPACKETSIZE) && (remaining > 0)) {
			TRACE_WARNING(
				"_UsbDataReceived: %u bytes discarded\n\r",
				(unsigned int)remaining);
		}
	} else {
		TRACE_WARNING("_UsbDataReceived: Transfer error\n\r");
	}
}


/**
 * \brief DMA RX callback function
 */
static void _UsDmaRxCallback(uint32_t channel, void *pArg)
{
	pArg = pArg;

	if (channel != usartDmaRxChannel)
		return;

	/*Clear TIMEOUT Flag and Start Time-out After Next Character Received*/
	USART_AcknowledgeRxTimeOut(BASE_USART, 0);

	SCB_InvalidateDCache_by_Addr((uint32_t *)(usartBuffers[usartCurrentBuffer]),
								 DATABUFFERSIZE);

	/* Send buffer through the USB */
	while (CDCDSerialDriver_Write(usartBuffers[usartCurrentBuffer],
								  DATAPACKETSIZE, 0, 0) != USBD_STATUS_SUCCESS);

	/* Restart read on buffer */
	_UsartDmaRx();

}

/**
 * \brief DMA TX callback function
 */
static void _UsDmaTxCallback(uint32_t channel, void *pArg)
{
	pArg = pArg;

	if (channel != usartDmaTxChannel)
		return;

	/* Restart USB read */
	CDCDSerialDriver_Read(usbBuffer,
						  DATAPACKETSIZE,
						  (TransferCallback) _UsbDataReceived,
						  0);
}

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
					  (XdmadTransferCallback)_UsDmaRxCallback,
					  0);
	/* Set TX callback */
	XDMAD_SetCallback(pDmad, usartDmaTxChannel,
					  (XdmadTransferCallback)_UsDmaTxCallback,
					  0);
	XDMAD_PrepareChannel(pDmad, usartDmaRxChannel);
	XDMAD_PrepareChannel(pDmad, usartDmaTxChannel);
}

/**
 * Configure USART to work @ 115200
 */
static void _ConfigureUsart(void)
{
	PIO_Configure(pins, PIO_LISTSIZE(pins));
	PMC_EnablePeripheral(ID_USART);
	USART_DisableIt(BASE_USART, 0xFFFFFFFF);
	USART_Configure(BASE_USART,
					USART_MODE_ASYNCHRONOUS,
					115200,
					BOARD_MCK);

	USART_SetTransmitterEnabled(BASE_USART, 1);
	USART_SetReceiverEnabled(BASE_USART, 1);
	NVIC_EnableIRQ(USART2_IRQn);
}

/**
 * Test USB CDC Serial
 */
static void _SendText(void)
{
	uint32_t i, testCnt;

	if (!isCdcSerialON) {
		printf("\n\r!! Host serial program not ready!\n\r");
		return;
	}

	printf("\n\r- USB CDC Serial writing ...\n\r");

	/* Test data initialize */
	for (i = 0; i < TEST_BUFFER_SIZE; i ++) testBuffer[i] = (i % 10) + '0';

	printf("- Send 0,1,2 ... to host:\n\r");

	for (testCnt = 0; testCnt < TEST_COUNT; testCnt ++) {
		txDoneFlag = 0;
		CDCDSerialDriver_Write(testBuffer,
							   TEST_BUFFER_SIZE,
							   (TransferCallback) _UsbDataSent, 0);

		while (!txDoneFlag);
	}
}

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

	/* IRQ */
	NVIC_EnableIRQ(USBHS_IRQn);
}


/*! \brief Main function. Execution starts here.
 */
int main(void)
{
	uint8_t isUsbConnected = 0;
	/* Disable watchdog */
	WDT_Disable(WDT);


	SCB_EnableICache();
	SCB_EnableDCache();

	/* Output example information */
	printf("-- USB Device CDC Serial Project %s --\n\r", SOFTPACK_VERSION);
	printf("-- %s\n\r", BOARD_NAME);
	printf("-- Compiled: %s %s With %s--\n\r", __DATE__, __TIME__ ,
			COMPILER_NAME);

	/* Initialize PIO interrupts */
	PIO_InitializeInterrupts(0);

	/* Interrupt priority */
	NVIC_SetPriority(USBHS_IRQn, 2);

	/* Configure DMA driver */
	_ConfigureDma();

	/* Configure USART */
	_ConfigureUsart();
	_UsartDmaRxSetup();

	/* Initialize OTG clocks */
	_ConfigureUotghs();

	/* CDC serial driver initialization */
	CDCDSerialDriver_Initialize(&cdcdSerialDriverDescriptors);

	/* Help information */
	_DebugHelp();

	// Start USB stack to authorize VBus monitoring
	USBD_Connect();

	/* Driver loop */
	while (1) {
		/* Device is not configured */
		if (USBD_GetState() < USBD_STATE_CONFIGURED) {
			if (isUsbConnected) {
				isUsbConnected = 0;
				isCdcSerialON  = 0;
			}
		} else if (isUsbConnected == 0)
			isUsbConnected = 1;

		/* Serial port ON/OFF */
		if (CDCDSerialDriver_GetControlLineState() & CDCControlLineState_DTR) {
			if (!isCdcSerialON) {
				isCdcSerialON = 1;

				/* Start receiving data on the USART */
				_UsartDmaRx();
				USART_EnableIt(BASE_USART, US_CSR_FRAME | US_CSR_OVRE | US_IER_TIMEOUT);
				USART_EnableRecvTimeOut(BASE_USART, USART_TIMEOUT);
				/* Start receiving data on the USB */
				CDCDSerialDriver_Read(usbBuffer,
									  DATAPACKETSIZE,
									  (TransferCallback) _UsbDataReceived,
									  0);
			}
		} else if (isCdcSerialON)
			isCdcSerialON = 0;

		if (DBG_IsRxReady()) {
			uint8_t key = DBG_GetChar();

			/* ESC: CDC Echo ON/OFF */
			if (key == 27) {
				printf("** CDC Echo %s\n\r",
					   isCdcEchoON ? "OFF" : "ON");
				isCdcEchoON = !isCdcEchoON;
			}
			/* 't': Test CDC writing  */
			else if (key == 't')
				_SendText();
			else {
				printf("Alive\n\r");

				while (CDCDSerialDriver_Write((char *)"Alive\n\r", 8, 0, 0)
					   != USBD_STATUS_SUCCESS);

				_DebugHelp();
			}
		}
	}
}
