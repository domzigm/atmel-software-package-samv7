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

/** \cond usb_hid_transfer
 *  \page usb_hid_transfer USB HID Transfer Example
 *
 *  \section Purpose
 *
 *  The USB HID Transfer Project will help you to get familiar with the
 *  USB Device Port(UDP) and PIO interface on SAMV7/E7 Microcontrollers. Also
 *  it can help you to be familiar with the USB Framework that is used for
 *  rapid development of USB-compliant class drivers such as USB Human
 *  Interface Device class (HID).
 *
 *  \section Requirements
 *
 * This package can be used with SAMV71 Xplained Ultra board or SAME70 Xplained board
 * that have UDP interface, depending on the functions included.
 *
 *  \section Description
 *
 *  The demo simulates a customized HID device that reports customized data
 *  stream, in which informations on LEDs and buttons are packed, to host.
 *
 *  When an Xplained running this program connected to a host (PC for example),
 *  with USB cable, the Xplained appears as a "USB Human Interface Device" for
 *  the host.Then you can use the client application to read/write on it.
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
 *      \code
 *      -- USB Device HID Transfer Project xxx --
 *      -- SAMxxxxx-xx
 *      -- Compiled: xxx xx xxxx xx:xx:xx --
 *      \endcode
 *  -# When connecting USB cable to windows, the LED blinks.
 *     Then new "HID Transfer Device" appears in the
 *     hardware %device list.
 *  -# Then you can use the PC program !hidTest.exe to check the !device
 *     information and run tests.
 *  -# Find the HID Device whose VID is 03EB and PID is 6201, select item type
 *     and item to see its attributes.
 *  -# Type what you want to send in output edit box, use buttons on the right
 *     side to send. You can see data information in debug terminal.
 *  -# You can use the buttons above the input edit box to read data from
 *     !device of monitor the data, then the data and the status of the buttons
 *     on the board is read and the gray buttons is up or down based on the
 *     buttons status on the board.
 *
 *  \section References
 *  - usb_hid_transfer/main.c
 *  - pio: PIO interface driver
 *     - pio.h
 *     - pio_it.h
 *  - usb: USB Framework, USB HID driver and UDP interface driver
 *     - \ref usbd_framework
 *        - \ref usbd_api
 *     - \ref usbd_hid_tran "hid-Transfer"
 *        - \ref usbd_hid_xfr_drv
 */

/**
 *  \file
 *
 *  This file contains all the specific code for the
 *  usb_hid_transfer example
 */

/**
 *         Headers
 */

#include "board.h"

#include <HIDDTransferDriver.h>

#include <stdio.h>
#include <string.h>

/**
 *         Definitions
 */

/** Delay for push-button denounce (ms) */
#define DEBOUNCE_TIME      10
#define NO_PUSHBUTTON
/**
 *         External variables
 */

/** HID Transfer driver descriptors */
extern USBDDriverDescriptors hiddTransferDriverDescriptors;

/**
 *         Internal variables
 */

/** Pins for Buttons */
#ifndef NO_PUSHBUTTON
static Pin pinsButtons[] = { PINS_PUSHBUTTONS };
#endif

#ifndef NO_PUSHBUTTON
/**
 *  Remote wake-up support (optional)
 */

/** Button for Wake-UP the USB device. */
static const Pin pinWakeUp = PIN_PUSHBUTTON_1;

/**
 *  Interrupt service routine for the remote wake-up pin. Starts the debounce
 *  sequence.
 */
static void WakeUpHandler(const Pin *pin)
{
	TRACE_DEBUG("Wake-up handler\n\r");

	/* Check current level on the remote wake-up pin */
	if (!PIO_Get(&pinWakeUp))
		HIDDTransferDriver_RemoteWakeUp();
}

/**
 *  Configures the wake-up pin to generate interrupts.
 */
static void _ConfigureWakeUp(void)
{
	TRACE_INFO("Wake-up configuration\n\r");

	/* Configure PIO */
	PIO_Configure(&pinWakeUp, 1);
	PIO_SetDebounceFilter(&pinWakeUp, DEBOUNCE_TIME);
	PIO_ConfigureIt(&pinWakeUp, WakeUpHandler);
	PIO_EnableIt(&pinWakeUp);
}
#endif

/*
 *         Internal Functions
 */

/**
 *  Display the buffer, 8 byte a line
 *
 *  \param buffer   Pointer to the data location
 *  \param dwLen    Size of the data
 */
static void _ShowBuffer(uint8_t *pucBuffer, uint32_t dwLen)
{
	uint32_t dw;

	for (dw = 0; dw < dwLen; dw++) {
		if ((dw & 0x7) == 0)
			printf("\n\r");

		printf(" %02x", pucBuffer[dw]);
	}

	printf("\n\r");
}

/*----------------------------------------------------------------------------
 *         Callbacks re-implementation
 *----------------------------------------------------------------------------*/

/**
 *  Invoked whenever a SETUP request is received from the host. Forwards the
 *  request to the standard handler.
 */
void USBDCallbacks_RequestReceived(const USBGenericRequest *request)
{
	HIDDTransferDriver_RequestHandler(request);
}

/**
 * Invoked when the configuration of the device changes. Start reading
 * output reports.
 * \param cfgnum New configuration number.
 */
void USBDDriverCallbacks_ConfigurationChanged(uint8_t cfgnum)
{
	HIDDTransferDriver_ConfigurationChangedHandler(cfgnum);
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

/**
 *  Main function
 */

/**
 *  Initializes the system and then monitors buttons, sending the
 *  corresponding character when one is pressed.
 *  \callgraph
 */
int main(void)
{
	uint32_t dwCnt = 0;
	uint32_t dwLen;
	uint8_t  iBuffer[64];
	uint8_t  oBuffer[64] = {0x80};
	uint8_t  bmLEDs = 0;
	uint8_t  update;

	/* Disable watchdog */
	WDT_Disable(WDT);

	SCB_EnableICache();
	SCB_EnableDCache();

	printf("-- USB Device HID Transfer Project %s --\n\r", SOFTPACK_VERSION);
	printf("-- %s\n\r", BOARD_NAME);
	printf("-- Compiled: %s %s With %s--\n\r", __DATE__, __TIME__ ,
			COMPILER_NAME);

	/* If they are present, configure Vbus & Wake-up pins */
	PIO_InitializeInterrupts(0);

	/* If there is on board power, switch it off */
	_ConfigureUotghs();

#ifndef NO_PUSHBUTTON
	/* If there is wakeup pin, configure it */
	_ConfigureWakeUp();
#endif

	/* Configure PINs for LEDs and Buttons */
	LED_Configure(LED_YELLOW0);
	LED_Set(LED_YELLOW0);
#if 2 == LED_NUM
	LED_Configure(LED_YELLOW1);
	LED_Clear(LED_YELLOW1);
#endif
#ifdef NO_PUSHBUTTON
	printf("-- : DBG key 1 2 used as buttons\n\r");
	printf("-- : 1st press to push, 2nd press to release\n\r");
#else
	PIO_Configure(pinsButtons, PIO_LISTSIZE(pinsButtons));
#endif

	/* HID driver initialization */
	HIDDTransferDriver_Initialize(&hiddTransferDriverDescriptors);

	/* connect if needed */
	USBD_Connect();

	/* Infinite loop */
	while (1) {
		if (USBD_GetState() < USBD_STATE_CONFIGURED)
			continue;

		update = 0;
		dwLen = HIDDTransferDriver_Read(iBuffer, 64);

		if (dwLen) {
			printf("Data In(%u):", (unsigned int) dwLen);
			_ShowBuffer(iBuffer, dwLen);

			bmLEDs = iBuffer[0];
			update = 1;
		}

		dwLen = HIDDTransferDriver_ReadReport(iBuffer, 64);

		if (dwLen) {
			printf("Report In(%u):", (unsigned int)dwLen);
			_ShowBuffer(iBuffer, dwLen);

			bmLEDs = iBuffer[0];
			update = 1;
		}

		/* Update the status of LEDs */
		if (update && (0x80 & bmLEDs)) {
			/* LED1 */
			if (bmLEDs & 0x01)
				LED_Set(LED_YELLOW0);
			else
				LED_Clear(LED_YELLOW0);

#if 2 == LED_NUM

			/* LED2 */
			if (bmLEDs & 0x02)
				LED_Set(LED_YELLOW1);
			else
				LED_Clear(LED_YELLOW1);

#endif
		}

		/* Update the status of the buttons */
#ifdef NO_PUSHBUTTON

		if (DBG_IsRxReady()) {
			uint8_t key = DBG_GetChar();

			switch (key) {
			case '1' :
				if (oBuffer[0] & 0x01) oBuffer[0] &= ~0x01u;
				else                   oBuffer[0] |=  0x01u;

				break;

			case '2' :
				if (oBuffer[0] & 0x02) oBuffer[0] &= ~0x02u;
				else                   oBuffer[0] |=  0x02u;

				break;
			}
		}

#else
		oBuffer[0] = 0x80;

		if (PIO_Get(&pinsButtons[PUSHBUTTON_BP1]) == 0)
			oBuffer[0] |= 0x01;

#if 2 == BUTTON_NUM

		if (PIO_Get(&pinsButtons[PUSHBUTTON_BP2]) == 0)
			oBuffer[0] |= 0x02;

#endif
#endif

		sprintf((char *)&oBuffer[5], ":%04x:%05u!",
				 (unsigned int)dwCnt, (unsigned int)dwCnt);
		oBuffer[1] = (uint8_t)(dwCnt);
		oBuffer[2] = (uint8_t)(dwCnt >> 8);
		oBuffer[3] = (uint8_t)(dwCnt >> 16);
		oBuffer[4] = (uint8_t)(dwCnt >> 24);

		if (USBD_STATUS_SUCCESS == HIDDTransferDriver_Write(oBuffer, 64, 0, 0))
			dwCnt ++;
	}
}
/** \endcond */
