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

/** \cond usb_hid_mouse
 * \page usb_hid_mouse USB HID Mouse Example
 *
 * \section Purpose
 *
 * The USB HID Mouse Example will help you to get familiar with the
 * USB Device Port(UDP) and PIO interface on SAMV7/E7 Microcontrollers. Also
 * it can help you to be familiar with the USB Framework that is used for
 * rapid development of USB-compliant class drivers such as USB Human
 * Interface Device class (HID).
 *
 * \section Requirements
 *
 * This package can be used with SAMV71 Xplained Ultra board or SAME70 Xplained board
 * that has UDP interface and have push button or joystick on it.
 *
 * \section Description
 *
 * When a board running this program connected to a host (PC for example),
 * with USB cable, the board appears as a HID-compliant mouse for the host.
 * Then you can use the joystick or buttons on the board to control the
 * pointer on the host. * E.g., to move it.
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
 *     -- USB Device HID Mouse Project xxx --
 *     -- SAMxxxxx-xx
 *     -- Compiled: xxx xx xxxx xx:xx:xx --
 *     \endcode
 * -# When connecting USB cable to windows, the
 *    new "HID Mouse Device" appears in the
 *    hardware %device list.
 * -# Once the device is connected and configured, pressing the joystick or
 *    the configured board buttons move the cursor.
 *
 * \section References
 * - usb_hid_mouse/main.c
 * - pio: PIO interface driver
 *    - pio.h
 *    - pio_it.h
 * - usb: USB Framework, USB HID driver and UDP interface driver
 *    - \ref usbd_framework
 *       - \ref usbd_api
 *    - hid-mouse
 *       - \ref usbd_hid_mouse_drv
 *
 */

/**
 * \file
 *
 * This file contains all the specific code for the
 * usb_hid_mouse
 *
 * \section Contents
 *
 * The code can be roughly broken down as follows:
 *    - Configuration functions
 *       - USB configuration
 *       - PIO & Timer configurations in start of main
 *    - Interrupt handlers
 *    - The main function, which implements the program behaviour
 */

/*-----------------------------------------------------------------------------
 *         Headers
 *-----------------------------------------------------------------------------*/

#include "board.h"

#include "USBD.h"
#include "HIDDMouseDriver.h"

#include <string.h>
#include <stdbool.h>
#include <stdint.h>

/*----------------------------------------------------------------------------
 *         Definitions
 *----------------------------------------------------------------------------*/

#define NO_PUSHBUTTON

/** Speed of pointer movement X */
#define SPEED_X             4

/** Speed of pointer movement Y */
#define SPEED_Y             4

/*----------------------------------------------------------------------------
 *         External variables
 *----------------------------------------------------------------------------*/

extern USBDDriverDescriptors hiddMouseDriverDescriptors;

/*----------------------------------------------------------------------------
 *         Internal variables
 *----------------------------------------------------------------------------*/

#ifndef NO_PUSHBUTTON
/** List of pinsJoystick (push button) to configure for the application. */
static Pin pinsJoystick[] = {PINS_PUSHBUTTONS};
#endif

/*----------------------------------------------------------------------------
 *         Remote wake-up support (optional)
 *----------------------------------------------------------------------------*/


/**
 * Monitor buttons of joystick status.
 * \param pBtnStatus Pointer to button status bitmap.
 * \param pDx        Pointer to fill x value.
 * \param pDy        Pointer to fill y value.
 */
static uint8_t _ButtonsMonitor(uint8_t *pBtnStatus,
							   int8_t *pDx,
							   int8_t *pDy)
{
	uint8_t isChanged = 0;
	pBtnStatus = pBtnStatus; /*dummy */
#ifdef NO_PUSHBUTTON

	/* - Movement W S A D */
	if (DBG_IsRxReady()) {
		uint8_t key = DBG_GetChar();
		*pDx = 0; *pDy = 0;

		switch (key) {
		case 'w': case 'W':
			*pDy = -SPEED_Y; isChanged = 1;
			break;

		case 's': case 'S':
			*pDy = +SPEED_Y; isChanged = 1;
			break;

		case 'a': case 'A':
			*pDx = -SPEED_X; isChanged = 1;
			break;

		case 'd': case 'D':
			*pDx = +SPEED_X; isChanged = 1;
			break;

		default:
			break;
		}
	}

#else

	/* - Movement buttons, Joystick or Push buttons */
	/* Left */
	if (PIO_Get(&pinsJoystick[JOYSTICK_LEFT]) == 0) {
		*pDx = -SPEED_X;
		isChanged = 1;
	}
	/* Right */
	else if (PIO_Get(&pinsJoystick[JOYSTICK_RIGHT]) == 0) {
		*pDx = SPEED_X;
		isChanged = 1;
	} else
		*pDx = 0;

#endif
	return isChanged;
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
	HIDDMouseDriver_RequestHandler(request);
}

/**
 * Invoked when the configuration of the device changes. Start reading
 * output reports.
 * \param cfgnum New configuration number.
 */
void USBDDriverCallbacks_ConfigurationChanged(uint8_t cfgnum)
{
	HIDDMouseDriver_ConfigurationChangedHandler(cfgnum);
}

/*----------------------------------------------------------------------------
 *         Internal functions
 *----------------------------------------------------------------------------*/
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

/*----------------------------------------------------------------------------
 *         Exported function
 *----------------------------------------------------------------------------*/

/**
 * usb_hid_mouse application entry.
 *
 * Initializes the system and then monitors buttons, sending the
 * corresponding character when one is pressed.
 */
int main(void)
{
	uint8_t bmButtons = 0;
	int8_t dX = 0, dY = 0;
	uint8_t isChanged;
    
	/* Disable watchdog */
	WDT_Disable(WDT);

	SCB_EnableICache();
	SCB_EnableDCache();

	printf("-- USB Device HID Mouse Project %s --\n\r", SOFTPACK_VERSION);
	printf("-- %s\n\r", BOARD_NAME);
	printf("-- Compiled: %s %s With %s--\n\r", __DATE__, __TIME__ ,
			COMPILER_NAME);

	/* If they are present, configure Vbus & Wake-up pins */
	PIO_InitializeInterrupts(0);

	/* Initialize all USB power (off) */
	_ConfigureUotghs();

#ifdef NO_PUSHBUTTON
	printf("-- Press W S A D to move cursor\n\r");
#else
	/* Initialize key statuses and configure push buttons */
	PIO_Configure(pinsJoystick, PIO_LISTSIZE(pinsJoystick));
#endif

	/* HID driver initialization */
	HIDDMouseDriver_Initialize(&hiddMouseDriverDescriptors);

	// Start USB stack to authorize VBus monitoring
	USBD_Connect();

	/* Infinite loop */
	while (1) {
		if (USBD_GetState() < USBD_STATE_CONFIGURED)
			continue;

		isChanged = _ButtonsMonitor(&bmButtons, &dX, &dY);

		if (isChanged) {
			uint8_t status;

			do {
				status = HIDDMouseDriver_ChangePoints(bmButtons,
													  dX, dY);
			} while (status != USBD_STATUS_SUCCESS);
		}
	}
}
/** \endcond */
