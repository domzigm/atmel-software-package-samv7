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

/** \cond usb_hid_keyboard
 * \page usb_hid_keyboard USB HID Keyboard Example
 *
 * \section Purpose
 *
 * The USB HID Keyboard Example will help you to get familiar with the
 * USB Device Port(UDP) and PIO interface on SAMV7/E7 Microcontrollers. Also
 * it can help you to be familiar with the USB Framework that is used for
 * rapid development of USB-compliant class drivers such as USB human
 * Interface Device class (HID).
 *
 *  \section Requirements
 *
 *  This package can be used with SAMV71 Xplained Ultra board or SAME70 Xplained board.
 *
 * \section Description
 *
 * The demo simulates a simple keyboard with a caps lock and 'a' on it.
 *
 * When a board running this program connected to a host (PC for
 * example), with USB cable, the board appears as a HID Keyboard for
 * the host. Then you can use the push buttons on the board to input
 * letter to the host. E.g, to open a editor and input a letter 'a'.
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
 *     -- USB Device HID Keyboard Project xxx --
 *     -- SAMxxxxx-xx
 *     -- Compiled: xxx xx xxxx xx:xx:xx --
 *     \endcode
 * -# When connecting USB cable to windows, the
 *    new "HID Keyboard Device" appears in the
 *    hardware %device list.
 * -# Once the device is connected and configured, pressing any of the board
 *    buttons should send characters to the host PC. Pressing num. lock should
 *    also make the third LED toggle its state (on/off).
 *
 * \section References
 * - usb_hid_keyboard/main.c
 * - pio: PIO interface driver
 *    - pio.h
 *    - pio_it.h
 * - usb: USB Framework, USB HID driver and UDP interface driver
 *    - \ref usbd_framework
 *      - \ref usbd_api
 *    - \ref usbd_hid_kbd_drv
 */

/**
 * \file
 *
 * This file contains all the specific code for the
 * usb_hid_keyboard
 *
 * \section Contents
 *
 * The code can be roughly broken down as follows:
 *    - Configuration functions
 *       - VBus_Configure
 *       - ConfigurePit
 *       - ConfigureWakeUp
 *       - PIO & Timer configurations in start of main
 *    - Interrupt handlers
 *       - ISR_Vbus
 *       - ISR_Pit
 *       - WakeUpHandler
 *    - Callback functions
 *       - HIDDKeyboardCallbacks_LedsChanged
 *    - The main function, which implements the program behavior
 *
 */

/*-----------------------------------------------------------------------------
 *         Headers
 *-----------------------------------------------------------------------------*/

#include "board.h"

#include "USBD.h"
#include "HIDDKeyboardDriver.h"
#include "USBD_LEDs.h"

#include <string.h>
#include <stdbool.h>
#include <stdint.h>


/*-----------------------------------------------------------------------------
 *         Definitions
 *-----------------------------------------------------------------------------*/
#define NO_PUSHBUTTON

/** Number of keys used in the example. */
#define NUM_KEYS                    2

/** Number of non-modifiers keys. */
#define NUM_NORMAL_KEYS             1

/** Number of modifier keys. */
#define NUM_MODIFIER_KEYS           (NUM_KEYS - NUM_NORMAL_KEYS)

/** NumLock LED index. */
#define LED_NUMLOCK                 USBD_LEDOTHER

/*---------------------------------------------------------------------------
 *         External variables
 *---------------------------------------------------------------------------*/

/** Descriptor list for HID keyboard device */
extern USBDDriverDescriptors hiddKeyboardDriverDescriptors;

/*---------------------------------------------------------------------------
 *         Internal variables
 *---------------------------------------------------------------------------*/

#ifdef NO_PUSHBUTTON
#else
/** List of pinsPushButtons to configure for the application. */
static Pin pinsPushButtons[] = {PINS_PUSHBUTTONS};
#endif

/** Array of key codes produced by each button. */
static uint8_t keyCodes[NUM_KEYS] = {
	HIDKeypad_A,
	HIDKeypad_NUMLOCK,
};

/** Current status (pressed or not) for each key. */
static uint8_t keyStatus[NUM_KEYS];

/*---------------------------------------------------------------------------
 *         Callbacks re-implementation
 *---------------------------------------------------------------------------*/

/**
 *  Invoked whenever a SETUP request is received from the host. Forwards the
 *  request to the standard handler.
 */
void USBDCallbacks_RequestReceived(const USBGenericRequest *request)
{
	HIDDKeyboardDriver_RequestHandler(request);
}

/**
 * Invoked when the configuration of the device changes. Start reading
 * output reports.
 * \param cfgnum New configuration number.
 */
void USBDDriverCallbacks_ConfigurationChanged(uint8_t cfgnum)
{
	HIDDKeyboardDriver_ConfigurationChangedHandler(cfgnum);
}

/**
 * Invoked when the status of the keyboard LEDs changes. Turns the num. lock
 * LED on or off.
 * \param numLockStatus  Indicates the current status of the num. lock key.
 * \param capsLockStatus  Indicates the current status of the caps lock key.
 * \param scrollLockStatus  Indicates the current status of the scroll lock key.
 */
void HIDDKeyboardCallbacks_LedsChanged(
	uint8_t numLockStatus,
	uint8_t capsLockStatus,
	uint8_t scrollLockStatus)
{
	printf("%c %c %c\n\r",
		   numLockStatus ? 'N' : '_',
		   capsLockStatus ? 'C' : '_',
		   scrollLockStatus ? 'S' : '_'
		);

	/* Num. lock */
	if (numLockStatus)
		LED_Set(LED_NUMLOCK);
	else
		LED_Clear(LED_NUMLOCK);
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


/*---------------------------------------------------------------------------
 *         Exported function
 *---------------------------------------------------------------------------*/

/**
 * Initializes the system and then monitors buttons, sending the
 * corresponding character when one is pressed.
 *  \callgraph
 */
int main(void)
{
	uint32_t i;

	/* Disable watchdog */
	WDT_Disable(WDT);

	printf("-- USB Device HID Keyboard Project %s --\n\r", SOFTPACK_VERSION);
	printf("-- %s\n\r", BOARD_NAME);
	printf("-- Compiled: %s %s With %s--\n\r", __DATE__, __TIME__ ,
			COMPILER_NAME);

	SCB_EnableICache();
	SCB_EnableDCache();

	/* If they are present, configure Vbus & Wake-up pins */
	PIO_InitializeInterrupts(0);

	_ConfigureUotghs();

#ifdef NO_PUSHBUTTON
	printf("-- : DBG key 1 2 used as buttons\n\r");
	printf("-- : 1st press to push, 2nd press to release\n\r");
#else
	/* Initialize key statuses and configure push buttons */
	PIO_Configure(pinsPushButtons, PIO_LISTSIZE(pinsPushButtons));
#endif
	memset(keyStatus, 1, NUM_KEYS);

	/* Configure LEDs */
	LED_Configure(LED_NUMLOCK);

	/* HID driver initialization */
	HIDDKeyboardDriver_Initialize(&hiddKeyboardDriverDescriptors);

	// Start USB stack to authorize VBus monitoring
	USBD_Connect();


	/* Infinite loop */
	while (1) {
		uint8_t pressedKeys[NUM_KEYS];
		uint8_t pressedKeysSize = 0;
		uint8_t releasedKeys[NUM_KEYS];
		uint8_t releasedKeysSize = 0;

		if (USBD_GetState() < USBD_STATE_CONFIGURED)
			continue;

		/* Monitor buttons */
#ifdef NO_PUSHBUTTON

		if (DBG_IsRxReady()) {
			uint8_t key = DBG_GetChar();

			switch (key) {
			case '1': case '2':
				i = key - '1';

				if (keyStatus[i]) {
					/* Key press simulation */
					TRACE_INFO("Key %u pressed\n\r", (unsigned int)i);
					keyStatus[i] = 0;
					pressedKeys[pressedKeysSize] = keyCodes[i];
					pressedKeysSize ++;
					HIDDKeyboard_RemoteWakeUp();
				} else {
					/* Key release simulation */
					TRACE_INFO("Key %u released\n\r", (unsigned int)i);
					keyStatus[i] = 1;
					releasedKeys[releasedKeysSize] = keyCodes[i];
					releasedKeysSize++;
				}

				break;

			default: DBG_PutChar(key);
			}
		}

#else

		for (i = 0; i < PIO_LISTSIZE(pinsPushButtons); i++) {
			/* Check if button state has changed */
			uint8_t isButtonPressed = PIO_Get(&(pinsPushButtons[i]));

			if (isButtonPressed != keyStatus[i]) {
				/* Update button state */
				if (!isButtonPressed) {
					/* Key has been pressed */
					printf("BP %u pressed\n\r", (unsigned int)i);
					keyStatus[i] = 0;
					pressedKeys[pressedKeysSize] = keyCodes[i];
					pressedKeysSize++;
					HIDDKeyboardDriver_RemoteWakeUp();
				} else {
					/* Key has been released */
					printf("BP %u released\n\r", (unsigned int)i);
					keyStatus[i] = 1;
					releasedKeys[releasedKeysSize] = keyCodes[i];
					releasedKeysSize++;
				}
			}
		}

#endif

		/* Update key status in the HID driver if necessary */
		if ((pressedKeysSize != 0) || (releasedKeysSize != 0)) {
			uint8_t status;

			do {
				status = HIDDKeyboardDriver_ChangeKeys(pressedKeys,
													   pressedKeysSize,
													   releasedKeys,
													   releasedKeysSize);
			} while (status != USBD_STATUS_SUCCESS);
		}
	}
}
/** \endcond */
