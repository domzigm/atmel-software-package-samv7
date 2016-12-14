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

/** \cond usb_iad_cdc_hid
 * \page usb_iad_cdc_hid USB CDC(Serial)+HID(Keyboard) Example
 *
 * \section Purpose
 *
 * The USB CDCHID Project will help you to get familiar with the
 * USB Device Port(UDP)interface .Also it can help you to be familiar with the USB
 * Framework that is used for rapid development of USB-compliant class
 * drivers such as USB Communication Device class (CDC), and how to combine
 * two USB functions to a single composite device (such as CDC + HID).
 *
 * \section Requirements
 *
 * This package can be used with SAMV71 Xplained Ultra board or SAME70 Xplained board
 * that have UDP interface, depending on the functions included.
 *
 * \section win_drv_update Windows Driver Update
 *
 * The composite device is generally supported by Microsoft windows, but some
 * patches are needed for muti-interface functions such as CDC & Audio.
 *
 * \section Description
 *
 * This demo simulates a USB composite device that has USB Serial RS232
 * Converter and USB HID Keyboard functions.
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
 *     -- USB CDC HID Device Project xxx --
 *     -- SAMxxxxx-xx
 *     -- Compiled: xxx xx xxxx xx:xx:xx --
 *     \endcode
 * -# When connecting USB cable to windows, the LED blinks, and the host
 *    reports a new USB device attachment.
 * -# For the windows driver installation and the test functions, please
 *      refer to "USB CDC serial converter" &
 *      "USB HID Keyboard Project".
 * -# You can use the inf file
 *    libraries\\usb\\device\\composite\\drv\\CompositeCDCSerial.inf
 *    to install the CDC serial  port.
 *
 * \section Reference
 * - usb_iad_cdc_hid/main.c
 * - pio: Pin configurations and peripheral configure.
 * - usb: USB Device Framework, USB CDC driver and UDP interface driver
 *    - \ref usbd_framework
 *        - \ref usbd_api
 *    - \ref usbd_composite "composite"
 *       - \ref usbd_composite_drv
 *    - \ref usbd_hid "hid" \\ \ref usbd_hid_key "hid-keyboard"
 *       - \ref usbd_hid_kbd_drv
 *    - \ref usbd_cdc "cdc-serial"
 *       - \ref usbd_cdc_serial_drv
 * - projects:
 *    - \ref usb_hid_keyboard
 *    - \ref usb_cdc_serial
 *
 */

/**
 * \file
 *
 * This file contains all the specific code for the
 * usb_iad_cdc_hid project
 */

/*----------------------------------------------------------------------------
 *        Headers
 *----------------------------------------------------------------------------*/

#include "board.h"

#include <USBD_Config.h>
#include <USBD_LEDs.h>

#include <CDCHIDDDriver.h>
#include <CDCDSerial.h>
#include <HIDDKeyboard.h>

#include <string.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

/*---------------------------------------------------------------------------
 *      Definitions
 *---------------------------------------------------------------------------*/
#define NO_PUSHBUTTON
/** Number of keys used in the example. */
#define NUM_KEYS                    2

/** Number of non-modifiers keys. */
#define NUM_NORMAL_KEYS             1

/** Number of modifier keys. */
#define NUM_MODIFIER_KEYS           (NUM_KEYS - NUM_NORMAL_KEYS)

/** NumLock LED index. */
#define LED_NUMLOCK                 USBD_LEDOTHER

/** Size in bytes of the packet used for reading data from USB */
#define DATAPACKETSIZE \
	CHIP_USB_ENDPOINTS_MAXPACKETSIZE(CDCDSerialDriverDescriptors_DATAIN)

/** Size in bytes of the buffer used for reading data from the USB & USART */
#define DATABUFFERSIZE (DATAPACKETSIZE+2)

/*----------------------------------------------------------------------------
 *      External variables
 *----------------------------------------------------------------------------*/

extern const USBDDriverDescriptors cdchiddDriverDescriptors;

/*---------------------------------------------------------------------------
 *      Internal variables
 *---------------------------------------------------------------------------*/

/** Buffer for storing incoming USB data. */
COMPILER_ALIGNED(32) static unsigned char usbSerialBuffer0[DATABUFFERSIZE];

/** Serial port opened */
static unsigned char isSerialPortON = 0;

/*- HID */

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

/*-----------------------------------------------------------------------------
 *         Callback re-implementation
 *-----------------------------------------------------------------------------*/



/**
 * Invoked when the configuration of the device changes. Parse used endpoints.
 * \param cfgnum New configuration number.
 */
void USBDDriverCallbacks_ConfigurationChanged(uint8_t cfgnum)
{
	CDCHIDDDriver_ConfigurationChangedHandler(cfgnum);
}

/**
 * Invoked when a new SETUP request is received from the host. Forwards the
 * request to the Mass Storage device driver handler function.
 * \param request  Pointer to a USBGenericRequest instance.
 */
void USBDCallbacks_RequestReceived(const USBGenericRequest *request)
{
	CDCHIDDDriver_RequestHandler(request);
}

/*----------------------------------------------------------------------------
 *         Callbacks
 *----------------------------------------------------------------------------*/

/**
 * Invoked when the CDC ControlLineState is changed
 * \param DTR   New DTR value.
 * \param RTS   New RTS value.
 */
void CDCDSerial_ControlLineStateChanged(uint8_t DTR,
										uint8_t RTS)
{
	isSerialPortON = DTR;
	RTS = RTS; /* dummy */
}

/**
 * Invoked when the status of the keyboard LEDs changes. Turns the num. lock
 * LED on or off.
 * \param numLockStatus Indicates the current status of the num. lock key.
 * \param capsLockStatus Indicates the current status of the caps lock key.
 * \param scrollLockStatus Indicates the current status of the scroll lock key
 */
void HIDDKeyboardCallbacks_LedsChanged(
	uint8_t numLockStatus,
	uint8_t capsLockStatus,
	uint8_t scrollLockStatus)
{
	capsLockStatus = capsLockStatus;
	scrollLockStatus = scrollLockStatus;

	/* Num. lock */
	if (numLockStatus)
		LED_Set(LED_NUMLOCK);
	else
		LED_Clear(LED_NUMLOCK);
}

/*---------------------------------------------------------------------------
 *         Internal functions
 *---------------------------------------------------------------------------*/

/**
 * Monitor keyboard buttons & Update key status in HID driver
 */
static void _HIDDKeyboardProcessKeys(void)
{
	uint32_t i;
	uint8_t pressedKeys[NUM_KEYS];
	uint8_t pressedKeysSize = 0;
	uint8_t releasedKeys[NUM_KEYS];
	uint8_t releasedKeysSize = 0;

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
				TRACE_INFO("Key %u has been pressed\n\r", i);
				keyStatus[i] = 0;
				pressedKeys[pressedKeysSize] = keyCodes[i];
				pressedKeysSize++;
				HIDDKeyboard_RemoteWakeUp();
			} else {
				/* Key has been released */
				TRACE_INFO("Key %u has been released\n\r", i);
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
			status = HIDDKeyboard_ChangeKeys(pressedKeys,
											 pressedKeysSize,
											 releasedKeys,
											 releasedKeysSize);
		} while (status != USBD_STATUS_SUCCESS);
	}
}

/**
 * Configure USBHS settings for USB device
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
 * Initializes drivers and start the USB CDCHID device.
 */
int main(void)
{
	uint8_t usbConnected = 0, serialON = 0;

	/* Disable watchdog */
	WDT_Disable(WDT);

	SCB_EnableICache();
	SCB_EnableDCache();

	printf("-- USB CDC HID Device Project %s --\n\r", SOFTPACK_VERSION);
	printf("-- %s\n\r", BOARD_NAME);
	printf("-- Compiled: %s %s With %s--\n\r", __DATE__, __TIME__ ,
			COMPILER_NAME);

	/* If they are present, configure Vbus & Wake-up pins */
	PIO_InitializeInterrupts(0);

	/* If there is on board power, switch it off */
	_ConfigureUotghs();

	/* ----- HID Function Initialize */
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


	/* USB CDCHID driver initialization */
	CDCHIDDDriver_Initialize(&cdchiddDriverDescriptors);

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
				isSerialPortON = 0;
			}
		} else {
			if (usbConnected == 0) {
				printf("-I- USB Connect\n\r");
				usbConnected = 1;
			}

			if (!serialON && isSerialPortON) {
				printf("-I- SerialPort ON\n\r");
				/* Start receiving data on the USB */
				CDCDSerial_Read(usbSerialBuffer0,
								DATAPACKETSIZE,
								0,
								0);
				serialON = 1;
			} else if (serialON && !isSerialPortON) {
				printf("-I- SeriaoPort OFF\n\r");
				serialON = 0;
			}

			_HIDDKeyboardProcessKeys();
		}
	}
}
/** \endcond */
