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
 *  \page usb_core USB Device Enumeration Example
 *
 *  \section Purpose
 *
 *  The USB Device Enumeration Example will help you to get familiar with the
 *  USB Device Port(UDP)interface on SAMv7 Microcontrollers.
 *
 *  \section Requirements
 *
 *  This package can be used with SAMV71 Xplained Ultra board or SAME70 Xplained board
 *  that have UDP interface.
 *
 *  \section Description
 *
 *  The demo works as a USB device that can be recognized by host.
 *
 *  When an Xplained board running this program connected to a host (PC for
 *  example), with USB cable, host will notice the attachment of a USB device.
 *  No device driver offered for the device now.
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
 *      -- USB Device Core Project xxx --
 *      -- SAMxxxxx-xx
 *      -- Compiled: xxx xx xxxx xx:xx:xx --
 *      \endcode
 *  -# When connecting USB cable to windows, the LED blinks, and the host
 *     reports a new USB %device attachment.
 *
 *  \section References
 *  - usb_core/main.c
 *  - pio.h
 *  - pio_it.h
 *  - usb: USB Device Framework and UDP interface driver
 *     - \ref usbd_framework
 *       - \ref usbd_api
 *     - \ref usbd_enum
 */

/**
 *  \file
 *
 *  This file contains all the specific code for the
 *  usb_core example.
 *
 *  \section Contents
 *
 *  The code can be roughly broken down as follows:
 *     - Configuration functions
 *        - VBus_Configure
 *        - PIO configurations in start of main
 *     - Interrupt handlers
 *        - ISR_Vbus
 *     - Callback functions
 *        - USBDCallbacks_RequestReceived
 *     - The main function, which implements the program behaviour
 *
 */

/*---------------------------------------------------------------------------
 *         Headers
 *---------------------------------------------------------------------------*/

#include "board.h"

#include <USBDescriptors.h>
#include <USBRequests.h>
#include "USBD.h"
#include <USBDDriver.h>

#include <stdbool.h>
#include <stdint.h>

/*----------------------------------------------------------------------------
 *         Local types
 *----------------------------------------------------------------------------*/

/**  Configuration descriptors with one interface. */
struct SimpleConfigurationDescriptors {

	USBConfigurationDescriptor configuration;
	USBInterfaceDescriptor interface;
};

/*----------------------------------------------------------------------------
 *         Local variables
 *----------------------------------------------------------------------------*/

/**  Device descriptor. */
const USBDeviceDescriptor usbDeviceDescriptor = {

	sizeof(USBDeviceDescriptor),
	USBGenericDescriptor_DEVICE,
	USBDeviceDescriptor_USB2_00,
	0, // No device class code
	0, // No device subclass code
	0, // No device protocol code
	CHIP_USB_ENDPOINTS_MAXPACKETSIZE(0),
	0x03EB, // Atmel vendor ID
	0x0001, // Product ID
	0x0001, // Product release 0.01
	0, // No manufacturer string descriptor
	0, // No product string descriptor
	0, // No serial number string descriptor
	1 // One possible configuration
};

/**  Configuration descriptors. */
const struct SimpleConfigurationDescriptors configurationDescriptors = {

	// Configuration descriptor
	{
		sizeof(USBConfigurationDescriptor),
		USBGenericDescriptor_CONFIGURATION,
		sizeof(struct SimpleConfigurationDescriptors),
		0, // No interface in this configuration
		1, // This is configuration #1
		0, // No string descriptor for this configuration
		BOARD_USB_BMATTRIBUTES,
		USBConfigurationDescriptor_POWER(100)
	},
	// Interface descriptor
	{
		sizeof(USBInterfaceDescriptor),
		USBGenericDescriptor_INTERFACE,
		0, // This is interface #0
		0, // This is setting #0 for interface
		0, // Interface has no endpoint
		0, // No interface class code
		0, // No interface subclass code
		0, // No interface protocol code
		0, // No string descriptor
	}
};

/**  List of descriptors used by the device. */
const USBDDriverDescriptors usbdDriverDescriptors = {

	&usbDeviceDescriptor,
	(const USBConfigurationDescriptor *) &configurationDescriptors,
	0, // No full-speed device qualifier descriptor
	0, // No full-speed other speed configuration descriptor
	0, // No high-speed device descriptor (uses FS one)
	0, // No high-speed configuration descriptor (uses FS one)
	0, // No high-speed device qualifier descriptor
	0, // No high-speed other speed configuration descriptor
	0, // No string descriptor
	0  // No string descriptor
};

/** USB standard device driver. */
USBDDriver usbdDriver;


/**
 *  Invoked whenever a SETUP request is received from the host. Forwards the
 *  request to the standard handler.
 */
void USBDCallbacks_RequestReceived(const USBGenericRequest *request)
{
	USBDDriver_RequestHandler(&usbdDriver, request);
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

/*----------------------------------------------------------------------------
 *         Global functions
 *----------------------------------------------------------------------------*/

/**
 *  Initializes the system, connects the USB and waits indefinitely.
 *
 * \callgraph
 */
int main(void)
{
	/* Disable watchdog */
	WDT_Disable(WDT);

	SCB_EnableICache();
	SCB_EnableDCache();

	/* Output example information */
	printf("-- USB Device Core Project %s --\n\r", SOFTPACK_VERSION);
	printf("-- %s\n\r", BOARD_NAME);
	printf("-- Compiled: %s %s With %s--\n\r", __DATE__, __TIME__ ,
			COMPILER_NAME);

	/* If they are present, configure Vbus & Wake-up pins */
	PIO_InitializeInterrupts(0);

	/* connect if needed */
	/* Interrupt priority */
	NVIC_SetPriority(USBHS_IRQn, 2);
	/* Initialize USB clocks */
	_ConfigureUotghs();

	/* USB initialization, Disable Pull-up */
	TRACE_INFO("USB initialization\n\r");
	USBDDriver_Initialize(&usbdDriver, &usbdDriverDescriptors, 0);
	USBD_Init();

	/* Wait about 10ms so that host detach the device to re-enumerate
	   Device connection */
	TRACE_INFO("Connecting device\n\r");

	// Start USB stack to authorize VBus monitoring
	USBD_Connect();

	while (USBD_GetState() < USBD_STATE_CONFIGURED);

	// Infinite loop
	while (1);
}

