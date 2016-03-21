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

/** \cond usb_audio_looprec
 *  \page usb_audio_looprec USB Audio Loopback-Recorder Example
 *
 *  \section Purpose
 *
 *  The USB Audio Loopback-Recorder Example will help you to get
 *  familiar with the USB Device Port(UDP) and DACC on SAMV7/E7 micro-controllers.
 *  Also it can help you to be familiar with the USB Framework that is used for
 *  rapid development of USB-compliant class drivers such as USB Audio Device
 *  class.
 *
 *  \section Requirements
 *
 *  This package can be used with SAMV71 Xplained Ultra board or SAME70 Xplained board
 *  those have both UDP.
 *
 *  \section Description
 *
 *  The demo simulates an USB Desktop Speaker with Microphone which actually
 *  does not "speak out" but loop back the sound as microphone input.
 *
 *  When an Xplained board running this program connected to a host (PC for
 *  example), with USB cable, the Xplained board appears as a desktop speaker
 *  for the host. Then the host can play sound through host software. The audio
 *  stream from the host is then sent to the Xplained board. At the same time,
 *  the audio stream received is also sent back to host from Xplained board for
 *  recording.
 *
 *  \section Usage
 *
 *  -# Build the program and download it inside the board.
 *     Please refer to the Getting Started with SAM V71/E70 Microcontrollers.pdf
 *  -# On the computer, open and configure a terminal application
 *     (e.g. HyperTerminal on Microsoft Windows) with these settings:
 *    - 115200 bauds
 *    - 8 bits of data
 *    - No parity
 *    - 1 stop bit
 *    - No flow control
 *  -# Start the application.
 *  -# In the terminal window, the following text should appear:
 *  \code
 *  -- USB Device Audio LoopREC Example xxx --
 *  -- SAMxxxxx-xx
 *  -- Compiled: xxx xx xxxx xx:xx:xx --
 *  \endcode
 *  -# When connecting USB cable to windows, the host reports a new USB device
 * attachment (if it's the first time you connect an audio speaker demo board to
 * your host).
 * You can find new "USB Composite Device" and "USB Audio Device" appear in the
 * hardware device list.
 *  -# You can play sound in host side through the USB Audio Device.
 *     When playing sound, you can also record through the USB Audio Device on
 *     the host.
 *
 *  \section References
 *  - usb_audio_looprec/main.c
 *  - ssc: SSC interface driver
 *  - usb: USB Framework, Audio Device Class driver and UDP interface driver
 *      - \ref usbd_framework
 *         - \ref usbd_api
 *      - \ref usbd_audio_rec_drv
 *
 */

/**
 *  \file
 *
 *  This file contains all the specific code for the
 *  usb_audio_looprec example.
 */

/*----------------------------------------------------------------------------
 *         Headers
 *----------------------------------------------------------------------------*/

#include "board.h"

#include "USBD_LEDs.h"
#include "USBD_Config.h"

#include "AUDDSpeakerPhoneDriver.h"

#include <stdio.h>
#include <stdbool.h>
#include <stdint.h>

/*----------------------------------------------------------------------------
 *         Definitions
 *----------------------------------------------------------------------------*/

/**  Number of available audio buffers. */
#define BUFFER_NUMBER       8
/**  Size of one buffer in bytes. */
#define BUFFER_SIZE         (AUDDevice_BYTESPERFRAME \
							 + AUDDevice_BYTESPERSUBFRAME)

/*----------------------------------------------------------------------------
 *         External variables
 *----------------------------------------------------------------------------*/

/** Descriptor list for USB Audio SpeakerPhone Driver */
extern const USBDDriverDescriptors auddSpeakerPhoneDriverDescriptors;

/*----------------------------------------------------------------------------
 *         Internal variables
 *----------------------------------------------------------------------------*/

/**  Data buffers for receiving audio frames from the USB host. */
COMPILER_ALIGNED(32) static uint8_t buffers[BUFFER_NUMBER][BUFFER_SIZE];
/**  Number of samples stored in each data buffer. */
//static uint32_t bufferSizes[BUFFER_NUMBER];
/**  Next buffer in which USB data can be stored. */
static uint32_t inBufferIndex = 0;
/**  Number of buffers that can be sent to the DAC. */
static volatile uint32_t numBuffersToSend = 0;

/**  Current state of the playback stream interface. */
static volatile uint8_t isPlyActive = 0;
/**  Current state of the record stream interface. */
static volatile uint8_t isRecActive = 0;


/*----------------------------------------------------------------------------
 *         Internal functions
 *----------------------------------------------------------------------------*/

/**
 *  Invoked when a frame has been received.
 */
static void FrameReceived(uint32_t unused,
						  uint8_t status,
						  uint32_t transferred,
						  uint32_t remaining)
{
	unused = unused; /* dummy */
	transferred = transferred;
	remaining = remaining; /* dummy */

	if (status == USBD_STATUS_SUCCESS) {
		/* Loopback! add this buffer to write list */
		if (!isRecActive) {}
		else {
			AUDDSpeakerPhoneDriver_Write(buffers[inBufferIndex],
										 AUDDevice_BYTESPERFRAME);
		}

		/* Update input status data */
		//bufferSizes[inBufferIndex] = transferred / AUDDevice_BYTESPERSAMPLE;
		inBufferIndex = (inBufferIndex + 1) % BUFFER_NUMBER;
		numBuffersToSend++;

	} else if (status == USBD_STATUS_ABORTED) {
		/* Error , ABORT, add NULL buffer */
		//bufferSizes[inBufferIndex] = 0;
		inBufferIndex = (inBufferIndex + 1) % BUFFER_NUMBER;
		numBuffersToSend++;
	} else {
		/* Packet is discarded */
	}

	/* Receive next packet */
	AUDDSpeakerPhoneDriver_Read(buffers[inBufferIndex],
								AUDDevice_BYTESPERFRAME,
								(TransferCallback) FrameReceived,
								0); // No optional argument
}

/*----------------------------------------------------------------------------
 *         Callbacks re-implementation
 *----------------------------------------------------------------------------*/
/**
 * Invoked when the configuration of the device changes. Parse used endpoints.
 * \param cfgnum New configuration number.
 */
void USBDDriverCallbacks_ConfigurationChanged(unsigned char cfgnum)
{
	AUDDSpeakerPhoneDriver_ConfigurationChangeHandler(cfgnum);
}

/**
 * Invoked whenever the active setting of an interface is changed by the
 * host. Reset streaming interface.
 * \param interface Interface number.
 * \param setting Newly active setting.
 */
void USBDDriverCallbacks_InterfaceSettingChanged(unsigned char interface,
		unsigned char setting)
{
	AUDDSpeakerPhoneDriver_InterfaceSettingChangedHandler(interface, setting);
}

/**
 *  Invoked whenever a SETUP request is received from the host. Forwards the
 *  request to the standard handler.
 */
void USBDCallbacks_RequestReceived(const USBGenericRequest *request)
{
	AUDDSpeakerPhoneDriver_RequestHandler(request);
}

/**
 *  Invoked when an audio channel get muted or unmuted. Mutes/unmutes the
 *  channel at the DAC level.
 *  \param mic      Microphone/Speaker stream changed.
 *  \param channel  Channel number that changed.
 *  \param muted    Indicates the new mute status of the channel.
 */
void AUDDSpeakerPhoneDriver_MuteChanged(uint8_t mic,
										uint8_t channel,
										uint8_t muted)
{
	/* Speaker Master channel */
	if (!mic && channel == AUDDSpeakerPhoneDriver_MASTERCHANNEL) {
      if (muted){
			TRACE_WARNING("MuteMaster ");
      } else {
			TRACE_INFO("UnmuteMaster ");
      }
      }
}

/**
 *  Invoked when an audio streaming interface setting changed.
 *  Audio stream is automatically reset.
 *  Actually control streaming rate.
 *  \param mic         Microphone/Speaker stream changed.
 *  \param newSetting  New stream (interface) setting.
 */
void AUDDSpeakerPhoneDriver_StreamSettingChanged(uint8_t mic,
		uint8_t newSetting)
{
	/* Speaker stream */
	if (!mic)
		isPlyActive = (newSetting > 0);
	else
		isRecActive = (newSetting > 0);
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

/*----------------------------------------------------------------------------
 *         Exported functions
 *----------------------------------------------------------------------------*/

/**
 *  \brief usb_audio_looprec Application entry point.
 *
 *  Starts the driver and waits for an audio input stream to forward to the DAC.
 */
int main(void)
{
	volatile uint8_t plyOn = 0, recOn = 0;

	/* Disable watchdog */
	WDT_Disable(WDT);

	SCB_EnableICache();
	SCB_EnableDCache();

	printf("-- USB Device Audio LoopREC Example %s --\n\r", SOFTPACK_VERSION);
	printf("-- %s\n\r", BOARD_NAME);
	printf("-- Compiled: %s %s With %s--\n\r", __DATE__, __TIME__ ,
			COMPILER_NAME);

	/* If they are present, configure Vbus & Wake-up pins */
	PIO_InitializeInterrupts(0);

	/* Interrupt priority */
	NVIC_SetPriority(USBHS_IRQn, 2);

	/* Audio STREAM LED */
	LED_Configure(USBD_LEDOTHER);

	/* USB audio driver initialization */
	AUDDSpeakerPhoneDriver_Initialize(&auddSpeakerPhoneDriverDescriptors);

	/* Start USB stack to authorize VBus monitoring */
	VBus_Configure();

	/* Infinite loop */
	while (1) {
		if (USBD_GetState() < USBD_STATE_CONFIGURED) {
			continue;
		}

		if (plyOn) {
			if (isPlyActive == 0) {
				printf("plyE ");
				plyOn = 0;
			}
		} else if (isPlyActive) {
			/* Try to Start Reading the incoming audio stream */
			AUDDSpeakerPhoneDriver_Read(buffers[inBufferIndex],
										AUDDevice_BYTESPERFRAME,
										(TransferCallback) FrameReceived,
										0); // No optional argument
			printf("plyS ");
			plyOn = 1;
		}

		if (recOn) {
			if (isRecActive == 0) {
				printf("recE ");
				recOn = 0;
			}
		} else if (isRecActive) {
			printf("recS ");
			recOn = 1;
		}
	}
}
/** \endcond */
