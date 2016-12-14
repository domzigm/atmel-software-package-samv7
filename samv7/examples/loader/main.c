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
 *  \page loader Loader Example
 *
 *  \section Purpose
 *
 *  The Loader example shows how to detect a specified condition and then execute
 *  the embedded applications.
 *
 *  \section Requirements
 *
 *  This package can be used with SAMV71 Xplained Ultra board or SAME70 Xplained board.
 *
 *
 *  \section Description
 *
 *  In the demonstration two apps are used, one is "fft_demo", and another is
 *  "usb_video". To run the demonstration properly the apps should be flashed
 *  into the embedded flash in advance.
 *
 *  The demonstration program detects whether or not an Image sensor is mounted
 *  on the board then to execute the UVC (usb_video) example or the FFT example.
 *
 *  For UVC example an image sensor (OV2643/OV5640/OV7740/OV9740) should be used.
 *
 *  \section Usage
 *
 *  -# Build the program and download it inside the board.
 *  Please refer to the Getting Started with SAM V71/E70 Microcontrollers.pdf
 *  -# On the computer, open and configure a terminal application
 *     (e.g. HyperTerminal on Microsoft Windows) with these settings:
 *    - 115200 baud rate
 *    - 8 bits of data
 *    - No parity
 *    - 1 stop bit
 *    - No flow control
 *  -# Start the application.
 *  -# The following text should appear (values depend on the board and chip used):
 *     \code
 *      -- Loader Example xxx --
 *      -- xxxxxx-xx
 *      -- Compiled: xxx xx xxxx xx:xx:xx --
 *     \endcode
 *  -# Execution of the UVC or FFT example.
 *
 *  \section References
 *  - loader/main.c
 *  - loader/loader.c
 *  - loader/loader.h
 *  - trace.h
 */

/** \file
 *
 *  This file contains all the specific code for the loader example.
 *
 */

/*----------------------------------------------------------------------------
 *        Headers
 *----------------------------------------------------------------------------*/

#include "board.h"
#include "loader.h"

/*----------------------------------------------------------------------------
 *        Local definitions
 *----------------------------------------------------------------------------*/
#define LOCATION_USB_VIDEO  0x00420000
#define LOCATION_FFT_DEMO   0x00440000


/*----------------------------------------------------------------------------
 *        External functions
 *----------------------------------------------------------------------------*/
uint32_t UVC_preCondition(void);

/*----------------------------------------------------------------------------
 *        Local variables
 *----------------------------------------------------------------------------*/
static AppConfig appConfig[] = {
	{
		UVC_preCondition,
		LOCATION_USB_VIDEO,
		(char *)"USB video: UVC example with OV7740 or OV9740 modules.\n\r"
	},
	{
		NULL,
		LOCATION_FFT_DEMO,
		(char *)"FFT demo: The example demonstrates complex FFT of 22.05K signals."
	}
};

/*----------------------------------------------------------------------------
 *        Exported functions
 *----------------------------------------------------------------------------*/
/**
 *  \brief getting-started Application entry point.
 *
 *  \return Unused (ANSI-C compatibility).
 */
extern int main( void )
{
	/* Output example information */
	printf("\n\r-- Loader Example %s --\n\r", SOFTPACK_VERSION);
	printf("-- %s\n\r", BOARD_NAME);
	printf("-- Compiled: %s %s With %s--\n\r", __DATE__, __TIME__ , COMPILER_NAME);

	LOADER_AppSwitch(appConfig, sizeof(appConfig)/sizeof(appConfig[0]));

	return 0;
}
