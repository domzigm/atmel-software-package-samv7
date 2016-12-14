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
 *  \page usb_host_hid USB Host HID Example
 *
 *  \section Purpose
 *
 * This example shows how to implement a USB host CDC on SAMV7/E7 Microcontrollers.
 *
 *  \section Requirements
 *  This package can be used with SAMV71 Xplained Ultra board or SAME70 Xplained board
 *
 *  \section Description
 *
 * After loading the application, connect the board to a USB
 * HID mouse.
 *
 *  \section Usage
 *
 *  -# Build the program and download it inside the board. 
 *    Please refer to the Getting Started with SAM V71/E70 Microcontrollers.pdf
 *  -# On the computer, open and configure a terminal application
 *     (e.g. HyperTerminal on Microsoft Windows) with these settings:
 *    - 115200 baud rate
 *    - 8 bits of data
 *    - No parity
 *    - 1 stop bit
 *    - No flow control
 *  -# Connect the HID mouse device to SAM V71 Xplained Ultra board or SAME70 Xplained board
       with the OTG wire.
 *  -# Start the application. In the terminal window, the
 *     following text should appear (values depend on the board and chip used):
 *     \code
 *      -- USB HOST HID Mouse Example xxx --
 *      -- xxxxxx-xx
 *      -- Compiled: xxx xx xxxx xx:xx:xx --
 *     \endcode
 *  -# Human interface on SAM V71 Xplained Ultra board or SAME70 Xplained board.:
 *   - Led 0 is continuously on when a device is connected
 *   - Led 0 blinks when USB host has checked and enabled HID interface
 *   - The blink is slow (1s) with low speed device
 *   - The blink is normal (0.5s) with full speed device
 *   - The blink is fast (0.25s) with high speed device
 *   - Led 1 is on when the mouse move
 *
 *  \section References
 *  - usb_host_hid/main.c
 *  - usb_host_hid/ui.c
 *  - led.c
 *  - USBH.c
 *  - USBH_HAL.c
 *  - uhi_hid_mouse.c
 */

#include "board.h"
#include "conf_usb_host.h"
#include "ui.h"

/*! \brief Main function. Execution starts here.
 */
int main(void)
{

	WDT_Disable(WDT);

	SCB_EnableICache();
	SCB_EnableDCache();

	/* Output example information */
	printf("-- USB HOST HID Mouse Example %s --\n\r", SOFTPACK_VERSION);
	printf("-- %s\n\r", BOARD_NAME);
	printf("-- Compiled: %s %s  With %s--\n\r", __DATE__, __TIME__ ,
			COMPILER_NAME);

	TimeTick_Configure();

	irq_initialize_vectors();
	cpu_irq_enable();

	ui_init();

	// Start USB host stack
	USBH_start();

	// The USB management is entirely managed by interrupts.
	// As a consequence, the user application does only have to play with the power modes.
	while (true);
}
