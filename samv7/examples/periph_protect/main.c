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
 *  \page periph_protect Peripheral Protect Example
 *
 *  \section Purpose
 *
 *  The Peripheral Protect example demonstrates how to prevent a program from
 *  corrupting a PIO controller behaviour.
 *
 *  \section Requirements
 *
 *  This package can be used with SAMV71 Xplained Ultra board or SAME70 Xplained board.
 *
 *  \section Description
 *
 *  The application shows the protective mechanism of the PIO controller.
 *  The example enables or disables write-protection of PIOB user interfaces.
 *  When the write-protection is enabled, any write attempt to the 
 *  write-protected registers will be detected and the write operation aborts.
 *  So the value of the register won't be modified. Besides, the Write Protect  
 *  Status Register will indicate the offset address of the register.
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
 *  -# In the terminal window, the
 *     following text should appear (values depend on the board and chip used):
 *     \code
 *      -- Peripheral Protect Example xxx --
 *      -- SAMxxxxx-xx
 *      -- Compiled: xxx xx xxxx xx:xx:xx --
 *     \endcode
 *  -# Press one of the keys listed in the menu to perform the corresponding 
 *     action.
 *
 *  \section References
 *  - periph_protect/main.c
 *  - pio.h
 *  - pio.c
 */

/** \file
 *
 *  This file contains all the specific code for the periph_protect.
 */

/*----------------------------------------------------------------------------
 *        Headers
 *----------------------------------------------------------------------------*/

#include "board.h"
 
#include <stdbool.h>
#include <stdio.h>

/*----------------------------------------------------------------------------
 *        Local variables
 *----------------------------------------------------------------------------*/

/** pin instance */
Pin pinwp = PIN_LED_0;

/** Offset table */
uint32_t offset[] = 
{ 
	0x0000, 0x0004, 0x0010, 0x0014, 0x0020, 0x0024, 0x0050, 0x0054, 
	0x0060, 0x0064, 0x0070, 0x0074, 0x00A0, 0x00A4, 0x0090, 0x0094
};

/*----------------------------------------------------------------------------
 *        Local functions
 *----------------------------------------------------------------------------*/

/**
 *  \brief Get the PIO controller address 
 *
 *  Return the address of the PIO controller the Pin structure contains
 */
static uint32_t  *_GetPioFromPin(const Pin *pin)
{
	Pio *pio = pin->pio;
	return (uint32_t *)pio;
}

/**
 *  \brief Display main menu
 *
 *  Display the menu to show how to use this example
 */
static void _DisplayMenu(void)
{
	printf("\n\r\n\r");
	printf("Enter 'l' to enable Write Protect and ");
	printf("enter 'u' to disable Write Protect.\r\n");
	printf("\n\r");
	printf("Select the register to be written by a value(0x12345678).\n\r");
	printf("  0 : PIO Enable Register\t(0x0000)\n\r");
	printf("  1 : PIO Disable Register\t(0x0004)\n\r");
	printf("  2 : PIO Output Enable Register\t(0x0010)\n\r");
	printf("  3 : PIO Output Disable Register\t(0x0014)\n\r");
	printf("  4 : PIO Input Filter Enable Register\t(0x0020)\n\r");
	printf("  5 : PIO Input Filter Disable Register\t(0x0024)\n\r");
	printf("  6 : PIO Multi-driver Enable Register\t(0x0050)\n\r");
	printf("  7 : PIO Multi-driver Disable Register\t(0x0054)\n\r");
	printf("  8 : PIO Pull Up Disable Register\t(0x0060)\n\r");
	printf("  9 : PIO Pull Up Enable Register\t(0x0064)\n\r");
	printf("  a : PIO Peripheral ABCD Select Register 1\t(0x0070)\n\r");
	printf("  b : PIO Peripheral ABCD Select Register 2\t(0x0074)\n\r");
	printf("  c : PIO Output Write Enable Register\t(0x00A0)\n\r");
	printf("  d : PIO Output Write Disable Register\t(0x00A4)\n\r");
	printf("  e : PIO Pad Pull Down Disable Register\t(0x0090)\n\r");
	printf("  f : PIO Pad Pull Down Enable Register\t(0x0094)\n\r");
	printf("\n\r\n\r");
}

/*----------------------------------------------------------------------------
 *        Global functions
 *----------------------------------------------------------------------------*/

/**
 *  \brief periph-protect Application entry point.
 *
 *  \return Unused (ANSI-C compatibility).
 */
extern int main(void)
{
	uint8_t cmd;
	uint32_t wpsr;
	uint32_t *pReg;
	const uint32_t dummy = 0x12345678;

	/* Disable watchdog */
	WDT_Disable(WDT);

	/* Enable I and D cache */
	SCB_EnableICache();
	SCB_EnableDCache();

	/* Output example information */
	printf("-- Peripheral Protect Example %s --\n\r", SOFTPACK_VERSION);
	printf("-- %s\n\r", BOARD_NAME);
	printf("-- Compiled: %s %s With %s--\n\r", __DATE__, __TIME__, COMPILER_NAME);
	_DisplayMenu();

	/* Enable PIO controller peripheral clock */
	PMC_EnablePeripheral(pinwp.id);
	/* Get the address of PIO controller to be write-protected */
	pReg = _GetPioFromPin(&pinwp);

	while (1) {

		cmd = DBG_GetChar();
		switch (cmd) {
		case 'm':
			_DisplayMenu();
			break;

		case 'l':
			PIO_EnableWriteProtect(&pinwp);
			printf("The Write Protect is enabled.\r\n");
			break;

		case 'u':
			PIO_DisableWriteProtect(&pinwp);
			printf("The Write Protect is disabled.\r\n");
			break;

		case '0':
		case '1':
		case '2':
		case '3':
		case '4':
		case '5':
		case '6':
		case '7':
		case '8':
		case '9':
			*(pReg + offset[cmd - '0'] / 4) = dummy;
			break;

		case 'a':
		case 'b':
		case 'c':
		case 'd':
		case 'e':
		case 'f':
			*(pReg + offset[cmd - 'a' + 10] / 4) = dummy;
			break;

		default:
			cmd = 'x';
			break;
		}

		if ((cmd != 'x') && (cmd != 'm') && (cmd != 'l') && (cmd != 'u')) {
			/* A write access has been attempted */
			wpsr = PIO_GetWriteProtectViolationInfo(&pinwp);
			if ((wpsr & PIO_WPMR_WPEN_EN) == PIO_WPMR_WPEN_EN) {
				/* Write protect violation is detected */
				printf("Write protect violation is detected!\r\n");
				printf("The offset of write-protected register is 0x%04x.\r\n", 
					(unsigned int)((wpsr & 0x00FFFF00) >> 8));
			} else {
				/* No write protect violation*/
				printf("No write protect violation is detected.\r\n");
			}
		}
	}
}
