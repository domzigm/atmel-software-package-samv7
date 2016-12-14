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
 *  \page sdram SDRAM example
 *
 *  \section Purpose
 *
 *  The SDRAM example will help new users get familiar with Atmel's
 *  SAMV7/E7 family of micro-controllers. This basic application shows Shows how
 *  to initialize and perform read and write a SDRAM memory.
 *
 *  \section Requirements
 *
 *  This package can be used with SAMV71 Xplained Ultra board or SAME70 Xplained board.
 *
 *  \section Description
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
 *  -#In the terminal window, the
 *     following text should appear (values depend on the board and chip used):
 *     \code
 *      -- SDRAM Example xxx --
 *      -- xxxxxx-xx
 *      -- Compiled: xxx xx xxxx xx:xx:xx --
 *     \endcode
 *
 *  \section References
 *  - sdram/main.c
 *  - trace.h
 */

/** \file
 *
 *  This file contains all the specific code for the SDRAM example.
 *
 */

/*----------------------------------------------------------------------------
 *        Headers
 *----------------------------------------------------------------------------*/

#include "board.h"

#include <stdbool.h>
#include <stdio.h>

/*----------------------------------------------------------------------------
 *        Local functions
 *----------------------------------------------------------------------------*/
/**
 * \brief Test SDRAM access
 * \param baseAddr Base address of SDRAM
 * \param size  Size of memory in byte
 * \return 1: OK, 0: Error
 */
static uint32_t _sdramAccess(uint32_t baseAddr, uint32_t size)
{
	uint32_t i;
	uint32_t ret = 1;
	uint32_t *ptr32 = (uint32_t *) baseAddr;
	uint16_t *ptr16 = (uint16_t *) baseAddr;
	uint8_t *ptr8 = (uint8_t *) baseAddr;

	/* Test for 55AA55AA/AA55AA55 pattern */
	printf(" Test for 55AA55AA/AA55AA55 pattern ... \n\r");
	for (i = 0; i < size; i++) {
		if (i & 1) {
			ptr32[i] = 0x55AA55AA;
		} else {
			ptr32[i] = 0xAA55AA55;
		}
		memory_barrier()
	}
	for (i = 0; i < size; i++) {
		if (i & 1) {
			if (ptr32[i] != 0x55AA55AA) {
				printf("-E- Expected:%x, read %x @ %x \n\r",
					0x55AA55AA, (unsigned)(ptr32[i]), (unsigned)(baseAddr + i));
				ret = 0;
			}
		} else {
			if (ptr32[i] != 0xAA55AA55) {
				printf("-E- Expected:%x, read %x @ %x \n\r" ,
						0xAA55AA55 , (unsigned)(ptr32[i]), (unsigned)(baseAddr + i));
				ret = 0;
			}
		}
	}

	if (!ret)
		return ret;
	printf(" Test for BYTE accessing... \n\r");
	/* Test for BYTE accessing */
	for (i = 0; i < size; i++)
		ptr8[i] = (uint8_t)( i & 0xFF);

	for (i = 0; i < size; i++) {
		if (ptr8[i] != (uint8_t)(i & 0xFF)) {
			printf("-E- Expected:%x, read %x @ %x \n\r" ,
				(unsigned)(i & 0xFF), ptr8[i], (unsigned)(baseAddr + i));
			ret = 0;
		}
	}
	if (!ret)
		return ret;

	printf(" Test for WORD accessing... \n\r");
	/* Test for WORD accessing */
	for (i = 0; i < size / 2; i++)
		ptr16[i] = (uint16_t)( i & 0xFFFF);

	for (i = 0; i < size / 2; i++) {
		if (ptr16[i] != (uint16_t)(i & 0xFFFF))  {
			printf("-E- Expected:%x, read %x @ %x \n\r" ,
				(unsigned)(i & 0xFFFF), ptr16[i], (unsigned)(baseAddr + i));
			ret = 0;
		}
	}
	if (!ret)
		return ret;
	printf(" Test for DWORD accessing... \n\r");
	/* Test for DWORD accessing */
	for (i = 0; i < size / 4; i++) {
		ptr32[i] = (uint32_t)( i & 0xFFFFFFFF);
		memory_barrier()
	}
	for (i = 0; i < size / 4; i++) {
		if (ptr32[i] != (uint32_t)(i & 0xFFFFFFFF))  {
			printf("-E- Expected:%x, read %x @ %x \n\r" ,
				(unsigned)(i & 0xFFFFFFFF), (unsigned)(ptr32[i]),(unsigned)(baseAddr + i));
			ret = 0;
		}
	}
	return ret;
}

/*----------------------------------------------------------------------------
 *        Exported functions
 *----------------------------------------------------------------------------*/
/**
 *  \brief getting-started Application entry point.
 *
 *  \return Unused (ANSI-C compatibility).
 */
extern int main(void)
{
	/* Disable watchdog */
	WDT_Disable(WDT);

	/* Enable I and D cache */
	SCB_EnableICache();
	SCB_EnableDCache();

	/* Output example information */
	printf("\n\r-- SDRAM Example %s --\n\r", SOFTPACK_VERSION );
	printf("-- %s\n\r", BOARD_NAME);
	printf("-- Compiled: %s %s With %s--\n\r", __DATE__, __TIME__, COMPILER_NAME);

	TRACE_INFO("Configuring External SDRAM \n\r");
	/* SDRAM timing configuration */
	BOARD_ConfigureSdram();

	/* Full test SDRAM  */
	TRACE_INFO("Starting memory validation of External SDRAM \n\r");

	if (_sdramAccess(SDRAM_CS_ADDR, 0x200000)){
      TRACE_INFO("Test succeeded!");
    } else {
      TRACE_INFO("Test failed!");
    }

	return 1;
}

