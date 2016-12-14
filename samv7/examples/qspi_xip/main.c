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
 * \page qspi_xip QSPI XIP Example
 *
 * \section Purpose
 *
 * This example demonstrates how to setup the QSPI Flash in XIP mode to execute
 * code from QSPI flash.
 *
 * \section Requirements
 *
 * This package can be used with SAMV7x evaluation kits.
 *
 * \section Description
 *
 * Ths code writes the coremark benchmark code into flash via SPI and enables
 * quad mode spi to read code and to execute from it.
 *
 * \section Usage
 *
 *  -# Build the program and download it inside the SAM V71 Xplained Ultra board.
 *     Please refer to the Getting Started with SAM V71 Microcontrollers.pdf
 * -# Optionally, on the computer, open and configure a terminal application
 *    (e.g. HyperTerminal on Microsoft Windows) with these settings:
 *   - 115200 bauds
 *   - 8 bits of data
 *   - No parity
 *   - 1 stop bit
 *   - No flow control
 * -# Start the application.
 * -# Upon startup, the application will output the following lines on the
 *  terminal window:
 *    \code
 *    -- QSPI XIP Example xxx --
 *    -- SAMxxxxx-xx
 *    -- Compiled: xxx xx xxxx xx:xx:xx --
 *    QSPI drivers initialized
 *    \endcode
 * \section References
 * - qspi_flash/main.c
 * - qspi.c
 * - s25fl1.c
 */

/**
 * \file
 *
 * This file contains all the specific code for the Qspi_serialflash example.
 */

/*----------------------------------------------------------------------------
 *        Headers
 *----------------------------------------------------------------------------*/

#include <board.h>
#include <stdio.h>
#include <assert.h>
#include <string.h>
#include "stdlib.h"
#include "getting_started_hex.h"

/*----------------------------------------------------------------------------
 *        Local definitions
 *----------------------------------------------------------------------------*/

/** SPI peripheral pins to configure to access the serial flash. */
#define QSPI_PINS        PINS_QSPI

/*----------------------------------------------------------------------------
 *        Local variables
 *----------------------------------------------------------------------------*/
/** Pins to configure for the application. */
static Pin pins[] = QSPI_PINS;

COMPILER_ALIGNED(32) static uint32_t Buffer[4];

/*----------------------------------------------------------------------------
 *         Global functions
 *----------------------------------------------------------------------------*/

/**
 * \brief Application entry point for QSPI_XIP example.
 * Initializes the serial flash and performs XIP.
 *
 * \return Unused (ANSI-C compatibility).
 */

int main(void)
{
	uint8_t MemVerify = 0;
	uint32_t __Start_SP, idx;
	uint32_t (*__Start_New)(void);
	uint8_t *pMemory = (uint8_t *)( QSPIMEM_ADDR );

	/* Disable watchdog */
	WDT_Disable(WDT);

	SCB_EnableICache();
	SCB_EnableDCache();

	 /* Output example information */
	printf("-- QSPI XIP Example %s --\n\r", SOFTPACK_VERSION);
	printf("-- %s\n\r", BOARD_NAME);
	printf("-- Compiled: %s %s With %s--\n\r", __DATE__, __TIME__, COMPILER_NAME);

	/* Configure systick */
	TimeTick_Configure();

	/* Initialize the QSPI and serial flash */
	PIO_Configure(pins, PIO_LISTSIZE(pins));

	/* Enable the clock of QSPI */
	ENABLE_PERIPHERAL(ID_QSPI);

	S25FL1D_InitFlashInterface(1);
	printf("QSPI drivers initialized\n\r");

	/* enable quad mode */
	S25FL1D_QuadMode(ENABLE);

	/* get the code at the beginning of QSPI, run the code directly if it's valid */
	S25FL1D_ReadQuadIO(Buffer, sizeof(Buffer), 0, 1, 0);
	printf("-I- data at the beginning of QSPI: %08x %08x %08x %08x\n\r",
		(unsigned int)Buffer[0], (unsigned int)Buffer[1],
		(unsigned int)Buffer[2], (unsigned int)Buffer[3]);
	if ((IRAM_ADDR <= Buffer[0]) && (IRAM_ADDR + IRAM_SIZE > Buffer[0]) &&
		(QSPIMEM_ADDR < Buffer[1]) && (1 == (Buffer[1]&0x3))) {
		__Start_New = (uint32_t(*)(void)) Buffer[1];
		__Start_SP = Buffer[0];

		printf("-I- a valid application is already in QSPI, run it from QSPI\n\r");
		printf("========================================================= \n\r");

		__set_MSP(__Start_SP);
		__Start_New();
	} else {
		printf("-I- there isn't a valid application in QSPI, program first\n\r");
	}

	if (S25FL1D_Unprotect()) {
		printf("Unprotect QSPI Flash failed!\n\r");
		while (1);
	}

	/* erase entire chip  */
	S25FL1D_EraseChip();

	/* Flash the code to QSPI flash */
	printf("Writing to Memory\n\r");

	S25FL1D_Write((uint32_t *)pBuffercode, sizeof(pBuffercode), 0, 0);

	printf("Example code written 0x%x to Memory\n\r", sizeof(pBuffercode));

	printf("Verifying \n\r");

	/* Update QSPI Region to Full Access and cacheable*/
	MPU_UpdateRegions(MPU_QSPIMEM_REGION, QSPI_START_ADDRESS, \
		MPU_AP_FULL_ACCESS |
		INNER_NORMAL_WB_NWA_TYPE( NON_SHAREABLE ) |
		MPU_CalMPURegionSize(QSPI_END_ADDRESS - QSPI_START_ADDRESS) |
		MPU_REGION_ENABLE);

	/* Start continuous read mode to enter in XIP mode*/
	S25FL1D_ReadQuadIO(Buffer, sizeof(Buffer), 0, 1, 0);

	for (idx = 0; idx < sizeof(pBuffercode); idx++) {
		if (*pMemory == pBuffercode[idx]) {
			pMemory++;
		} else {
			MemVerify = 1;
			printf("Data does not match at 0x%x \n\r", (unsigned)pMemory);
			break;
		}
	}
	if (!MemVerify) {
		printf("Everything is OK \n\r");
		/* set PC and SP */
		__Start_New = (uint32_t(*) (void) ) Buffer[1];
		__Start_SP = Buffer[0];

		printf("\n\r Starting getting started example from QSPI flash \n\r");
		printf("========================================================= \n\r");

		__set_MSP(__Start_SP);

		__Start_New();
	}
	while (1);
}
