/* ----------------------------------------------------------------------------
*         SAM Software Package License
* ----------------------------------------------------------------------------
* Copyright (c) 2015, Atmel Corporation
*
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
* - Redistributions of source code must retain the above copyright notice,
* this list of conditions and the disclaimer below.
*
* Atmel's name may not be used to endorse or promote products derived from
* this software without specific prior written permission.
*
* DISCLAIMER: THIS SOFTWARE IS PROVIDED BY ATMEL "AS IS" AND ANY EXPRESS OR
* IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
* MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT ARE
* DISCLAIMED. IN NO EVENT SHALL ATMEL BE LIABLE FOR ANY DIRECT, INDIRECT,
* INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
* LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
* OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
* LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
* NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
* EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
* ----------------------------------------------------------------------------
*/

/**
 * \page qspi_flash QSPI with Serialflash Example
 *
 * \section Purpose
 *
 * This example demonstrates how to setup the QSPI in order to initialize, read
 * and write a serial dataflash.
 *
 * \section Requirements
 *
 * This package can be used with SAM V71 Xplained Ultra board.
 *
 * \section Description
 *
 * The demonstration program tests the serial dataflash present on the
 * evaluation kit by erasing and writing each one of its pages. It also gives
 * read/write bandwidth by the test.
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
 * terminal window:
 *    \code
 *    -- QSPI Serialflash Example xxx --
 *    -- SAMxxxxx-xx
 *    -- Compiled: xxx xx xxxx xx:xx:xx --
 *    QSPI drivers initialized
 *    \endcode
 * -# The program will connect to the serial firmware dataflash through the QSPI
 *    and start sending commands to it. It will perform the following:
 *    - Read the JEDEC identifier of the device to auto detect it
 *      The next line should indicate if the serial dataflash has been
 *      correctly identified.
 * \section References
 * - qspi_flash/main.c
 * - qspi.c
 * - s25fl1.c
 */

/**
 * \file
 *
 * This file contains all the specific code for the spi_serialflash example.
 */

/*----------------------------------------------------------------------------
 *        Headers
 *----------------------------------------------------------------------------*/

#include <board.h>

#include <stdio.h>
#include <assert.h>
#include <string.h>
#include "stdlib.h"
/*----------------------------------------------------------------------------
 *        Local definitions
 *----------------------------------------------------------------------------*/

/** Maximum device page size in bytes. */
#define MAXPAGESIZE         256

#define BUFFER_SIZE         512
/*----------------------------------------------------------------------------
 *        Local variables
 *----------------------------------------------------------------------------*/

/** Pins to configure for the application. */
static Pin Qspi_pins[] =  PINS_QSPI;

/** buffer for test QSPI flash */
COMPILER_ALIGNED(32) static uint32_t Buffer[BUFFER_SIZE],
				TestBuffer[BUFFER_SIZE];

static uint8_t PeripheralInit = 0;
/*----------------------------------------------------------------------------
 *         Local functions
 *----------------------------------------------------------------------------*/


static void _fillupbuffer(uint32_t *pBuff, uint32_t size)
{
	uint32_t i;

	// one page
	for (i = 0; i < (size / 4); ) {
		pBuff[i++] = 0x9955030c;
		pBuff[i++] = 0x11232244;
		pBuff[i++] = 0xDEADBEAF;
		pBuff[i++] = 0xBEAFDEAD;
		pBuff[i++] = 0xFF770088;
		pBuff[i++] = 0xBAD69DAD;
	}
}

static uint8_t  _VerifyData(
	uint32_t AddrBegin, uint32_t AddrEnd, uint32_t *TestBuff , uint8_t secure)
{
	static uint8_t TestPassed = 0;
	uint32_t i, j, Fault = 0;
	uint8_t *pBuffRx, *pBuffTx;

	pBuffTx = (uint8_t *)TestBuff;
	TRACE_INFO_WP("Verifying data from  0x%x  to 0x%x \n\r",
		(unsigned int)AddrBegin, (unsigned int)AddrEnd);

	if (secure) {
		TRACE_INFO(" Reading data with scramble ON \n\r");
	} else {
		TRACE_INFO(" Reading data with scramble OFF \n\r");
	}

	for (i = AddrBegin; i < AddrEnd; ) {
		if (PeripheralInit == 2) {
			S25FL1D_Read(Buffer, BUFFER_SIZE, i);
			pBuffRx = (uint8_t *)Buffer;
			pBuffRx = &pBuffRx[6];
		} else if (PeripheralInit == 1) {
			S25FL1D_ReadQuadIO(Buffer, BUFFER_SIZE, i, 0, secure);
			pBuffRx = (uint8_t *)Buffer;
		}


		for (j = 0; j < BUFFER_SIZE; j++) {
			if (pBuffRx[j] != pBuffTx[j]) {
				TestPassed = 1;

				if (Fault == 0)
					printf("\n\rData does not match @ 0x%x ", (unsigned int)i);

				Fault++;
			} else {
				if (Fault > 1)
					printf("upto 0x%x \r\n", (unsigned int)i);

				Fault = 0;
			}
		}

		i += BUFFER_SIZE;
	}

	if (Fault > 1) {
		TRACE_INFO_WP("upto 0x%x \r\n", (unsigned int)i);
		Fault = 0;
	}

	if (TestPassed)
		TRACE_ERROR("Data Does not match \n\r");

	return TestPassed;

}

static void QSPI_UserMenu(void)
{
	printf("\n\r===============Choose the peripheral==================");
	printf("\n\r       1. Run example as QSPI");
	printf("\n\r       2. Run example as SPI master");
	printf("\n\r       3. Display Menu");
	printf("\n\r======================================================\n\r");
}
/*----------------------------------------------------------------------------
 *         Global functions
 *----------------------------------------------------------------------------*/
/**
 * \brief Application entry point for SPI with Serialflash example.
 * Initializes the serial flash and performs several tests on it.
 *
 * \return Unused (ANSI-C compatibility).
 */

int main(void)
{
	uint32_t i;
	uint32_t deviceId;
	uint8_t ucKey;
	uint8_t TestPassed = 0;

	/* Disable watchdog */
	WDT_Disable(WDT);

	/* Output example information */
	printf("-- QSPI Serialflash Example %s --\n\r", SOFTPACK_VERSION);
	printf("-- %s\n\r", BOARD_NAME);
	printf("-- Compiled: %s %s With %s--\n\r", __DATE__, __TIME__ , COMPILER_NAME);
	SCB_EnableICache();
	SCB_EnableDCache();
	TimeTick_Configure();

	PIO_Configure(Qspi_pins, PIO_LISTSIZE(Qspi_pins));
	ENABLE_PERIPHERAL(ID_QSPI);

	QSPI_UserMenu();

	while (1) {
		ucKey = DBG_GetChar();

		switch (ucKey) {
		case '1' :
			S25FL1D_InitFlashInterface(1);
			TRACE_INFO("QSPI drivers initialized ");
			/* enable quad mode */
			S25FL1D_QuadMode(ENABLE);
			PeripheralInit = 1;
			break;

		case '2' :
			S25FL1D_InitFlashInterface(0);
			TRACE_INFO("QSPI Initialized in SPI mode");
			S25FL1D_QuadMode(DISABLE);
			PeripheralInit = 2;
			break;

		case '3' :
			QSPI_UserMenu();
			PeripheralInit = 0;
			break;

		default:
			break;

		}


		if (PeripheralInit) {
			while (1) {
				deviceId = S25FL1D_ReadJedecId();
				printf("ID read: Manufacture ID = 0x%x, Device Type = 0x%x, \
				Capacity = 0x%x\n\r",
					   (uint8_t)(deviceId), (uint8_t)(deviceId >> 8), (uint8_t)(deviceId >> 16));
				break;
			}

			/* erase entire chip  */
			S25FL1D_EraseChip();

			/* fill up the buffer*/
			_fillupbuffer(TestBuffer, BUFFER_SIZE);
			printf("Writing buffer to Flash memory...... \n\r");

			/* write the buffer to flash memory */
			for (i = 0; i < 0x200000; ) {
				S25FL1D_Write(TestBuffer, BUFFER_SIZE, i, 0);
				i += BUFFER_SIZE;
			}

			TestPassed = _VerifyData(0, 0x200000, TestBuffer, 0);

			printf("Erasing a block(64 KB) @ Add 0x10000 \n\r");
			S25FL1D_Erase64KBlock(0x10000);

			memset(TestBuffer, 0xFFFFFF, BUFFER_SIZE);
			SCB_CleanDCache_by_Addr((uint32_t *)TestBuffer, sizeof(TestBuffer));
			TestPassed = _VerifyData(0x10000, (0x10000 + 64 * 1024), TestBuffer, 0);

			if (TestPassed)
				printf(" \n\r**** Test Failed ***** \n\r");
			else
				printf(" \n\r### Test Passed ###\n\r");
		}

		PeripheralInit = 0;
		QSPI_UserMenu();
	}
}
