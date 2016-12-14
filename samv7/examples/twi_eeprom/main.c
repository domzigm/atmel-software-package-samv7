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
 * \page twi_eeprom TWI EEPROM Example
 *
 * \section Purpose
 *
 * This basic example program demonstrates how to use the TWI peripheral
 * to access an external serial EEPROM chip.
 *
 * \section Requirements
 *
 * This package can be used with SAMV71 Xplained Ultra board or SAME70 Xplained board.
 *
 * \section Description
 *
 * The code can be roughly broken down as follows:
 * <ul>
 * <li>Configure TWI pins.</li>
 * <li>Enable TWI peripheral clock.</li>
 * <li>Configure TWI clock.</li>
 * <li>Initialize TWI as twi master.</li>
 * <li>TWI interrupt handler.</li>
 * <li>The main function, which implements the program behaviour.</li>
 * <ol>
 * <li>Set the first and second page of the EEPROM to all zeroes.</li>
 * <li>Write pattern in page 0. </li>
 * <li>Read back data in page 0 and compare with original pattern (polling).</li>
 * <li>Write pattern in page 1. </li>
 * <li>Read back data in page 1 and compare with original pattern (interrupts).</li>
 * </ol>
 * </ul>
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
 *    \code
 *     -- TWI EEPROM Example xxx --
 *     -- SAMxxxxx-xx
 *     -- Compiled: xxx xx xxxx xx:xx:xx --
 *    \endcode
 * -# The following traces detail operations on the EEPROM, displaying success
 *    or error messages depending on the results of the commands.
 *
 * \section References
 * - twi_eeprom/main.c
 * - twi.c
 * - twid.c
 */

/**
 * \file
 *
 * This file contains all the specific code for the twi eeprom example.
 */

/*----------------------------------------------------------------------------
 *        Headers
 *----------------------------------------------------------------------------*/

#include "board.h"

#include <stdint.h>
#include <stdio.h>
#include <stdarg.h>
#include <string.h>
#include <assert.h>

/*----------------------------------------------------------------------------
 *        Local definitions
 *----------------------------------------------------------------------------*/

/** TWI clock frequency in Hz. */
#define TWCK            400000

/** Slave address of twi_eeprom example.*/
#define AT24MAC_ADDRESS         0x57
#define AT24MAC_SERIAL_NUM_ADD  0x5F

/** Page size of an AT24MAC402 chip (in bytes)*/
#define PAGE_SIZE       16

/** Page numbers of an AT24MAC402 chip */
#define EEPROM_PAGES    16

/** EEPROM Pins definition */
#define BOARD_PINS_TWI_EEPROM PINS_TWI0

/** TWI0 peripheral ID for EEPROM device*/
#define BOARD_ID_TWI_EEPROM   ID_TWIHS0

/** TWI0 base address for EEPROM device */
#define BOARD_BASE_TWI_EEPROM TWIHS0

/*----------------------------------------------------------------------------
 *        Local variables
 *----------------------------------------------------------------------------*/
/** PIO pins to configure. */
static const Pin pins[] = BOARD_PINS_TWI_EEPROM;
static TwihsDma twi_dma;

/** TWI driver instance.*/
static Twid twid;

/** Page buffer.*/
COMPILER_ALIGNED(32) static uint8_t pData[PAGE_SIZE];

static uint8_t CallBackFired = 0;

/*----------------------------------------------------------------------------
 *        Local functions
 *----------------------------------------------------------------------------*/
/**
 * \brief TWI interrupt handler. Forwards the interrupt to the TWI driver handler.
 */
void TWIHS0_Handler(void)
{
	TWID_Handler(&twid);
}

/**
 * DMA interrupt handler.
 */
void XDMAC_Handler(void)
{
	XDMAD_Handler(twi_dma.pTwiDma);
}

/**
 * \brief Dummy callback, to test asynchronous transfer modes.
 */
static void TestCallback(void)
{
	CallBackFired++;
}

static void _fillBuffer(uint8_t *pbuff)
{
	uint32_t i;

	/* Write checkerboard pattern in first page */
	for (i = 0; i < PAGE_SIZE; i++) {
		/* Even*/
		if ((i & 1) == 0)
			pbuff[i] = 0xA5;
		/* Odd */
		else
			pbuff[i] = 0x5A;
	}
}

static void _checkReadBuffer(uint8_t *pBuff)
{
	uint16_t i, NoError;

	NoError = 0;
	for (i = 0; i < PAGE_SIZE; i++) {
		/* Even */
		if (((i & 1) == 0) && (pBuff[i] != 0xA5)) {
			printf( "-E- Data mismatch at offset #%u: expected 0xA5, \
				read 0x%02x\n\r", (unsigned int)i, (unsigned int)pData[i] );
			NoError++;
		}
		/* Odd */
		else {
			if (((i & 1) == 1) && (pBuff[i] != 0x5A)) {
				printf( "-E- Data mismatch at offset #%u: expected 0x5A, read\
					0x%02x\n\r", (unsigned int)i, (unsigned int)pData[i]);
				NoError++;
			}
		}
	}
	printf("-I- %u comparison error(s) found \r", (unsigned int)NoError);
}

/*----------------------------------------------------------------------------
 *        Global functions
 *----------------------------------------------------------------------------*/

/**
 * \brief Application entry point for TWI EEPROM example.
 *
 * \return Unused (ANSI-C compatibility).
 */

extern int main( void )
{
	uint8_t i;
	uint8_t SNo[16];
	Async async;

	/* Disable watchdog */
	WDT_Disable(WDT);

	/* Enable I and D cache */
	SCB_EnableICache();
	SCB_EnableDCache();

	CallBackFired = 0;

	/* Output example information */
	printf("-- TWI EEPROM Example %s --\n\r", SOFTPACK_VERSION);
	printf("-- %s\n\r", BOARD_NAME);
	printf("-- Compiled: %s %s With %s--\n\r", __DATE__, __TIME__, COMPILER_NAME);

	TimeTick_Configure();

	/* Configure TWI pins. */
	PIO_Configure(pins, PIO_LISTSIZE(pins));
	PMC_EnablePeripheral(BOARD_ID_TWI_EEPROM);
	/* Configure TWI */

	twi_dma.pTwid = malloc(sizeof(Twid));
	twi_dma.pTwiDma = malloc(sizeof(sXdmad));

	TWI_ConfigureMaster(BOARD_BASE_TWI_EEPROM, TWCK, BOARD_MCK);
	TWID_Initialize(&twid, BOARD_BASE_TWI_EEPROM);
	TWID_DmaInitialize(&twi_dma, BOARD_BASE_TWI_EEPROM, 0);

	TWID_Read(&twid, AT24MAC_SERIAL_NUM_ADD, 0x80, 1, SNo, PAGE_SIZE, 0);

	/* Erase all page */
	memset(pData, 0, PAGE_SIZE);
	for (i = 0; i < EEPROM_PAGES; i++) {
		printf("-I- Filling page #%d with zeroes ...\r\n", i);
		TWID_Write(&twid, AT24MAC_ADDRESS, i*PAGE_SIZE, 1, pData, PAGE_SIZE, 0);
		/* Wait at least 10 ms */
		Wait(10);
	}
	printf("\r\n");

	_fillBuffer(pData);

	/* Synchronous operation */
	for (i = 0; i < EEPROM_PAGES; i++) {
		printf("\n\r-I- Filling page #%d with checkerboard pattern ...\r", i);
		TWID_DmaWrite(&twi_dma, AT24MAC_ADDRESS, i*PAGE_SIZE, 1, pData, PAGE_SIZE, 0);
		/* Wait at least 10 ms */
		Wait(10);
	}
	printf("\r\n");

	/* Read back data */
	memset(pData, 0, PAGE_SIZE);
	for (i = 0; i < EEPROM_PAGES; i++) {
		printf("-I- Reading page #%d... ", i);
		TWID_DmaRead(&twi_dma, AT24MAC_ADDRESS, i*PAGE_SIZE, 1, pData, PAGE_SIZE, 0);
		/* Wait at least 10 ms */
		Wait(10);
		_checkReadBuffer(pData);
	}
	 printf("\r\n");

	/* Configure TWI interrupts */
	NVIC_ClearPendingIRQ(TWIHS0_IRQn);
	NVIC_EnableIRQ(TWIHS0_IRQn);

	/* Asynchronous operation */
	printf("-I- Read/write on page #0 (IRQ mode)\n\r");

	/* Write checkerboard pattern in first page */
	_fillBuffer(pData);
	memset(&async, 0, sizeof(async));
	async.callback = (void *) TestCallback;

	for (i = 0; i < EEPROM_PAGES; i++) {
		printf("-I- Filling page #%d with checkerboard pattern ...\r", i);
		TWID_Write(&twid, AT24MAC_ADDRESS, i*PAGE_SIZE, 1, pData, PAGE_SIZE, &async);
		while (!ASYNC_IsFinished(&async));
		/* Wait at least 10 ms */
		Wait(10);
	}
	printf("\r\n");


	/* Read back data */
	memset(pData, 0, PAGE_SIZE);
	memset(&async, 0, sizeof(async));
	async.callback = (void *) TestCallback;
	for (i = 0; i < EEPROM_PAGES; i++) {
		printf("-I- Reading page # %d...\r", i);
		TWID_Read(&twid, AT24MAC_ADDRESS, i*PAGE_SIZE, 1, pData, PAGE_SIZE, &async);
		while (!ASYNC_IsFinished(&async));
		/* Wait at least 10 ms */
		Wait(10);
		_checkReadBuffer(pData);
	}
	printf("\r\n Callback Fired %d times", CallBackFired);

	return 0;
}
