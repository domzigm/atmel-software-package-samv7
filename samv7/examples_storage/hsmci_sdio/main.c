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
 * \page hsmci_sdio Basic SDIO Card Example
 *
 * \section Purpose
 *
 *  The Basic SDIO Card Example will help you to get familiar with HSMCI
 *  interface on SAM Microcontrollers. It can also help you to get familiar
 *  with the SDIO operation flow which can be used for fast implementation
 *  of your own SD drivers and other applications related.
 *
 *  \section Requirements
 *
 *  This package can be used with SAMV71 Xplained Ultra board or SAME70 Xplained board.
 *
 *  \section Description
 *
 *  The demonstration program detects SDIO device connected and perform
 *  R/W operation on it.
 *
 *  Open HyperTerminal before running this program, use SAM-BA to download
 *  this program to SRAM , make the program run, the HyperTerminal
 *  will give out the test results.
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
 *  -# Start the application
 *  -# In HyperTerminal, it will show something like
 *      \code
 *      -- Basic HSMCI SDIO Example xxx --
 *      -- SAMxxxxx-xx
 *      -- Compiled: xxx xx xxxx xx:xx:xx --
 *      -I- Please connect a SD card ...
 *      -I- SD card connection detected
 *      -I- Cannot check if SD card is write-protected
 *      -I- SD/MMC card initialization successful
 *      R/W Direct test:
 *      ...
 *      \endcode
 *  -# After card inserted following commands can be used:
 *     - 'f' Change current function number
 *     - 'r' Dump SDIO register value
 *     - 'w' Write to SDIO register
 *
 *  \section References
 *  - hsmci_sdcard/main.c
 *  - hsmci.h
 *  - pio.h
 *
 */

/**
 *  \file
 *
 *  \section Purpose
 *
 *  This file contains all the specific code for the hsmci_sdcard example.
 *
 *  \section Contents
 *  The hsmci_sdio application can be roughly broken down as follows:
 *     - Optional functions
 *        - CardDetectConfigure
 *        - CardIsConnected
 *     - Interrupt handlers
 *        - MCI_IrqHandler
 *     - The main function, which implements the program behaviour
 *        - I/O configuration
 *        - SD card auto-detect (if supported)
 *        - Initialize MCI interface and installing an isr relating to MCI
 *        - Initialize sdcard, get necessary sdcard's parameters
 *        - Test RW_DIRECT and RW_EXTENDED at SDIO CIA area.
 */
/*----------------------------------------------------------------------------
 *         Headers
 *----------------------------------------------------------------------------*/

#include "board.h"
#include "libsdmmc.h"

#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <assert.h>

/*----------------------------------------------------------------------------
 *         Local definitions
 *----------------------------------------------------------------------------*/

/*----------------------------------------------------------------------------
 *         Local variables
 *----------------------------------------------------------------------------*/

/** DMA driver instance */
static sXdmad dmaDrv;

/** MCI driver instance. */
static sMcid mciDrv[BOARD_NUM_MCI];

/** SDCard driver instance. */
COMPILER_ALIGNED(32) static sSdCard sdDrv[BOARD_NUM_MCI];

/** Current selected MCI interface */
static uint8_t bMciID = 0;

/** SD card pins instance. */
static const Pin pinsSd[] = {BOARD_MCI_PINS_SLOTA, BOARD_MCI_PIN_CK};

/** SD card detection pin instance. */
static const Pin pinsCd[] = {BOARD_MCI_PIN_CD};

/** Date buffer */
COMPILER_ALIGNED(32) static uint8_t pBuffer[SDMMC_BLOCK_SIZE];

/** Current function */
static uint8_t curFunc = 0;

/*----------------------------------------------------------------------------
 *         Local macros
 *----------------------------------------------------------------------------*/

/*----------------------------------------------------------------------------
 *         Local functions
 *----------------------------------------------------------------------------*/

/**
 * MCI interrupt handler. Forwards the event to the MCI driver handlers.
 */
void HSMCI_Handler(void)
{
	uint32_t i;

	for (i = 0; i < BOARD_NUM_MCI; i++)
		MCID_Handler(&mciDrv[i]);
}

void XDMAC_Handler(void)
{
	XDMAD_Handler(&dmaDrv);
}

/*----------------------------------------------------------------------------
 *         Optional: SD card detection (connection, protection)
 *----------------------------------------------------------------------------*/

/**
 * Configure for SD detect pin
 */
static void CardDetectConfigure(void)
{
	PIO_Configure(pinsCd, PIO_LISTSIZE(pinsCd));
	/* No protection detect pin */
}

/**
 * Return 1 if card is inserted.
 */
static uint8_t CardIsConnected(uint8_t iMci)
{
	return PIO_Get(&pinsCd[iMci]) ? 0 : 1;
}

#if 0
/**
 * Return 1 if any card is inserted.
 */
static uint8_t AnyCardIsConnected(void)
{
	uint32_t i;

	for (i = 0; i < BOARD_NUM_MCI; i ++) {
		if (CardIsConnected(i))
			return 1;
	}

	return 0;
}
#endif

/**
 * Return 1 if card is protected.
 */
static uint8_t CardIsProtected(void)
{
	printf("-I- Cannot check if SD card is write-protected\n\r");
	return 0;
}

/**
 * Delay some loop
 */
static void LoopDelay(volatile unsigned int loop)
{
	for (; loop > 0; loop--);
}

/**
 * Display: Dump Splitting row
 */
static void DumpSeperator(void)
{
	printf("\n\r==========================================\n\r");
}

/**
 * Dump buffer
 * \param pData Pointer to data buffer.
 * \param len   Buffer length.
 */
static void DumpBuffer(unsigned char *pData, unsigned int len)
{
	unsigned int i;

	printf("-I- buffer %u: %c .. %c .. %c .. %c..",
		   len, pData[0], pData[3], pData[8], pData[8 + 5]);

	for (i = 0; i < len; i ++) {
		if ((i % 16) == 0) printf("\n\r%3x:", i);

		printf(" %02X", pData[i]);
	}

	printf("\n\r");
}

/**
 * \brief Get 32-bit number input (Dec or Hex).
 * Before first character input format can be changed once by
 * 'x' to hex.
 * \param nbChar Number of character to wait.
 * \param pNum   Pointer to uint32_t for input result.
 * \return 0 if valid data input.
 */
static uint8_t GetU32Input(uint8_t nbChar, uint32_t *pU32)
{
	uint8_t key, isHex = 0;
	uint32_t  i;
	uint32_t  result = 0;

	for (i = 0; i < nbChar;) {
		key = DBG_GetChar();

		/* User cancel input */
		if (key == 27) {
			printf(" Cancelled\n\r");
			return key;
		}

		/* Short input */
		if (key == '\r') break;

		if (key == 'x' && i == 0) {
			if (isHex == 0) {
				isHex = 1;
				DBG_PutChar(key);
			}

			continue;
		}

		if (key > '9' || key < '0') {
			if (isHex) {
				if (key < 'a' || key > 'z')
					continue;
			} else
				continue;
		}

		DBG_PutChar(key);

		if (isHex) {
			if (key >= 'a')
				result = result * 16 + (key - 'a' + 10);
			else
				result = result * 16 + (key - '0');
		} else
			result = result * 10 + (key - '0');

		i++;
	}

	if (pU32) *pU32 = result;

	return 0;
}

/**
 *  Dump a register
 */
static uint8_t DumpReg(uint8_t iMci)
{
	uint32_t addr;
	uint32_t data;

	DumpSeperator();
	printf("Address to read: ");

	if (0 == GetU32Input(5, &addr)) {
		printf("\n\r");

		if (0 == SDIO_ReadDirect(&sdDrv[iMci], curFunc, addr, (uint8_t *)&data, 1)) {
			printf("- SDIO.%d.0x%x: 0x%x\n\r",
				   curFunc, (unsigned int)addr, (uint8_t)data);
		}
	}

	return 0;
}

/**
 *  Write a register
 */
static uint8_t WriteReg(uint8_t iMci)
{
	uint32_t addr;
	uint32_t data;

	DumpSeperator();
	printf("Address to write: ");

	if (0 == GetU32Input(5, &addr)) {
		printf("\n\rData to write: ");

		if (0 == GetU32Input(3, &data)) {
			printf("\n\r- SDIO.%d0x%x = 0x%x\n\r",
				   curFunc, (unsigned int)addr, (unsigned int)data);
			SDIO_WriteDirect(&sdDrv[iMci], curFunc, addr, data);
		}
	}

	return 0;
}

/**
 *  Initialize the inserted card
 */
static uint8_t InitCard(uint8_t iMci)
{
	sSdCard *pSd = &sdDrv[iMci];
	uint8_t rc;

	/* Initialize the SD card driver */
	rc = SD_Init(pSd);

	if (rc)
		printf("-E- SD/MMC initialization failed: %x\n\r", rc);
	else
		printf("-I- SD/MMC card initialization successful\n\r");

	if (SD_GetCardType(pSd) & CARD_TYPE_bmSDIO) {
		SDIO_DumpCardInformation(pSd);
		return 1;
	} else
		printf("-I- Not a SDIO card, type %x\n\r", SD_GetCardType(pSd));

	return 0;
}

/**
 *  Perform test on SDIO.CIA
 */
static uint8_t TestCIA(uint8_t iMci)
{
	sSdCard *pSd = &sdDrv[iMci];

	DumpSeperator();
	/* SDIO always has FN1(IEN.1) and Mem(IEN.0), test with these bits */
	printf("R/W Direct test:\n\r");

	printf("CIA:\n\r");
	SDIO_ReadDirect(pSd, SDIO_CIA, 0x0, &pBuffer[0], 0x14);
	DumpBuffer(pBuffer, 0x14);

	printf("Write 0x03 to IEN(CIA.4): rc %d\n\r",
		   SDIO_WriteDirect(pSd, SDIO_CIA, SDIO_IEN_REG, 0x03));
	printf("IEN After Write:");

	SDIO_ReadDirect(pSd, SDIO_CIA, SDIO_IEN_REG, &pBuffer[4], 1);
	printf("0x%02X\n\r", pBuffer[4]);

	if (0x03 == pBuffer[4])
		printf("-- test OK\n\r");
	else
		printf("-- test FAIL\n\r");

	printf("R/W Extended test:\n\r");
	pBuffer[0x0] = 0x13;
	pBuffer[0x1] = 0x14;
	pBuffer[0x2] = 0x15;
	SDIO_WriteBytes(pSd, SDIO_CIA, 0xFD, 0, &pBuffer[0], 0x3, 0, 0);

	printf("CIA after write:\n\r");
	SDIO_ReadBytes(pSd, SDIO_CIA, 0xF0, 0, pBuffer, 0x10, 0, 0);
	DumpBuffer(pBuffer, 0x10);

	if (pBuffer[13] != 0x13)
		printf("-- CIA.2 Fail\n\r");
	else if (pBuffer[14] != 0x14)
		printf("-- CIA.4 Fail\n\r");
	else if (pBuffer[15] != 0x15)
		printf("-- CIA.4 Fail\n\r");
	else
		printf("-- test OK\n\r");

	/* Restore data to 0 */
	SDIO_WriteDirect(pSd, SDIO_CIA, SDIO_IOE_REG, 0);
	SDIO_WriteDirect(pSd, SDIO_CIA, SDIO_IEN_REG, 0);

	return 0;
}

/**
 * Initialize PIOs
 */
static void _ConfigurePIOs(void)
{
	/* Configure SDcard pins */
	PIO_Configure(pinsSd, PIO_LISTSIZE(pinsSd));
	/* Configure SD card detection */
	CardDetectConfigure();
	/* Configure SDcard power pins */
	//PIO_Configure(pinsPu, PIO_LISTSIZE(pinsPu));
	/* Check if card is write-protected (if supported) */
	CardIsProtected();
}

/**
 * Initialize driver instances.
 */
static void _ConfigureDrivers(void)
{
	uint32_t i;

	/* Initialize the DMA driver */
	XDMAD_Initialize(&dmaDrv, 0);

	NVIC_EnableIRQ(XDMAC_IRQn);

	/* Initialize the HSMCI driver */
	MCID_Init(&mciDrv[0], HSMCI, ID_HSMCI, BOARD_MCK, &dmaDrv, 0);

	NVIC_EnableIRQ(HSMCI_IRQn);

	/* Initialize SD driver */
	for (i = 0; i < BOARD_NUM_MCI; i++)
		SDD_InitializeSdmmcMode(&sdDrv[i], &mciDrv[i], 0);
}


/**
 * \brief Display menu.
 */
static void _DisplayMenu(void)
{
	printf("\n\r");
	printf("=========================================================\n\r");
	printf("Menu: press a key to test.\n\r");
	printf("---------------------------------------------------------\n\r");
	printf(" m: Switch SDIO connection (MCI0, MCI1).\n\r");
	printf(" r: Dump SDIO register value.\n\r");
	printf(" w: Write to SDIO register.\n\r");
	printf(" f: Change current function number.\n\r");
	printf("=========================================================\n\r");
}

/*----------------------------------------------------------------------------
 *         Global functions
 *----------------------------------------------------------------------------*/
/**
 *  \brief hsmci_sdcard Application entry point.
 *
 *  \return Unused (ANSI-C compatibility).
 */
int main(void)
{
	uint8_t sdState = 0; /* 0: no card, 1: connected, 2: error */

	/* Disable watchdog */
	WDT_Disable(WDT);

	SCB_EnableICache();
	SCB_EnableDCache();

	/* Output example information */
	printf("-- Basic HSMCI SDIO Example %s --\n\r", SOFTPACK_VERSION);
	printf("-- %s\n\r", BOARD_NAME);
	printf("-- Compiled: %s %s With %s--\n\r", __DATE__, __TIME__ , COMPILER_NAME);

	/* Initialize PIO pins */
	_ConfigurePIOs();

	/* Initialize drivers */
	_ConfigureDrivers();

	while (1) {
		/* Check card connection */
		if (CardIsConnected(bMciID)) {
			if (sdState == 0) {
				LoopDelay(BOARD_MCK / 50);

				if (InitCard(bMciID)) {
					sdState = 1;
					TestCIA(bMciID);
					_DisplayMenu();
				} else
					sdState = 2;
			}
		} else if (sdState) {
			printf("\n\r** Card disconnected\n\r");
			sdState = 0;
		}

		if (DBG_IsRxReady()) {
			uint8_t key = DBG_GetChar();

			switch (key | (sdState << 8)) {
			case 'm':
			case 0x100 | 'm':
			case 0x200 | 'm':
				bMciID = !bMciID;
				sdState = 0;
				break;

			case 0x100 | 'r':
				DumpReg(bMciID);
				break;

			case 0x100 | 'w':
				WriteReg(bMciID);
				break;

			case 0x100 | 't':
				TestCIA(bMciID);
				printf("\n\r");
				break;

			case 0x100 | 'f':
				curFunc = ((curFunc + 1) % 7);
				printf("** SDIO Function -> %d\n\r", curFunc);
				break;
			}

			_DisplayMenu();
		}
	}
}

