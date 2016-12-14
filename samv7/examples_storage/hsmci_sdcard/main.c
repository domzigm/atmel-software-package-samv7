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
 * \page hsmci_sdcard Basic SD/MMC Card Example
 *
 * \section Purpose
 *
 *  The hsmci_sdcard will help you to get familiar with HSMCI interface on
 *  SAM Microcontrollers. It can also help you to get familiar with the SD
 *  operation flow which can be used for fast implementation of your own SD
 *  drivers and other applications related.
 *
 *  \section Requirements
 *
 *  This package can be used with SAMV71 Xplained Ultra board or SAME70 Xplained board.
 *
 *  \section Description
 *
 *  The demonstration program detects, initialize the SD/MMC memory card
 *  inserted, and performs R/W test on it.
 *
 *  Open HyperTerminal before running this program, use SAM-BA to download
 *  this program to SRAM or Flash, make the program run, the HyperTerminal
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
 *      -- HSMCI SD/MMC Example xxx --
 *      -- SAMxxxxx-xx
 *      -- Compiled: xxx xx xxxx xx:xx:xx --
 *      -I- Please connect a SD card ...
 *      -I- SD card connection detected
 *      -I- Cannot check if SD card is write-protected
 *      -I- SD/MMC card initialization successful
 *      -I- Card size: *** MB
 *      -I- Block size: *** Bytes
 *      -I- Testing block [  *** -   ***] ..."
 *      \endcode
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
 *  The hsmci_sdcard application can be roughly broken down as follows:
 *     - Optional functions
 *        - CheckProtection
 *        - WaitSdConn
 *     - Interrupt handlers
 *        - ISR_Mci0
 *     - The main function, which implements the program behaviour
 *        - I/O configuration
 *        - SD card auto-detect and check whether SD card is write-protected
 *          (if supported)
 *        - Initialize MCI interface and installing an isr relating to MCI
 *        - Initialize sdcard, get necessary sdcard's parameters
 *        - write/read sdcard at max available SD clock
 */
/*----------------------------------------------------------------------------
 *         Headers
 *----------------------------------------------------------------------------*/

#include "board.h"

#include "libsdmmc.h"
#include "Media.h"
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <assert.h>

/*----------------------------------------------------------------------------
 *         Local definitions
 *----------------------------------------------------------------------------*/

/** Maximum number of blocks read once */
#define NB_MULTI_BLOCKS     5

/** Split R/W to 2, first R/W 4 blocks then remaining */
#define NB_SPLIT_MULTI      4

/** Test settings: start block address (0) */
#define TEST_BLOCK_START    (0)

/** Test settings: end block address (total SD/MMC) */
#define TEST_BLOCK_END      SD_GetNumberBlocks(&sdDrv[bMciID])

/** Test settings: skip size when "skip" key pressed */
#define TEST_BLOCK_SKIP     (100 * 1024 * 2)    // 100M

/**  Number of errors displayed */
#define NB_ERRORS           5

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

COMPILER_ALIGNED(32) static uint8_t pBuffer[SDMMC_BLOCK_SIZE * NB_MULTI_BLOCKS];

/** Number of errors found */
static uint32_t nbErrors;
/*----------------------------------------------------------------------------
 *         Local macros
 *----------------------------------------------------------------------------*/

/* Defined to test Multi-Block functions */

/** \def READ_MULTI
 *  \brief Define to test multi-read (SD_Read())
 *         or
 *         single-read is used (SD_ReadBlocks()) */
#define READ_MULTI
/** \def WRITE_MULTI
 *  \brief Define to test multi-write (SD_Write())
 *         or
 *         single-write is used (SD_WriteBlocks()) */
#define WRITE_MULTI

/** \macro SDT_ReadFun
 * Function used for SD card test reading.
 * \param pSd  Pointer to a SD card driver instance.
 * \param address  Address of the block to read.
 * \param nbBlocks Number of blocks to be read.
 * \param pData    Data buffer whose size is at least the block size.
 */
#ifdef  READ_MULTI
#define MMCT_ReadFun(pSd, blk, nbBlk, pData) \
	SD_Read(pSd, blk, pData, nbBlk, NULL, NULL)
#else
#define MMCT_ReadFun(pSd, blk, nbBlk, pData) \
	SD_ReadBlocks(pSd, blk, pData, nbBlk)
#endif

/** \macro SDT_WriteFun
 * Function used for SD card test writing.
 * \param pSd  Pointer to a SD card driver instance.
 * \param address  Address of the block to read.
 * \param nbBlocks Number of blocks to be read.
 * \param pData    Data buffer whose size is at least the block size.
 */
#ifdef  WRITE_MULTI
#define MMCT_WriteFun(pSd, blk, nbBlk, pData) \
	SD_Write(pSd, blk, pData, nbBlk, NULL, NULL)
#else
#define MMCT_WriteFun(pSd, blk, nbBlk, pData) \
	SD_WriteBlocks(pSd, blk, pData, nbBlk)
#endif

/*----------------------------------------------------------------------------
 *         Local functions
 *----------------------------------------------------------------------------*/

/**
 * DMA interrupt handler.
 */
void XDMAC_Handler(void)
{
	XDMAD_Handler(&dmaDrv);
}

/**
 * MCI interrupt handler. Forwards the event to the MCI driver handlers.
 */
void HSMCI_Handler(void)
{
	MCID_Handler(&mciDrv[0]);
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
	for (; loop > 0; loop --);
}

/**
 * \brief Max Error Break
 * Check if max number of error achieved.
 * \param halt Whether halt the device if error number achieved.
 */
static uint8_t MaxErrorBreak(uint8_t halt)
{
	if (NB_ERRORS) {
		if (nbErrors ++ > NB_ERRORS) {
			while (halt);

			nbErrors = 0;
			return 1;
		}
	}

	return 0;
}

/**
 * Display: Dump Splitting row
 */
static void DumpSeperator(void)
{
	printf("\n\r==========================================\n\r");
}

/**
 * Dump card registers
 * \param slot Card slot (not used now).
 */
static void DumpCardInfo(uint8_t iMci)
{
	sSdCard *pSd = &sdDrv[iMci];

	if (SD_GetCardType(pSd) & CARD_TYPE_bmSDIO)
		SDIO_DumpCardInformation(pSd);

	if (SD_GetCardType(pSd) & CARD_TYPE_bmSDMMC) {
		SD_DumpCID(pSd->CID);
		SD_DumpCSD(pSd->CSD);
	}
}

/**
 * Run tests on the inserted card
 * \param slot Card slot (not used now).
 */
static uint8_t CardInit(uint8_t iMci)
{
	sSdCard *pSd = &sdDrv[iMci];
	uint8_t error;
	uint8_t retry = 2;

	DumpSeperator();

	while (retry --) {
		error = SD_Init(pSd);

		if (error == SDMMC_OK) break;
	}

	if (error) {
		printf("-E- SD/MMC card initialization failed: %d\n\r", error);
		return 1;
	}

	printf("-I- SD/MMC card initialization successful\n\r");
	printf("-I- Card size: %d MB", (int)SD_GetTotalSizeKB(pSd) / 1000);
	printf(", %d * %dB\n\r",
		   (int)SD_GetNumberBlocks(pSd), (int)SD_GetBlockSize(pSd));
	DumpCardInfo(iMci);
	return 0;
}

/**
 * Disk test
 * \param slot Card slot (not used now).
 * \param clr  Do block clear.
 * \param wr   Do block write.
 * \param rd   Do block read.
 */
static void DiskTest(uint8_t iMci,
					 uint8_t clr,
					 uint8_t wr,
					 uint8_t rd)
{
	sSdCard *pSd = &sdDrv[iMci];
	uint8_t error = 0;
	uint32_t i, errcnt = 0;
	uint32_t multiBlock, block, splitMulti;

	DumpSeperator();

	if (pSd->bCardType == CARD_SDIO) {
		printf("-!- SDIO only card, please run SDIO example\n\r");
		return;
	}

	printf("-!- MCI %d, code: 1.clr, 2.wr, 3.rd\n\r", iMci);

	/* Perform tests on each block */
	multiBlock = 0;

	for (block = TEST_BLOCK_START;
		 block < TEST_BLOCK_END;
		 block += multiBlock) {

		/* Perform different single or multiple bloc operations */
		if (multiBlock >= 16)   multiBlock <<= 1;
		else                    multiBlock ++;

		if (multiBlock > NB_MULTI_BLOCKS)
			multiBlock = 1;

		/* Multi-block adjustment */
		if (block + multiBlock > TEST_BLOCK_END)
			multiBlock = TEST_BLOCK_END - block;

		/* ** Perform single block or multi block transfer */
		printf("\r-I- Testing block [%6u - %6u] ...",
			   (unsigned int)block, (unsigned int)(block + multiBlock - 1));

		if (clr) {
			/* - Clear the block */
			memset(pBuffer, 0, SDMMC_BLOCK_SIZE * multiBlock);
			SCB_CleanDCache_by_Addr((uint32_t *)pBuffer, SDMMC_BLOCK_SIZE * multiBlock);

			for (i = 0; i < SDMMC_BLOCK_SIZE * multiBlock; i++) {
				if (pBuffer[i] != 0) {
					/* Fatal error */
					printf("\n\r-F- Data @ %u for write : 0x00 <> 0x%02x\n\r",
						   (unsigned int)i, pBuffer[i]);
					return;
				}
			}

			error = MMCT_WriteFun(pSd, block, multiBlock, pBuffer);

			if (error) {
				printf("\n\r-E- 1. Write block (%d) #%u\n\r", error,
					   (unsigned int)block);

				if (MaxErrorBreak(0)) return;

				/* Skip following test */
				continue;
			}

			__DMB();
			/* - Read back the data to check the write operation */
			memset(pBuffer, 0xFF, SDMMC_BLOCK_SIZE * multiBlock);
			SCB_CleanDCache_by_Addr((uint32_t *)pBuffer, SDMMC_BLOCK_SIZE * multiBlock);

			error = MMCT_ReadFun(pSd, block, multiBlock, pBuffer);

			if (error) {
				printf("\n\r-E- 1. Read block (%d) #%u\n\r",
					   error, (unsigned int)block);

				if (MaxErrorBreak(0)) return;

				/* Skip following test */
				continue;
			}

			__DMB();

			for (i = 0; i < SDMMC_BLOCK_SIZE * multiBlock; i++) {
				if (pBuffer[i] != 0) {
					printf("\n\r-E- 1. B%u.D[%u] : 0 <> 0x%02X\n\r",
						   (unsigned int)block, (unsigned int)i, (int)pBuffer[i]);

					if (MaxErrorBreak(0)) return;

					/* Only find first verify error. */
					break;
				}
			}
		}

		if (wr) {
			/* - Write a checkerboard pattern on the block */
			for (i = 0; i < SDMMC_BLOCK_SIZE * multiBlock; i++) {
				if ((i & 1) == 0)  pBuffer[i] = (i & 0x55);
				else               pBuffer[i] = (i & 0xAA);
			}

			SCB_CleanDCache_by_Addr((uint32_t *)pBuffer, SDMMC_BLOCK_SIZE * multiBlock);

			for (i = 0; i < multiBlock; ) {
				splitMulti = ((multiBlock - i) > NB_SPLIT_MULTI) ?
							 NB_SPLIT_MULTI : (multiBlock - i);
				error = MMCT_WriteFun(pSd,
									  block + i,
									  splitMulti,
									  &pBuffer[i * SDMMC_BLOCK_SIZE]);

				if (error) break;

				i += splitMulti;
			}

			__DMB();

			if (error) {
				printf("\n\r-E- 2. Write block #%u(%u+%u): %d\n\r",
					   (unsigned int)(block + i),
					   (unsigned int)block,
					   (unsigned int)i, error);

				if (MaxErrorBreak(0)) return;

				/* Skip Following Test */
				continue;
			}
		}

		if (rd) {
			/* - Read back the data to check the write operation */
			memset(pBuffer, 0, SDMMC_BLOCK_SIZE * multiBlock);
			SCB_CleanDCache_by_Addr((uint32_t *)pBuffer, SDMMC_BLOCK_SIZE * multiBlock);

			for (i = 0; i < multiBlock; ) {
				splitMulti = ((multiBlock - i) > NB_SPLIT_MULTI) ?
							 NB_SPLIT_MULTI : (multiBlock - i);
				error = MMCT_ReadFun(pSd,
									 block + i,
									 splitMulti,
									 &pBuffer[i * SDMMC_BLOCK_SIZE]);

				if (error) break;

				i += splitMulti;
			}

			if (error) {
				printf("\n\r-E- 2. Read block #%u(%u+%u): %d\n\r",
					   (unsigned int)(block + i),
					   (unsigned int)block,
					   (unsigned int)i, error);

				if (MaxErrorBreak(0)) return;

				/* Skip Following Test */
				continue;
			}

			__DMB();
			errcnt = 0;

			for (i = 0; i < SDMMC_BLOCK_SIZE * multiBlock; i++) {

				if (!(((i & 1) == 0) && (pBuffer[i] == (i & 0x55))) &&
					!(((i & 1) != 0) && (pBuffer[i] == (i & 0xAA))) ) {
					uint32_t j, js;
					printf("\n\r-E- 2.%d. Data @ %u (0x%x)\n\r",
						   (int)errcnt, (unsigned int)i, (unsigned int)i);
					printf("  -Src:");
					js = (i > 8) ? (i - 8) : 0;

					for (j = js; j < i + 8; j ++)
						printf(" %02x",
							   (unsigned int)(((j & 1) != 0) ? (j & 0xAA) : (j & 0x55)));

					printf("\n\r  -Dat:");

					for (j = js; j < i + 8; j ++)
						printf("%c%02x", (i == j) ? '!' : ' ', pBuffer[j]);

					printf("\n\r");

					if (MaxErrorBreak(0)) return;

					// Only find first 3 verify error.
					if (errcnt ++ >= 3)
						break;
				}
			}
		}

		if (DBG_IsRxReady()) {
			switch (DBG_GetChar()) {
			/* Skip 100M */
			case 'k':
				block += TEST_BLOCK_SKIP;

				if (block > TEST_BLOCK_END)
					block -= 5 + multiBlock;

				printf("\n\r");
				break;

			/* Cancel */
			case 'c':
				return;
			}
		}
	}

	printf("All block tested!\n\r");
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

	/* Enable XDMA interrupt and give it priority over any other peripheral interrupt */
	NVIC_ClearPendingIRQ(XDMAC_IRQn);
	NVIC_SetPriority(XDMAC_IRQn, 1);
	NVIC_EnableIRQ(XDMAC_IRQn);

	/* Initialize the HSMCI driver */
	MCID_Init(&mciDrv[0], HSMCI, ID_HSMCI, BOARD_MCK, &dmaDrv, 0 );

	/* Enable MCI interrupt and give it priority lower than DMA*/
	NVIC_ClearPendingIRQ(HSMCI_IRQn);
	NVIC_SetPriority(HSMCI_IRQn, 3);
	NVIC_EnableIRQ(HSMCI_IRQn);

	/* Initialize SD driver */
	for (i = 0; i < BOARD_NUM_MCI; i ++)
		SDD_InitializeSdmmcMode(&sdDrv[i], &mciDrv[i], 0);
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
	uint32_t i;
	uint8_t connected[BOARD_NUM_MCI];

	/* Disable watchdog */
	WDT_Disable(WDT);

	SCB_EnableICache();
	SCB_EnableDCache();

	/* Output example information */
	printf("-- HSMCI SD/MMC Example %s --\n\r", SOFTPACK_VERSION);
	printf("-- %s\n\r", BOARD_NAME);
	printf("-- Compiled: %s %s  With %s--\n\r", __DATE__, __TIME__ , COMPILER_NAME);


	/* Initialize PIO pins */
	_ConfigurePIOs();

	/* Initialize drivers */
	_ConfigureDrivers();

	/* Initialize connections */
	for (i = 0; i < BOARD_NUM_MCI; i++)
		connected[i] = 0;

	/* Check if any card is inserted */
	if (!AnyCardIsConnected()) {
		printf("-- Please insert a card\n\r");

		while (!AnyCardIsConnected());
	}

	/* Test all cards */
	for (i = 0; i < BOARD_NUM_MCI; i++) {
		if (connected[i] == 0 && CardIsConnected(i)) {
			connected[i] = 1;
			LoopDelay(BOARD_MCK / 1000 / 200);

			if (!CardInit(i))
				DiskTest(i, 1, 1, 1);
		}
	}

	while (1);
}

