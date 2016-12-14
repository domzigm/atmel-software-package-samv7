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
 *  \page hsmci_multimedia_card Basic MultiMediaCard Example
 *
 *  \section Purpose
 *
 *  The Basic MultiMediaCard Example will help you to get familiar with HSMCI
 *  interface on SAM Microcontrollers. It can also help you to get familiar
 *  with the SD and MMC operation flow which can be used for fast implementation
 *  of your own SD/MMC drivers and other applications related.
 *
 *  \section Requirements
 *
 *  This package can be used with SAMV71 Xplained Ultra board or SAME70 Xplained board.
 *
 *  \section Description
 *
 *  Open HyperTerminal before running this program, the HyperTerminal will
 *  give out the test hints, you can run different tests on a inserted card.
 *
 *  \section Usage
 *
 * The MultiMedia Card Example offers a set of functions to perform
 *  MultiMedia Card tests:
 *  -# Dump MultiMedia Card information
 *  -# Test all blocks on MultiMedia Card
 *  -# Test R/W Speed (performance) of the MultiMedia Card
 *  You can find following information depends on your needs:
 *  - Usage of auto detection of sdcard insert and sdcard write-protection
 *  - (HS)MCI interface initialize sequence and interrupt installation
 *  - SD/MMC card driver implementation based on (HS)MCI interface
 *  - SD card physical layer initialize sequence implementation
 *  - MMC card physical layer initialize sequence implementation
 *  - Sample usage of SD/MMC card write and read
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
 *  -# In HyperTerminal, it will show something like on start up
 *      \code
 *      -- MultiMedia Card Example xxx --
 *      -- SAMxxxxx-xx
 *      -- Compiled: xxx xx xxxx xx:xx:xx --
 *      -I- Cannot check if SD card is write-protected
 *
 *      ==========================================
 *      -I- Card Type 1, CSD_STRUCTURE 0
 *      -I- SD 4-BITS BUS
 *      -I- CMD6(1) arg 0x80FFFF01
 *      -I- SD HS Not Supported
 *      -I- SD/MMC TRANS SPEED 25000 KBit/s
 *      -I- SD/MMC card initialization successful
 *      -I- Card size: 483 MB, 990976 * 512B
 *      ...
 *      \endcode
 *  -# Test function menu is like this
 *      \code
 *      # i,I   : Re-initialize card
 *      # t     : Disk R/W/Verify test
 *      # T     : Disk performance test
 *      # p     : Change number of blocks in one access for test
 *      \endcode
 *
 *  \par See Also
 *  - \ref hsmci_sdcard : Another Simple Example for SD/MMC access.
 *  - \ref sdmmc_lib : SD/MMC card driver with mci-interface.
 *  - \ref hsmci_module : sdcard physical layer driver with hsmci-interface.
 *
 *  \section References
 *  - hsmci_multimedia_card/main.c
 *  - hsmci.h
 *  - pio.h
 */

/**
 *  \file
 *
 * This file contains all the specific code for the hsmci_multimedia_card example.
 *
 *  \section Purpose
 *
 *  \section Contents
 *  The hsmci_multimedia_card application can be roughly broken down as follows:
 *     - Optional functions for detection (card insert, card protect)
 *        - CardDetectConfigure(), CardIsConnected()
 *        - CardIsProtected()
 *     - Interrupt handlers
 *        - MCI_IrqHandler()
 *     - The main function, which implements the program behaviour
 *        - I/O configuration
 *        - SD/MMC card auto-detect write-protected-check (if supported)
 *        - Initialize MCI interface and installing an isr relating to MCI
 *        - Initialize sdcard, get necessary sdcard's parameters
 *        - write/read sdcard
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

/** Maximum number of blocks read once (for performance test) */
#define NB_MULTI_BLOCKS     64//8192

/** Split R/W to 2, first R/W 4 blocks then remaining */
#define NB_SPLIT_MULTI      4

/** Test settings: start block address (0) */
#define TEST_BLOCK_START    (0)

/** Test settings: end block address (total SD/MMC) */
#define TEST_BLOCK_END      SD_GetNumberBlocks(&sdDrv[bMciID])

/** Test settings: skip size when "skip" key pressed */
#define TEST_BLOCK_SKIP     (100 * 1024 * 2)    // 100M

/** Test settings: Number of bytes to test performance */
#define TEST_PERFORMENCT_SIZE   (4*1024*1024)

/** Test settings: The value used to generate test data */
#define TEST_FILL_VALUE_U32     (0x5A6C1439)

/** Number of errors displayed */
#define NB_ERRORS       3

/** Number of bad blocks displayed */
#define NB_BAD_BLOCK    200

/*----------------------------------------------------------------------------
 *         Local variables
 *----------------------------------------------------------------------------*/

/** DMA driver instance */
static sXdmad dmaDrv;

/** MCI driver instance. */
static sMcid mciDrv[2];

COMPILER_ALIGNED(32) static sSdCard sdDrv[2];

/** Current selected MCI interface */
static uint8_t bMciID = 0;

/** SD card pins instance. */
static const Pin pinsSd[] = {BOARD_MCI_PINS_SLOTA, BOARD_MCI_PIN_CK};

/** SD card detection pin instance. */
static const Pin pinsCd[] = {BOARD_MCI_PIN_CD};

COMPILER_ALIGNED(32) static uint8_t pBuffer[SDMMC_BLOCK_SIZE * NB_MULTI_BLOCKS];

/** Number or errors detected */
static uint32_t nbErrors;

/** Number of block r/w once to test performance */
static uint32_t performanceMultiBlock = NB_MULTI_BLOCKS;

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
	uint32_t i;

	for (i = 0; i < BOARD_NUM_MCI; i ++)
		MCID_Handler(&mciDrv[i]);
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
 * Check if the card is connected.
 * \param iMci Controller number.
 * Return 1 if card is inserted.
 */
static uint8_t CardIsConnected(uint8_t iMci)
{
	return PIO_Get(&pinsCd[iMci]) ? 0 : 1;
}

/**
 * Check if the card is write protected.
 * \param iMci Controller number.
 * Return 1 if card is protected.
 */
static uint8_t CardIsProtected(void)
{
	printf("-I- Cannot check if SD card is write-protected\n\r");
	return 0;
}

/**
 * Get Dec Input
 * \param numChar Number of character to wait.
 * \param pInt    Pointer to uint32_t for input result.
 * \return 0 if valid data input.
 */
static uint8_t GetDecInput(uint8_t numChar, uint32_t *pInt)
{
	uint8_t key;
	uint32_t  i;
	uint32_t  result = 0;

	for (i = 0; i < numChar;) {
		key = DBG_GetChar();

		if (key == 27) {
			printf(" Canceled\n\r");
			return key;
		}

		if (key > '9' || key < '0') continue;

		DBG_PutChar(key);
		result = result * 10 + (key - '0');
		i ++;
	}

	if (pInt) *pInt = result;

	return 0;
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
 * Display: Dump main menu
 */
static void DumpMenu(void)
{
	DumpSeperator();
	printf("-!- MCK is %uMHz\n\r", (unsigned)BOARD_MCK / 1000000);
	printf("-!- Buffer@%x,size 0x%x\n\r", (unsigned int)pBuffer, sizeof(pBuffer));
	printf("# C     : Change number of blocks in one access for test\n\r");
	printf("# D     : Dump block contents\n\r");
	printf("# I     : Re-initialize card\n\r");
	printf("# T     : Disk R/W/Verify test\n\r");
	printf("# P     : Disk performance test\n\r");
	printf("# R     : Read verify only test\n\r");
	printf("# V     : Read verify with performance test\n\r");
}

/**
 * Dump buffer
 * \param pData Pointer to data buffer.
 * \param len   Buffer length.
 */
static void DumpBuffer(unsigned char *pData, unsigned int len)
{
	uint32_t i;
	printf("-I- buffer %u: %c .. %c .. %c .. %c..",
		   len, pData[0], pData[3], pData[8], pData[8 + 5]);

	for (i = 0; i < len; i ++) {
		if ((i % 16) == 0) printf("\n\r%3x:", (unsigned int)i);

		printf(" %02X", pData[i]);
	}

	printf("\n\r");
}

/**
 * Dump block & information
 * \param pData Pointer to data block.
 * \param block Block number.
 */
static void DumpBlock(uint8_t *pData, uint32_t block)
{
	uint32_t i;
	printf("-I- Block %d: %c .. %c .. %c .. %c..",
		   (int)block, pData[0], pData[3], pData[8], pData[8 + 5]);

	for (i = 0; i < 512; i ++) {
		if ((i % 16) == 0) printf("\n\r%3x:", (unsigned int)i);

		printf(" %02X", pData[i]);
	}

	printf("\n\r");
}

/**
 * Dump card registers
 * \param iMci Controller number.
 */
static void DumpCardInfo(uint8_t iMci)
{
	sSdCard *pSd = &sdDrv[iMci];

	if (SD_GetCardType(pSd) & CARD_TYPE_bmSDIO)
		SDIO_DumpCardInformation(pSd);

	if (SD_GetCardType(pSd) & CARD_TYPE_bmSDMMC) {
		SD_DumpCID(pSd->CID);
		SD_DumpCSD(pSd->CSD);

		if (SD_GetCardType(pSd) & CARD_TYPE_bmSD)
			SD_DumpSdStatus(pSd->SSR);
	}
}

/**
 * Run tests on the inserted card
 * \param iMci Controller number.
 */
static void CardInit(uint8_t iMci)
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
		return;
	}

	printf("-I- SD/MMC card initialization successful\n\r");

	if (SD_GetCardType(pSd) & CARD_TYPE_bmSDMMC) {
		printf("-I- MEM Card OK, size: %d MB", (int)SD_GetTotalSizeKB(pSd) / 1000);
		printf(", %d * %dB\n\r", (int)SD_GetNumberBlocks(pSd),
			   (int)SD_GetBlockSize(pSd));
	}

	if (SD_GetCardType(pSd) & CARD_TYPE_bmSDIO)
		printf("-I- IO Card Detected OK\n\r");

	DumpCardInfo(iMci);
}

/**
 * Block Dump (read)
 * \param iMci Controller number.
 */
static void BlockDump(uint8_t iMci)
{
	sSdCard *pSd = &sdDrv[iMci];
	uint32_t block;
	DumpSeperator();
	printf("-!- Input block:");

	if (GetDecInput(5, &block))
		return;

	printf("\n\r-I- Dump Block %d: %d\n\r",
		   (int)block, MMCT_ReadFun(pSd, block, 1, pBuffer));
	DumpBlock(pBuffer, block);
}

/**
 * Sdio test
 * \param iMci Controller number.
 */
static void SdioTest(uint8_t iMci)
{
	sSdCard *pSd = &sdDrv[iMci];
	uint32_t i;

	DumpSeperator();

	/* SDIO always has FN1(IEN.1) and Mem(IEN.0), test with these bits */
	printf("R/W Direct test:\n\r");

	printf("CIA:\n\r");
	SDIO_ReadDirect(pSd, SDIO_CIA, 0, &pBuffer[0], 0x14);
	DumpBuffer(pBuffer, 0x14);
	printf("Write 0x03 to IEN(CIA.4): rc %d\n\r",
		   SDIO_WriteDirect(pSd, SDIO_CIA, SDIO_IEN_REG, 0x03));
	printf("IEN After Write:");
	SDIO_ReadDirect(pSd, SDIO_CIA, SDIO_IEN_REG, &pBuffer[1], 1);
	printf("0x%02X\n\r", pBuffer[1]);

	if (0x03 == pBuffer[1])
		printf("-- test OK\n\r");
	else
		printf("-- test FAIL\n\r");

	SDIO_WriteDirect(pSd, SDIO_CIA, SDIO_IEN_REG, pBuffer[SDIO_IEN_REG]);

	printf("R/W Extended test:\n\r");
	printf("Dump CIA:\n\r");

	for (i = 0; i < 0x40; i ++) pBuffer[i] = 0xFF; /* Clear Buffer */

	SDIO_ReadBytes(pSd, SDIO_CIA, 0, 0, pBuffer, 0x39, 0, 0);
	DumpBuffer(pBuffer, 0x14);

	printf("Modify Some R/W bytes (2,4) for FN0 and write:\n\r");
	pBuffer[0x2] = 0x2; /* IOE */
	pBuffer[0x4] = 0x2; /* IEN */
	/* Dont write to CIA.0x7, CIA.0xD or operation pause transfer */
	SDIO_WriteBytes(pSd, SDIO_CIA, 2, 0, &pBuffer[2], 5/*(0xC-2)*/, 0, 0);
	printf("CIA after write:\n\r");
	//SDIO_ReadDirect(pSd, SDIO_CIA, 0, pBuffer, 0x14);
	SDIO_ReadBytes(pSd, SDIO_CIA, 0, 0, pBuffer, 0x14, 0, 0);
	DumpBuffer(pBuffer, 0x14);

	if (pBuffer[0x2] != 0x2)
		printf("-- CIA.2 Fail\n\r");
	else if (pBuffer[0x4] != 0x2)
		printf("-- CIA.4 Fail\n\r");
	else
		printf("-- test OK\n\r");

	/* Restore data to 0 */
	SDIO_WriteDirect(pSd, SDIO_CIA, SDIO_IOE_REG, 0);
	SDIO_WriteDirect(pSd, SDIO_CIA, SDIO_IEN_REG, 0);
}

/**
 * Sdio performance test
 * \param iMci Controller number.
 */
static void SdioPerformanceTest(uint8_t iMci)
{
	sSdCard *pSd = &sdDrv[iMci];
	uint32_t i, blkSize = 1, totalNb = 0;
	uint32_t tickStart, tickEnd, ticks, rwSpeed;

	DumpSeperator();
	printf("-I- IO Performance test, size %dK, MCK %dMHz\n\r",
		   TEST_PERFORMENCT_SIZE / 1024,
		   (int)(BOARD_MCK / 1000000));
	printf("Read Direct  speed: ");
	tickStart = GetTicks();

	for (totalNb = 0;;) {
		if (SDIO_ReadDirect(pSd, SDIO_CIA, 0, &pBuffer[0], 0x14))
			return;

		totalNb += 0x14;
		tickEnd = GetTicks();
		ticks = GetDelayInTicks(tickStart, tickEnd);

		if (ticks > 800) break;
	}

	rwSpeed = totalNb / ticks;
	printf("%uK\n\r", (unsigned int)rwSpeed);
	printf("Write Direct speed: ");
	tickStart = GetTicks();

	for (totalNb = 0;;) {
		for (i = 0; i < 0x14; i ++) {
			if (SDIO_WriteDirect(pSd,
								 SDIO_CIA,
								 SDIO_IEN_REG,
								 pBuffer[SDIO_IEN_REG]))
				return;
		}

		totalNb += 0x14;
		tickEnd = GetTicks();
		ticks = GetDelayInTicks(tickStart, tickEnd);

		if (ticks > 800) break;
	}

	rwSpeed = totalNb / ticks;
	printf("%uK\n\r", (unsigned int)rwSpeed);

	printf("R/W Extended test:\n\r");

	for (blkSize = 4; blkSize <= 512; blkSize <<= 1) {
		printf("- Cnt %3u: ", (unsigned int)blkSize);
		tickStart = GetTicks();

		for (totalNb = 0;;) {
			if (SDIO_ReadBytes(pSd,
							   SDIO_CIA, 0,
							   0,
							   pBuffer, blkSize,
							   0, 0))
				return;

			totalNb += blkSize;
			tickEnd = GetTicks();
			ticks = GetDelayInTicks(tickStart, tickEnd);

			if (ticks > 800) break;
		}

		rwSpeed = totalNb / ticks;
		printf("R %5uK, ", (unsigned int)rwSpeed);

		for (i = 0; i < blkSize; i ++)
			pBuffer[i] = pBuffer[SDIO_IEN_REG];

		tickStart = GetTicks();

		for (totalNb = 0; ;) {
			if (SDIO_WriteBytes(pSd,
								SDIO_CIA, SDIO_IEN_REG,
								1,
								pBuffer, blkSize,
								0, 0))
				return;

			totalNb += blkSize;
			tickEnd = GetTicks();
			ticks = GetDelayInTicks(tickStart, tickEnd);

			if (ticks > 800) break;
		}

		rwSpeed = totalNb / ticks;
		printf("W %5uK", (unsigned int)rwSpeed);
		printf("\n\r");
	}
}

/**
 * Disk test
 * \param iMci Controller number.
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
	printf("-!- Test code: 1.clr, 2.wr, 3.rd\n\r");

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

			/* - Read back the data to check the write operation */
			memset(pBuffer, 0xFF, SDMMC_BLOCK_SIZE * multiBlock);
			SCB_CleanDCache_by_Addr((uint32_t *)pBuffer, SDMMC_BLOCK_SIZE * multiBlock);
			error = MMCT_ReadFun(pSd, block, multiBlock, pBuffer);

			if (error) {
				printf("\n\r-E- 1. Read block (%d) #%u\n\r", error,
					   (unsigned int)block);

				if (MaxErrorBreak(0)) return;

				/* Skip following test */
				continue;
			}

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
			/* - Write a checker board pattern on the block */
			for (i = 0; i < SDMMC_BLOCK_SIZE * multiBlock; i++) {
				if ((i & 1) == 0)  pBuffer[i] = (i & 0x55);
				else               pBuffer[i] = (i & 0xAA);
			}

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

			errcnt = 0;

			for (i = 0; i < SDMMC_BLOCK_SIZE * multiBlock; i++) {

				if (!(((i & 1) == 0) && (pBuffer[i] == (i & 0x55))) &&
					!(((i & 1) != 0) && (pBuffer[i] == (i & 0xAA))) ) {
					uint32_t j, js;
					printf("\n\r-E- 2.0x%x. Data @ %u (0x%x)\n\r",
						   (unsigned int)errcnt, (unsigned int)i, (unsigned int)i);
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
 * Run disk performance test
 * R/W test can be masked to verify previous written data only
 * \param iMci Controller number.
 * \param wr   Do block write.
 * \param rd   Do block read.
 * \param errDetail Dump detailed error information.
 */
static void DiskPerformanceTest(uint8_t iMci,
								uint8_t wr,
								uint8_t rd,
								uint8_t errDetail)
{
	sSdCard *pSd = &sdDrv[iMci];
	uint8_t error = 0;
	uint32_t  block, i, nBadBlock = 0, nErrors;
	uint32_t  tickStart, tickEnd, ticks, rwSpeed;
	uint32_t *pBuf;
	DumpSeperator();
	printf("-I- Performance test, size %dK, Multi %d, MCK %dMHz\n\r",
		   TEST_PERFORMENCT_SIZE / 1024,
		   (int)performanceMultiBlock,
		   (int)(BOARD_MCK / 1000000));
#ifdef READ_MULTI
	printf("-I- Read by Multi block, size %d\n\r", (int)SDMMC_BLOCK_SIZE);
#else
	printf("-I- Read block by block, size %d\n\r", (int)SDMMC_BLOCK_SIZE);
#endif
#ifdef WRITE_MULTI
	printf("-I- Write by Multi block, size %d\n\r", (int)SDMMC_BLOCK_SIZE);
#else
	printf("-I- Write block by block, size %d\n\r", (int)SDMMC_BLOCK_SIZE);
#endif

	if (wr) {
		printf("--- Write test .. ");

		for (i = 0; i < SDMMC_BLOCK_SIZE * performanceMultiBlock; i += 4) {
			pBuf = (uint32_t *)(void *)(&pBuffer[i]);
			*pBuf = TEST_FILL_VALUE_U32;
		}

		nBadBlock = 0;
		tickStart = GetTicks();

		for (block = TEST_BLOCK_START;
			 block < (TEST_PERFORMENCT_SIZE / SDMMC_BLOCK_SIZE)
			 + TEST_BLOCK_START;
			 block += performanceMultiBlock) {
			pBuf = (uint32_t *)(void *)pBuffer;
			*pBuf = block;
			error = MMCT_WriteFun(pSd,
								  block, performanceMultiBlock,
								  pBuffer);

			if (error) {
				if (nBadBlock ++ >= NB_BAD_BLOCK) {
					printf("-E- WR_B(%u)\n\r", (unsigned int)block);
					break;
				} else error = 0;
			}
		}

		tickEnd = GetTicks();
		ticks = GetDelayInTicks(tickStart, tickEnd);
		rwSpeed = (TEST_PERFORMENCT_SIZE
				   - nBadBlock * performanceMultiBlock * SDMMC_BLOCK_SIZE)
				  / ticks;
		printf("Done, Bad %u, Speed %uK\n\r",
			   (unsigned int)nBadBlock, (unsigned int)rwSpeed);
	}

	if (rd) {
		printf("--- Read test .. ");
		nBadBlock = 0;
		tickStart = GetTicks();

		for (block = TEST_BLOCK_START;
			 block < (TEST_PERFORMENCT_SIZE / SDMMC_BLOCK_SIZE)
			 + TEST_BLOCK_START;
			 block += performanceMultiBlock) {

			error = MMCT_ReadFun(pSd,
								 block, performanceMultiBlock,
								 pBuffer);

			if (error) {
				if (nBadBlock ++ >= NB_BAD_BLOCK) {
					printf("-E- RD_B(%u)\n\r", (unsigned int)block);
					break;
				} else error = 0;
			}

			if (error) break;
		}

		tickEnd = GetTicks();
		ticks = GetDelayInTicks(tickStart, tickEnd);
		rwSpeed = (TEST_PERFORMENCT_SIZE
				   - nBadBlock * performanceMultiBlock * SDMMC_BLOCK_SIZE)
				  / ticks;
		printf("Done, read  %u, Speed %uK\n\r",
			   (unsigned int)(TEST_PERFORMENCT_SIZE -
							  nBadBlock * performanceMultiBlock * SDMMC_BLOCK_SIZE) / SDMMC_BLOCK_SIZE,
			   (unsigned int)rwSpeed);
	}

	printf("--- Data verify .. ");
	nErrors = 0;

	for (block = TEST_BLOCK_START;
		 block < (TEST_PERFORMENCT_SIZE / SDMMC_BLOCK_SIZE) + TEST_BLOCK_START;
		 block += performanceMultiBlock) {

		memset(pBuffer, 0x00, SDMMC_BLOCK_SIZE * performanceMultiBlock);
		SCB_CleanDCache_by_Addr((uint32_t *)pBuffer,
								SDMMC_BLOCK_SIZE * performanceMultiBlock);
		error = MMCT_ReadFun(pSd,
							 block, performanceMultiBlock,
							 pBuffer);

		if (error) {
			printf("-E- RD_B(%u)\n\r", (unsigned int)block);
			break;
		}

		pBuf = (uint32_t *)(void *)pBuffer;

		if (*pBuf != block) {
			if (errDetail) {
				if (nErrors ++ < NB_ERRORS) {
					printf("-E- Blk(%u)[0](%08x<>%08x)\n\r",
						   (unsigned int)block,
						   (unsigned int)block,
						   (unsigned int)(*pBuf));
				}
			} else {
				printf("-E- BlkN(%x<>%x)\n\r",
					   (unsigned int)block, (unsigned int)(*pBuf));
				error = 1;
				break;
			}
		}

		for (i = 4; i < SDMMC_BLOCK_SIZE * performanceMultiBlock; i += 4) {
			pBuf = (uint32_t *)(void *)(&pBuffer[i]);

			if ((*pBuf != TEST_FILL_VALUE_U32)) {
				if (errDetail) {
					/* Dump 10 errors only */
					if (nErrors ++ < NB_ERRORS) {
						uint32_t j;
						printf("-E- Blk(%u)[%u](%08x.. <>",
							   (unsigned int)block,
							   (unsigned int)i,
							   (unsigned int)TEST_FILL_VALUE_U32);

						for (j = (i > 4) ? (i - 4) : i;
							 j <= i + 4;
							 j += 4) {
							printf("%c%08X",
								   (i == j) ? '!' : ' ',
								   (unsigned int)(*pBuf));
						}

						printf(")\n\r");
					}
				} else {
					printf("-E- Blk(%u)[%u](%x<>%x)\n\r",
						   (unsigned int)block,
						   (unsigned int)i,
						   (unsigned int)TEST_FILL_VALUE_U32,
						   (unsigned int)(*pBuf));
					error = 1;
					break;
				}
			}
		}

		if (error) break;
	}

	if (errDetail && nErrors)
		printf("-I- %u u32 ERRORS found!\n\r", (unsigned int)nErrors);

	if (error)
		return;

	printf("OK\n\r");
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

	NVIC_ClearPendingIRQ(XDMAC_IRQn);
	NVIC_SetPriority(XDMAC_IRQn, 1);
	NVIC_EnableIRQ(XDMAC_IRQn);

	/* Initialize the HSMCI driver */
	MCID_Init(&mciDrv[0], HSMCI, ID_HSMCI, BOARD_MCK, &dmaDrv, 0);

	NVIC_ClearPendingIRQ(HSMCI_IRQn);
	NVIC_SetPriority(HSMCI_IRQn, 3);
	NVIC_EnableIRQ(HSMCI_IRQn);

	/* Initialize SD driver */
	for (i = 0; i < BOARD_NUM_MCI; i++)
		SDD_InitializeSdmmcMode(&sdDrv[i], &mciDrv[i], 0);
}

/*----------------------------------------------------------------------------
 *         Global functions
 *----------------------------------------------------------------------------*/

/**
 *  \brief hsmci_multimedia_card Application entry point.
 *
 *  \return Unused (ANSI-C compatibility).
 */
int main(void)
{
	uint8_t connected = 0;

	/* Disable watchdog */
	WDT_Disable(WDT);

	SCB_EnableICache();
	SCB_EnableDCache();
	TimeTick_Configure();

	/* Output example information*/
	printf("-- MultiMedia Card Example %s --\n\r", SOFTPACK_VERSION);
	printf("-- %s\n\r", BOARD_NAME);
	printf("-- Compiled: %s %s With %s--\n\r", __DATE__, __TIME__ , COMPILER_NAME);
	bMciID = 0;
	/* Initialize PIO pins */
	_ConfigurePIOs();

	/* Initialize drivers */
	_ConfigureDrivers();


	/* Card insert detection loop */
	for (; ;) {
		if (CardIsConnected(bMciID)) {
			if (connected == 0) {
				connected = 1;
				/* Delay before card initialize */
				Wait(300);
				/* Do card test */
				CardInit(bMciID);
				DumpMenu();
			}
		} else if (connected) {
			connected = 0;
			printf("** Card Disconnected\n\r");
		}

		if (DBG_IsRxReady()) {
			uint8_t key = DBG_GetChar();

			switch (key) {
			/* Change performance test block size */
			case 'c':
			case 'C': {
				if (performanceMultiBlock >= NB_MULTI_BLOCKS)
					performanceMultiBlock = 1;
				else
					performanceMultiBlock <<= 1;

				printf("-!- Performance Multi set to %d\n\r", (int)performanceMultiBlock);
			}
			break;

			/* Show help information */
			default:
				if (!connected)
					DumpMenu();
				else {
					switch (key) {
					/* Dump block contents */
					case 'd':
					case 'D':
						BlockDump(bMciID);
						break;

					/* Initialize the card again */
					case 'I':
					case 'i':
						CardInit(bMciID);
						break;

					/* Run test on whole disk */
					case 't':
					case 'T':
						if (SD_GetCardType(&sdDrv[bMciID]) & CARD_TYPE_bmSDIO)
							SdioTest(bMciID);

						if (SD_GetCardType(&sdDrv[bMciID]) & CARD_TYPE_bmSDMMC)
							DiskTest(bMciID, 1, 1, 1);

						printf("\n\r");
						break;

					/* Run performance test */
					case 'P':
					case 'p':
						if (SD_GetCardType(&sdDrv[bMciID]) & CARD_TYPE_bmSDIO)
							SdioPerformanceTest(bMciID);

						if (SD_GetCardType(&sdDrv[bMciID]) & CARD_TYPE_bmSDMMC)
							DiskPerformanceTest(bMciID, 1, 1, 0);

						printf("\n\r");
						break;

					/* Read/Verify ONLY test */
					case 'r':
					case 'R':
						DiskTest(bMciID, 0, 0, 1);
						printf("\n\r");
						break;

					/* Read/Verify ONLY performance test */
					case 'V':
					case 'v':
						DiskPerformanceTest(bMciID, 0, 1, 1);
						printf("\n\r");
						break;

					/* Show help information */
					default:
						DumpMenu();
					}
				}

				break;
			}
		}
	}
}

