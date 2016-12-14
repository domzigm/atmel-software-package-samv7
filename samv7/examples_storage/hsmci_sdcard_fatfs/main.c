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
 * \page hsmci_sdcard_fatfs Basic SD/MMC Card Example with fatfs file system
 *
 * \section Purpose
 *
 *  The hsmci_sdcard_fatfs will help you to get familiar with HSMCI interface
 * with fatfs support on
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
 *  - hsmci_sdcard_fatfs/main.c
 *  - hsmci.h
 *  - pio.h
 *
 */

/**
 *  \file
 *
 *  \section Purpose
 *
 *  This file contains all the specific code for the hsmci_sdcard_fatfs example.
 *
 *----------------------------------------------------------------------------
 *         Headers
 *----------------------------------------------------------------------------*/

#include "board.h"

#include "libsdmmc.h"
#include "../fatfs_config.h"
#include "Media.h"
#include "MEDSdcard.h"
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

/** Bytes per cluster, FS format is necessary to make it effective*/
#define AllOCSIZE        4096
/*----------------------------------------------------------------------------
 *         Local variables
 *----------------------------------------------------------------------------*/

/** Maximum number of LUNs which can be defined.*/
/** (Logical drive = physical drive = medium number)*/
#define MAX_LUNS        1

/** Available media.*/
extern sMedia medias[MAX_LUNS];

/** DMA driver instance */
static sXdmad dmaDrv;

/** MCI driver instance. */
static sMcid mciDrv[BOARD_NUM_MCI];

/** SDCard driver instance. */
extern  sSdCard sdDrv[BOARD_NUM_MCI];

/** SD card pins instance. */
static const Pin pinsSd[] = {BOARD_MCI_PINS_SLOTA, BOARD_MCI_PIN_CK};

/** SD card detection pin instance. */
static const Pin pinsCd[] = {BOARD_MCI_PIN_CD};


#define ID_DRV DRV_MMC

#if _FS_TINY == 0
	#define STR_ROOT_DIRECTORY "0:"
#else
	#define STR_ROOT_DIRECTORY ""
#endif

const char *FileName = STR_ROOT_DIRECTORY "Basic.bin";
const char *FileNameReadMe = STR_ROOT_DIRECTORY "ReadMe.txt";

const char *ReadMeText = "Samv7 FatFS example: Done!!";


/** size of the file to write/read.minimum size 512 for erase operation*/
#define DATA_SIZE 4096

uint8_t data[DATA_SIZE];

typedef struct _ALIGN_FATFS {
	uint8_t padding[16];
	FATFS fs;
} ALIGN_FATFS;

COMPILER_ALIGNED(32) ALIGN_FATFS aligned_fs;

/** Test settings: Number of bytes to test performance */
#define TEST_PERFORMENCT_SIZE   (4*1024*1024)


#define ASSERT(condition, ...)  { \
		if (!(condition)) { \
			printf("-F- ASSERT: "); \
			printf(__VA_ARGS__); \
		} \
	}
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

	for (i = 0; i < BOARD_NUM_MCI; i++) {
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
	TRACE_INFO("  Cannot check if SD card is write-protected\n\r");
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
	uint8_t error;
	uint8_t retry = 2;

	DumpSeperator();

	while (retry--) {
		error = SD_Init(&sdDrv[iMci]);

		if (error == SDMMC_OK) break;
	}

	if (error) {
		TRACE_ERROR("  SD/MMC card initialization failed: %d\n\r", error);
		return 1;
	}

	TRACE_INFO("  SD/MMC card initialization successful\n\r");
	TRACE_INFO("  Card size: %d MB", (int)SD_GetTotalSizeKB(&sdDrv[iMci]) / 1000);
	printf(", %d * %dB\n\r",
		   (int)SD_GetNumberBlocks(&sdDrv[iMci]),
		   (int)SD_GetBlockSize(&sdDrv[iMci]));
	DumpCardInfo(iMci);
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
	/* Check if card is write-protected (if supported) */
	CardIsProtected();
}

/**
 * Scan files under a certain path
 * \param path    folder path
 * return scan result, 1: success.
 */
static FRESULT scan_files (char *path)
{
	FRESULT res;
	FILINFO fno;
	DIR dir;
	int32_t i;
	char *fn;
#if _USE_LFN
	static char lfn[_MAX_LFN * (_DF1S ? 2 : 1) + 1];
	fno.lfname = lfn;
	fno.lfsize = sizeof(lfn);
#endif


	res = f_opendir(&dir, path);

	if (res == FR_OK) {
		i = strlen(path);

		for (;; ) {
			res = f_readdir(&dir, &fno);

			if (res != FR_OK || fno.fname[0] == 0) break;

#if _USE_LFN
			fn = *fno.lfname ? fno.lfname : fno.fname;
#else
			fn = fno.fname;
#endif

			if (*fn == '.') continue;

			if (fno.fattrib & AM_DIR) {
				sprintf(&path[i], "/%s", fn);
				res = scan_files(path);

				if (res != FR_OK) break;

				path[i] = 0;
			} else
				printf("%s/%s\n\r", path, fn);
		}
	}

	return res;
}

static uint8_t formatdisk(const TCHAR *pDrv)
{
	FRESULT res;
#if _FS_TINY == 0
	/** Format disk*/
	TRACE_INFO(" Please wait a moment. Formatting the disk...\n\r");
	res = f_mkfs(pDrv,    // Drv
				 0,    // FDISK partition
				 AllOCSIZE); // AllocSize
	TRACE_INFO("  Format disk finished !\n\r");

	if (res != FR_OK) {
		TRACE_ERROR("  f_mkfs pb: 0x%X\n\r", res);
		return 0;
	}

	return 1;
#else
	TRACE_INFO("  Please run Full version FAT FS test first\n\r");
	return 0;
#endif
}

/**
 * Do file system tests
 * \return test result, 1: success.
 */
static uint8_t RunFsTest(void)
{
	uint32_t i;
	uint32_t ByteToRead;
	uint32_t ByteRead;
	uint32_t len = 0;
#if _FS_TINY == 0
	uint32_t ByteWritten;
	char key;
#endif

	const TCHAR Drv_Num = ID_DRV;
	FRESULT res;
	DIR dirs;
	FIL FileObject;
	uint32_t tickStart, tickEnd, ticks, rwSpeed;

	gNbMedias = 1;
	/** Mount disk*/
	TRACE_INFO("Mount disk %d\n\r", ID_DRV);

	if (0 != ( ((uint32_t)&aligned_fs.fs.win) & (32 - 1) )) {
		TRACE_ERROR("field win in FATFS should aligned to cache line!\n\r");
		return 0;
	}

	/** Clear file system object*/
	memset(&aligned_fs.fs, 0, sizeof(FATFS));

	/* Mount the drive */
	res = f_mount(&aligned_fs.fs, &Drv_Num, 1);

	if (res != FR_OK) {
		/* If No file system found, format it */
		if (res == FR_NO_FILESYSTEM) {
			TRACE_INFO("No file System found\n\r");

			if (formatdisk(&Drv_Num)) {
				/* Try to mount again */
				res = f_mount(&aligned_fs.fs, &Drv_Num, 1);

				if (res != FR_OK) {
					TRACE_ERROR("f_mount pb: 0x%X\n\r", res);
					return 0;
				}
			} else
				return 0;
		} else {
			TRACE_ERROR("f_mount pb: 0x%X\n\r", res);
			return 0;
		}
	}

	/** Test if the disk is formatted*/
	res = f_opendir (&dirs, STR_ROOT_DIRECTORY);

	if (res == FR_OK) {
		/** erase sdcard to re-format it ?*/
		TRACE_INFO("The disk is already formatted.\n\r");

		/** Display the file tree*/
		TRACE_INFO(" Display files contained on the SDcard :\n\r");
		DumpSeperator();
		scan_files((char *)STR_ROOT_DIRECTORY);
		DumpSeperator();

#if _FS_TINY == 0
		TRACE_INFO("Do you want to erase the sdcard to re-format disk ? (y/n)!\n\r");
		key = DBG_GetChar();

		if ((key == 'y') || (key == 'Y')) {
			for (i = 0; i < 100; i++)
				MEDSdcard_EraseBlock(&medias[ID_DRV], i);

			TRACE_INFO(" Erase the first 100 blocks complete !\n\r");
			res = FR_NO_FILESYSTEM;
		}

#endif
	}

	if (res == FR_NO_FILESYSTEM) {

#if _FS_TINY == 0
		/** Format disk*/
		TRACE_INFO("Format disk %d\n\r", ID_DRV);

		if (!(formatdisk(&Drv_Num)))
			return 0;

#else
		TRACE_INFO("Please run Full version FAT FS test first\n\r");
		return 0;
#endif
	}

#if _FS_TINY == 0
	DumpSeperator();
	/** Create a new file*/
	TRACE_INFO("Create a file : \"%s\"\n\r", FileName);
	res = f_open(&FileObject, FileName, FA_CREATE_ALWAYS | FA_WRITE);

	if (res != FR_OK) {
		TRACE_ERROR("f_open create pb: 0x%X\n\r", res);
		return 0;
	}

	/** Write a checkerboard pattern in the buffer*/
	for (i = 0; i < sizeof(data); i++) {
		if ((i & 1) == 0)
			data[i] = (i & 0x55);
		else
			data[i] = (i & 0xAA);
	}

	TRACE_INFO("Writing to file\n\r");
	tickStart = GetTicks();

	for (i = 0; i < TEST_PERFORMENCT_SIZE; i += DATA_SIZE) {
		res = f_write(&FileObject, data, DATA_SIZE, (UINT*)&ByteWritten);

		if (res != FR_OK) {
			TRACE_ERROR("f_write pb: 0x%X\n\r", res);
			return 0;
		}
	}

	tickEnd = GetTicks();
	ticks = GetDelayInTicks(tickStart, tickEnd);
	rwSpeed = TEST_PERFORMENCT_SIZE / ticks;
	TRACE_INFO("Done, Bad %u, Speed %uK\n\r", (unsigned int)0, (unsigned int)rwSpeed);
	/** Close the file*/
	TRACE_INFO("Close file\n\r");
	res = f_close(&FileObject);

	if (res != FR_OK) {
		TRACE_ERROR("f_close pb: 0x%X\n\r", res);
		return 0;
	}

#endif

	DumpSeperator();
	/** Open the file*/
	TRACE_INFO("Open file to read: \"%s\"\n\r", FileName);
	res = f_open(&FileObject, FileName, FA_OPEN_EXISTING | FA_READ);

	if (res != FR_OK) {
		TRACE_ERROR("f_open read pb: 0x%X\n\r", res);
		return 0;
	}

	/** Read file*/
	TRACE_INFO("Read file\n\r");
	memset(data, 0, DATA_SIZE);
	ByteToRead = FileObject.fsize;
	tickStart = GetTicks();

	for (i = 0; i < ByteToRead; i += DATA_SIZE) {
		res = f_read(&FileObject, data, DATA_SIZE, (UINT*)&ByteRead);

		if (res != FR_OK) {
			TRACE_ERROR("f_read pb: 0x%X\n\r", res);
			return 0;
		}
	}

	tickEnd = GetTicks();
	ticks = GetDelayInTicks(tickStart, tickEnd);
	rwSpeed = ByteToRead / ticks;
	TRACE_INFO("Done, Bad %u, Speed %uK\n\r", (unsigned int)0, (unsigned int)rwSpeed);

	/** Close the file*/
	TRACE_INFO("Close file\n\r");
	res = f_close(&FileObject);

	if (res != FR_OK) {
		TRACE_ERROR("f_close pb: 0x%X\n\r", res);
		return 0;
	}

	/** compare read data with the expected data*/
	for (i = 0; i < sizeof(data); i++) {
		ASSERT((((i & 1) == 0) && (data[i] == (i & 0x55)))
			   || (data[i] == (i & 0xAA)),
			   "Invalid data at data[%u] (expected 0x%02X, read 0x%02X)\n\r",
			   (unsigned int)i, (unsigned int)(((i & 1) == 0) ? (i & 0x55) : (i & 0xAA)), data[i]);
	}

	TRACE_INFO("File data OK !\n\r");

	DumpSeperator();
	/** Create a new file*/
	TRACE_INFO("Create a file : \"%s\"\n\r", FileNameReadMe);
	res = f_open(&FileObject, FileNameReadMe, FA_CREATE_ALWAYS | FA_WRITE);

	if (res != FR_OK) {
		TRACE_ERROR("  f_open create pb: 0x%X\n\r", res);
		return 0;
	}

	TRACE_INFO("Write ReadMe file\n\r");
	tickStart = GetTicks();

	while (*ReadMeText != '\0') {
		res = f_write(&FileObject, ReadMeText++, 1 , (UINT*)&ByteWritten);
		len++;
	}

	if (res != FR_OK) {
		TRACE_ERROR("f_write pb: 0x%X\n\r", res);
		return 0;
	}

	tickEnd = GetTicks();
	ticks = GetDelayInTicks(tickStart, tickEnd);
	rwSpeed = len / ticks;
	TRACE_INFO("Done, Bad %u, Speed %uK\n\r", (unsigned int)0, (unsigned int)rwSpeed);
	/** Close the file*/
	TRACE_INFO("Close file\n\r");
	res = f_close(&FileObject);

	if (res != FR_OK) {
		TRACE_ERROR("  f_close pb: 0x%X\n\r", res);
		return 0;
	}

	DumpSeperator();
	return 1;
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
	MCID_Init(&mciDrv[0], HSMCI, ID_HSMCI, BOARD_MCK, &dmaDrv, 0);

	/* Enable MCI interrupt and give it priority lower than DMA*/
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
 *  \brief hsmci_sdcard_fatfs Application entry point.
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
	printf("-- HSMCI SD/MMC FatFS Example %s --\n\r", SOFTPACK_VERSION);
	printf("-- %s\n\r", BOARD_NAME);
	printf("-- Compiled: %s %s With %s--\n\r", __DATE__, __TIME__ , COMPILER_NAME);

	TimeTick_Configure();
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
			CardInit(i);
		}

		if (connected[i]) {
			if (RunFsTest()) {
				TRACE_INFO("  Test passed !\n\r");

			} else
				printf("-F- Test Failed !\n\r");
		}
	}

	while (1);
}

