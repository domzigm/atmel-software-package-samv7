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
 * \page isi_gray_bmp ISI Monochrome Capture Example
 *
 * \section Purpose
 *
 * This example demonstrates how to capture gray images using ISI and save the
 * images to SD Card in 256-bit color BMP files.
 *
 * \section Requirements
 *
 * This package can be used with SAM V71 Xplained Ultra board or SAME70 Xplained board.
 * The provided program uses the Image Sensor Interface to connects a CAMx_MT9v022 sensor
 * adept board mounted with a monochrome sensor MT9V022.
 *
 *  \section Description
 *
 *  The demonstration program detects, initialize the SD/MMC memory card
 *  inserted, and wait for any keys entered to the console to start capturing
 *  images and the following processing.
 *
 *  Open HyperTerminal before running this program, use SAM-BA to download
 *  this program to Flash, make the program run, the HyperTerminal
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
 *      -- ISI GRAY BMP Example xxx --
 *      -- SAMxxxxx-xx
 *      -- Compiled: xxx xx xxxx xx:xx:xx --
 *      \endcode
 *
 *  \section References
 *  - isi_gray_bmp/main.c
 *  - isi.c
 *  - twi.c
 *  - twid.c
 *  - pio.h
 *
 */

/**
 *  \file
 *
 *  \section Purpose
 *
 *  This file contains all the specific code for the isi_gray_bmp example.
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

/** TWI clock frequency in Hz. */
#define TWCK                    400000
/** TWI peripheral ID for Sensor configuration */
#define BOARD_ID_TWI_ISI        ID_TWIHS0
/** TWI base address for Sensor configuration */
#define BOARD_BASE_TWI_ISI      TWIHS0

/** Frame Buffer Descriptors */
#define ISI_MAX_PREV_BUFFER     1

/** TWI Pins definition */
#define BOARD_PINS_TWI_ISI      PINS_TWI0


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
#define AllOCSIZE		512
/*----------------------------------------------------------------------------
 *         Local variables
 *----------------------------------------------------------------------------*/
extern const sensorProfile_t mt9v022Profile;

/** ISI pins to configure. */
const Pin pinsTWI[] = BOARD_PINS_TWI_ISI;
const Pin pin_ISI_RST = BOARD_ISI_RST;
const Pin pin_ISI_PWD = BOARD_ISI_PWD;
const Pin pPinsISI[]= {BOARD_ISI_PINS};

/** TWI driver instance.*/
static Twid twid;

/** Image size */
static uint32_t wImageWidth, wImageHeight;


/* ISI DMA descriptor for codec path */
COMPILER_WORD_ALIGNED static ISI_FrameBufferDescriptors  codecBufDescList[ISI_MAX_PREV_BUFFER];

/* Image output format */
static sensorOutputFormat_t wImageFormat;

#define RGB_YUV_8B 0xFD
#define GRAY_10B   0xF9
#define UVC_10B    0xF6
#define UVC_12B    0xEE

#define CAMX_MT9V022_SLAVE_ADDR  (0x30>>1)

#define ISI_BUF_SIZE        640*480

/** Buffer for ISI capture */
static uint8_t * pIsiBuffer = (uint8_t *)SDRAM_CS_ADDR;

/** Buffer for BMP file */
static uint8_t * pBmpBuffer = (uint8_t *)(SDRAM_CS_ADDR + ISI_BUF_SIZE + 256);

static uint32_t bmpHeaderLength;

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

static char FileName[] = STR_ROOT_DIRECTORY "012345678.jpg";

typedef struct _ALIGN_FATFS{
	uint8_t padding[16];
	FATFS fs;
}ALIGN_FATFS;

COMPILER_ALIGNED(32) ALIGN_FATFS aligned_fs;

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

/**
 * \brief TWI initialization.
 */
static void _twiInit(void)
{
	/* Configure TWI pins. */
	PIO_Configure(pinsTWI, PIO_LISTSIZE(pinsTWI));
	/* Enable TWI peripheral clock */
	PMC_EnablePeripheral(BOARD_ID_TWI_ISI);
	/* Configure TWI */
	TWI_ConfigureMaster(BOARD_BASE_TWI_ISI, TWCK, BOARD_MCK);
	TWID_Initialize(&twid, BOARD_BASE_TWI_ISI);

	/* Configure TWI interrupts */
	NVIC_ClearPendingIRQ(TWIHS0_IRQn);
	NVIC_EnableIRQ(TWIHS0_IRQn);
}

/**
 * \brief ISI PCK initialization.
 */
static void _isiPckInit(void)
{
	/* Configure ISI pins. */
	PIO_Configure(pPinsISI, PIO_LISTSIZE(pPinsISI));

	/* Disable programmable clock 1 output */
	REG_PMC_SCDR = PMC_SCER_PCK0;
	/* Enable the DAC master clock */
	PMC->PMC_PCK[0] = PMC_PCK_CSS_MCK | (2<<4);
	/* Enable programmable clock 0 output */
	REG_PMC_SCER = PMC_SCER_PCK0;
	/* Wait for the PCKRDY0 bit to be set in the PMC_SR register*/
	while ((REG_PMC_SR & PMC_SR_PCKRDY0) == 0);
	/* ISI PWD OFF*/

	PIO_Clear(&pin_ISI_PWD);
	PIO_Clear(&pin_ISI_RST);
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
	TRACE_INFO("  Cannot check if SD card is write-protected\n\r");
	return 0;
}

/**
 * Delay some loop
 */
static void LoopDelay(volatile unsigned int loop)
{
	for(; loop > 0; loop --);
}

/**
 * Run tests on the inserted card
 * \param slot Card slot (not used now).
 */
static uint8_t CardInit(uint8_t iMci)
{
	uint8_t error;
	uint8_t retry = 2;

	while (retry --) {
		error = SD_Init(&sdDrv[iMci]);
		if (error == SDMMC_OK) break;
	}
	if (error) {
		TRACE_ERROR("  SD/MMC card initialization failed: %d\n\r", error);
		return 1;
	}
	TRACE_INFO("  SD/MMC card initialization successful\n\r");
	TRACE_INFO("  Card size: %d MB", (int)SD_GetTotalSizeKB(&sdDrv[iMci])/1000);
	printf(", %d * %dB\n\r",
		(int)SD_GetNumberBlocks(&sdDrv[iMci]),
		(int)SD_GetBlockSize(&sdDrv[iMci]));
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
static FRESULT scan_files (char* path)
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
		for (;;) {
			res = f_readdir(&dir, &fno);
			if (res != FR_OK || fno.fname[0] == 0)
				break;
#if _USE_LFN
			fn = *fno.lfname ? fno.lfname : fno.fname;
#else
			fn = fno.fname;
#endif
			if (*fn == '.') continue;
			if (fno.fattrib & AM_DIR) {
				sprintf(&path[i], "/%s", fn);
				res = scan_files(path);
				if (res != FR_OK)
					break;
				path[i] = 0;
			} else {
				printf("%s/%s\n\r", path, fn);
			}
		}
	}
	return res;
}

/**
 * Initialize driver instances.
 */
static void _ConfigureDrivers(void)
{
	uint32_t i;
	/* Initialize the DMA driver */
	XDMAD_Initialize(&dmaDrv,0);

	/* Enable XDMA interrupt and give it priority over any other peripheral interrupt */
	NVIC_ClearPendingIRQ(XDMAC_IRQn);
	NVIC_SetPriority(XDMAC_IRQn, 1);
	NVIC_EnableIRQ( XDMAC_IRQn );

	/* Initialize the HSMCI driver */
	MCID_Init(&mciDrv[0], HSMCI, ID_HSMCI, BOARD_MCK, &dmaDrv, 0 );

	/* Enable MCI interrupt and give it priority lower than DMA*/
	NVIC_ClearPendingIRQ(HSMCI_IRQn);
	NVIC_SetPriority(HSMCI_IRQn, 3);
	NVIC_EnableIRQ( HSMCI_IRQn );

	/* Initialize SD driver */
	for (i = 0; i < BOARD_NUM_MCI; i ++) {
		SDD_InitializeSdmmcMode(&sdDrv[i], &mciDrv[i], 0);
	}
}

/**
 * \brief There is a TWI to I/O IC PCA9557 to extend the I/O signal,
 * write register-1 of PCA9557 through TWI can change the 8-PIOs output status,
 * the 5 output (P0, P1, P2, P3, P4) connect the enable signal of the data buffer
 * IC (SN74AHC244) which control the ISI data path. The different data path select
 * different video mode.
 * \param inputMode  0: RGB/YUV 8B,
 *                   1: Grayscale 10B
 *                   2: UVC 10B Align ISI (D9..D2) + '10'
 *                   3: UVC 12B Align ISI (D11..D4) + '1000'
 */
static void camx_mt9v022_configure_mode(uint8_t inputMode)
{
	uint8_t reg = 0x0;
	uint8_t val;
	switch (inputMode){
	case 0:
		val = RGB_YUV_8B;
		break;
	case 1:
		val = GRAY_10B;
		break;
	case 2:
		val = UVC_10B;
		break;
	case 3:
		val = UVC_12B;
		break;
	default:
		val = 0xFF;
		break;
    }
	TWID_Write(&twid, CAMX_MT9V022_SLAVE_ADDR, 0x03, 1, &reg, 1, 0);
	TWID_Write(&twid, CAMX_MT9V022_SLAVE_ADDR, 1, 1, &val, 1, 0);
}

/**
 * \brief initialize ISI sensor for capturing images
 */
static void isi_init(void)
{
	camx_mt9v022_configure_mode(0);

	/* Initialize mono sensor */
	if (sensor_setup(&twid, &mt9v022Profile, VGA) != SENSOR_OK){
		printf("-E- Sensor setup failed.");
		while (1);
	}
	/* Retrieve sensor output format and size */
	sensor_get_output(&wImageFormat, &wImageWidth, &wImageHeight, VGA);

	/* Enable ISI peripheral clock */
	PMC_EnablePeripheral(ID_ISI);
	/* Reset ISI peripheral */
	ISI_Reset();
	/* Set the windows blank */
	ISI_SetBlank(0, 0);

	/* calculate scaler factor automatically. */
	ISI_calcScalerFactor();

	ISI_RgbSwapMode(0);
	ISI_setInputStream(0);
	ISI_YCrCbFormat(0);
	ISI_SetSensorSize(wImageWidth/2, wImageHeight);

	ISI_Enable();
}

/**
 * \brief Capture images using ISI sensor
 * \param pBuffer   pointer to the buffer image to store in
 */
static void isi_capture(void * pBuffer)
{
	volatile uint32_t delay;

	for (delay = 0; delay < 0xffff; delay++);

	/* prepare buffer descriptor */
	codecBufDescList[0].Current = (uint32_t)pBuffer;
	codecBufDescList[0].Control = ISI_DMA_C_CTRL_C_WB;
	codecBufDescList[0].Next    = NULL;
	ISI_setDmaInCodecPath((uint32_t)&codecBufDescList[0], ISI_DMA_C_CTRL_C_FETCH,
						(uint32_t)pBuffer);

	SCB_CleanDCache_by_Addr((uint32_t *)codecBufDescList, sizeof(codecBufDescList));

	/* capture image */
	ISI_DmaChannelEnable(ISI_DMA_CHER_C_CH_EN);
	for (delay = 0; delay < 0xffff; delay++);
	ISI->ISI_CR |= ISI_CR_ISI_CDC;

	/* wait capture finished */
	while (ISI_DMA_C_CTRL_C_DONE != (codecBufDescList[0].Control & ISI_DMA_C_CTRL_C_DONE))
		SCB_InvalidateDCache_by_Addr((uint32_t *)codecBufDescList, sizeof(codecBufDescList));

	for (delay = 0; delay < 0xffff; delay++);
	ISI_DmaChannelDisable(ISI_DMA_CHDR_C_CH_DIS);
}

/**
 * \brief Fill BMP's header and the palette for 256 color bitmap file
 *
 * \param pbuf      Pointer to the buffer to fill BMP file header and palette.
 * \param bmpHSize  BMP width.
 * \param bmpVSize  BMP height.
 * \return length of the header and the palette
*/
static uint32_t _fillBmpHeaderPalette(uint8_t * pbuf, uint32_t  bmpHSize, uint32_t  bmpVSize)
{
	uint32_t length;

	BMPHeader* header = (BMPHeader*)pbuf;

	uint32_t i;

	header->offset = sizeof(BMPHeader) + 256 * 4;
	header->type   = BMP_TYPE;
	header->fileSize   = (bmpHSize * bmpVSize) + header->offset;
	header->reserved1  = 0;
	header->reserved2  = 0;
	header->headerSize = BITMAPINFOHEADER;
	header->width  = bmpHSize;
	header->height = bmpVSize;
	header->planes = 1;
	header->bits   = 8;
	header->compression = 0;
	header->imageSize   = bmpHSize * bmpVSize;
	header->xresolution = 0;
	header->yresolution = 0;
	header->ncolours    = 0;
	header->importantcolours = 0;

	length = sizeof(BMPHeader);

	/* fill palette */
	for (i = 0; i < 256; i++) {
		pbuf[length++] = (uint8_t)i;
		pbuf[length++] = (uint8_t)i;
		pbuf[length++] = (uint8_t)i;
		pbuf[length++] = 0;
	}
	return length;
}

/**
 * \brief save image to BMP file
 *
 * \param pbuffer   Pointer to the pixel color.
 * \param width     BMP width.
 * \param height    BMP height.
 */
static uint32_t image_save_to_bmp(void * pbuffer, uint32_t width, uint32_t height)
{
#if _FS_TINY == 0
	uint32_t ByteWritten;
#endif
	static uint32_t mount = 0;
	uint32_t h, w, idx;
	uint32_t bmpSize = bmpHeaderLength;
	FRESULT res;
	FIL FileObject;

	const TCHAR Drv_Num = ID_DRV;
	DIR dirs;

	if (0 == mount){
		gNbMedias = 1;
		/** Mount disk*/
		TRACE_INFO("Mount disk %d\n\r", ID_DRV);
		/** Clear file system object*/
		memset(&aligned_fs.fs, 0, sizeof(FATFS));

		/* Mount the drive */
		res = f_mount(&aligned_fs.fs, &Drv_Num, 1);
		if ( res != FR_OK ) {
		/* If No file system found, format it */
			if (res == FR_NO_FILESYSTEM) {
				TRACE_INFO("No file System found\n\r");
				return 0;
			} else {
				TRACE_ERROR("f_mount pb: 0x%X\n\r", res);
				return 0;
			}
		}

		/** Test if the disk is formatted*/
		res = f_opendir(&dirs,STR_ROOT_DIRECTORY);
		if (res != FR_OK) {
			/** erase sdcard to re-format it ?*/
			TRACE_INFO("The disk is not formatted.\n\r");
			return 0;
		}
		mount = 1;
	}

#if _FS_TINY == 0
	/** Create a new file*/
	sprintf(FileName, "%s%08x.bmp", STR_ROOT_DIRECTORY, GetTicks());

	printf("Create a file : \"%s\"\n\r", FileName);
	res = f_open(&FileObject, FileName, FA_CREATE_ALWAYS|FA_WRITE);
	if ( res != FR_OK ) {
		TRACE_ERROR("f_open create pb: 0x%X\n\r", res);
		return 0;
	}

	/* for the image captured is upside down, here reverse it */
	for (h = 0; h < height; h++) {
		idx = (height - h - 1)*width;
		for (w = 0; w < width; w++) {
			pBmpBuffer[bmpSize++] = pIsiBuffer[idx + w];
		}
	}

	/* write data to BMP file */
	res = f_write(&FileObject, pBmpBuffer, bmpSize, &ByteWritten);
	if (res != FR_OK) {
		TRACE_ERROR("f_write pb: 0x%X\n\r", res);
		return 0;
	}

	/** Close the file*/
	res = f_close(&FileObject);
	if ( res != FR_OK ) {
		TRACE_ERROR("f_close pb: 0x%X\n\r", res);
		return 0;
	}
	printf("Create file %s OK!\n\r", FileName);
#endif
	return 0;
}

/*----------------------------------------------------------------------------
 *         Global functions
 *----------------------------------------------------------------------------*/

/**
 *  \brief isi_gray_bmp Application entry point.
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
	printf("-- ISI Gray BMP Example %s --\n\r", SOFTPACK_VERSION);
	printf("-- %s\n\r", BOARD_NAME);
	printf( "-- Compiled: %s %s With %s--\n\r", __DATE__, __TIME__, COMPILER_NAME);

	/* Enable SDRAM for Frame buffer */
	BOARD_ConfigureSdram();

	/* Configure Time Tick */
	TimeTick_Configure();
	/* Initialize PIO pins */
	_ConfigurePIOs();

	/* Initialize drivers */
	_ConfigureDrivers();
	/* Initialize connections */
	for (i = 0; i < BOARD_NUM_MCI; i ++) {
		connected[i] = 0;
	}

	/* Check if any card is inserted */
	if (!AnyCardIsConnected()) {
		printf("-- Please insert a card\n\r");
		while (!AnyCardIsConnected());
	}

	/* TWI Initialize */
	_twiInit();

	/* ISI PCK clock Initialize */
	_isiPckInit();

	PIO_Set(&pin_ISI_RST);

	/* set all image data line high resistance, so PA27 which is connect to both
	   ISI_D7 of image sensor and DAT3 of SD card can be at high voltage level,
	   and this forbids SD card working under SPI mode.
	*/
	camx_mt9v022_configure_mode(0xFF);

	for (i = 0; i < BOARD_NUM_MCI; i ++) {
		if (connected[i] == 0 && CardIsConnected(i)) {
			connected[i] = 1;
			LoopDelay(BOARD_MCK / 1000 / 200);
			CardInit(i);
		}
	}

	isi_init();

	/* header and palette of BMP file is fixed, fill only once */
	bmpHeaderLength = _fillBmpHeaderPalette(pBmpBuffer, wImageWidth, wImageHeight);

	printf( "\n\r-I- Press any key to capture an image ...\n\r");

	while (1) {
		DBG_GetChar();
		printf("-I- Capture Image!\n\r");
		isi_capture(pIsiBuffer);
		printf("-I- Image Captured!\n\r");
		SCB_InvalidateDCache_by_Addr((uint32_t *)pIsiBuffer, ISI_BUF_SIZE);
		image_save_to_bmp(pIsiBuffer, wImageWidth, wImageHeight);
		printf("\n\r");
	}
}

