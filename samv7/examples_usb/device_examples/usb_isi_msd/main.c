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

/** \cond usb_massstorage
 * \page usb_isi_msd USB Still Image Camera Example
 *
 * \section Purpose
 *
 * This example demonstrates an Still Image Camera with SAMV7/E7 Microcontrollers.
 *
 * \section Requirements
 *
 * This package can be used with SAMV71 Xplained Ultra board or SAME70 Xplained board
 *
 * - On-board ISI interface.
 * - External sensor, in the example, Omnivision OV7740 sensor could be used.
 * This package can be used with all Atmel Xplained board that have USB interface,
 * On-board ISI interface with an external CMOS-type image sensor board and SD
 * socket with a SD card inserted.
 *
 * \section Description
 *
 * The provided program uses the Image Sensor Interface to connects a CMOS-type
 * image sensor to the processor. It captures images and stores them to the SD
 * card in JPEG or BMP files.
 *
 * When the board running this program connected to a host (PC for example), with
 * USB cable, the board appears as a USB Disk for the host. Then we can capture
 * images and store them into SD card, or we can view the images stored previously.
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
 *     \code
 *     -- USB ISI MSD Example xxx --
 *     -- SAMxxxxx-xx
 *     -- Compiled: xxx xx xxxx xx:xx:xx --
 *     \endcode
 * The user can then choose any of the available options to perform the
 * described action.
 *
 * \section References
 * - usb_isi_msd/main.c
 * - twi.c
 * - twid.c
 * - isi.c
 * - memories: Storage Media interface for MSD
 * - usb: USB Framework, USB MSD driver and UDP interface driver
 *    - \ref usbd_framework
 *       - \ref usbd_api
 *    - \ref usbd_msd
 *       - \ref usbd_msd_drv
 */

/**
 * \file
 *
 * This file contains all the specific code for the \ref usb_isi_msd.
 */

/*----------------------------------------------------------------------------
 *        Headers
 *----------------------------------------------------------------------------*/

#include "board.h"

#include "libstoragemedia.h"
#include "libsdmmc.h"

#include "MSDDriver.h"
#include "MSDLun.h"

#include "../fatfs_config.h"
#include "Media.h"
#include "MEDSdcard.h"

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include "libjpeg.h"

int32_t isi_init(uint32_t index, uint32_t *pwImageWidth,
				 uint32_t *pwImageHeight);
void isi_capture_codec(void *pBuffer);
void isi_capture_preview(void *pBuffer);

/*----------------------------------------------------------------------------
 *        Compiling Options
 *----------------------------------------------------------------------------*/

/* No transfer speed dump */
#define DBG_SPEED_OFF
#define USBHS_PRI                       3
#define HSMCI_PRI                       2
#define XDMAC_PRI                       1
/*----------------------------------------------------------------------------
 *        Local definitions
 *----------------------------------------------------------------------------*/

/** Maximum number of LUNs which can be defined. */
#define MAX_LUNS            1

/** Media index for different disks */
#define DRV_SDMMC           0    /**< SD card */

/** Size of one block in bytes. */
#define BLOCK_SIZE          512

/** Size of the MSD IO buffer in bytes (150K, more the better). */
#define MSD_BUFFER_SIZE     (128 * BLOCK_SIZE)

#define ID_DRV DRV_MMC

#if _FS_TINY == 0
	#define STR_ROOT_DIRECTORY "0:"
#else
	#define STR_ROOT_DIRECTORY ""
#endif

#define BMP_HEADER_LENGTH    0x100

/*----------------------------------------------------------------------------
 *        Global variables
 *----------------------------------------------------------------------------*/

/** MSD Driver Descriptors List */
extern const USBDDriverDescriptors msdDriverDescriptors;

/** SD card pins instance. */
static const Pin pinsSd[] = {BOARD_MCI_PINS_SLOTA, BOARD_MCI_PIN_CK};

/** SD card detection pin instance. */
static const Pin pinsCd[] = {BOARD_MCI_PIN_CD};

/** Available media. */
extern sMedia medias[MAX_LUNS];

/*----------------------------------------------------------------------------
 *        Local variables
 *----------------------------------------------------------------------------*/
/** DMA driver instance */
static sXdmad dmaDrv;

/** Device LUNs. */
static MSDLun luns[MAX_LUNS];

/** SDCard driver instance. */
extern sSdCard sdDrv[BOARD_NUM_MCI];

/** MCI driver instance. */
/** SDCard driver instance. */
static sMcid mciDrv[BOARD_NUM_MCI];

/** LUN read/write buffer. */
static uint8_t mSdBuffer[MSD_BUFFER_SIZE];

/** Total data write to disk */
uint32_t msdWriteTotal = 0;

/** Delay TO event */
uint8_t  msdRefresh = 0;

/** Image size */
uint32_t wImageWidth, wImageHeight;

/** Buffer for ISI capture */
static uint16_t *pIsiBuffer  = (uint16_t *)SDRAM_CS_ADDR;
/** Buffer for JPEG compress result */
static uint16_t *pJpegBuffer = (uint16_t *)(SDRAM_CS_ADDR + 640 * 480 * 2);
/** Buffer for write files to SD card */
static uint8_t writeBuffer[_MAX_SS];
/** index of the write buffer */
static uint32_t writeBufIdx;

static FATFS fs;

static char FileName[] = STR_ROOT_DIRECTORY "012345678.jpg";

/**
 * XDMA0 interrupt handler.
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

/*-----------------------------------------------------------------------------
 *         Callback re-implementation
 *-----------------------------------------------------------------------------*/

/**
 * Invoked when a new SETUP request is received from the host. Forwards the
 * request to the Mass Storage device driver handler function.
 * \param request  Pointer to a USBGenericRequest instance.
 */
void USBDCallbacks_RequestReceived(const USBGenericRequest *request)
{
	MSDDriver_RequestHandler(request);
}

/**
 * Invoked when the configuration of the device changes. Resets the mass
 * storage driver.
 * \param cfgnum New configuration number.
 */
void USBDDriverCallbacks_ConfigurationChanged(unsigned char cfgnum)
{
	MSDDriver_ConfigurationChangeHandler(cfgnum);
}

/*----------------------------------------------------------------------------
 *        Callbacks
 *----------------------------------------------------------------------------*/
/**
 * Invoked when the MSD finish a READ/WRITE.
 * \param flowDirection 1 - device to host (READ10)
 *                      0 - host to device (WRITE10)
 * \param dataLength Length of data transferred in bytes.
 * \param fifoNullCount Times that FIFO is NULL to wait
 * \param fifoFullCount Times that FIFO is filled to wait
 */
static void MSDCallbacks_Data(uint8_t flowDirection,
							  uint32_t  dataLength,
							  uint32_t  fifoNullCount,
							  uint32_t  fifoFullCount)
{
	fifoNullCount = fifoNullCount; /* dummy */
	fifoFullCount = fifoFullCount;  /*dummy */

	if (!flowDirection)
		msdWriteTotal += dataLength;
}
/*----------------------------------------------------------------------------
 *        Local functions
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
 * Run init on the inserted card
 * \param iMci Controller number.
 */
static void CardInit(sSdCard *pSd)
{
	uint8_t error;
	uint8_t retry = 2;

	while (retry --) {
		error = SD_Init(pSd);

		if (error == SDMMC_OK) break;
	}

	if (error) {
		TRACE_ERROR("SD/MMC card initialization failed: %d\n\r", error);
		return;
	}

	TRACE_INFO(" SD/MMC card initialization successful\n\r");

	if (SD_GetCardType(pSd) & CARD_TYPE_bmSDMMC) {
		TRACE_INFO(" MEM Card OK, size: %d MB", (int)SD_GetTotalSizeKB(pSd) / 1000);
		TRACE_INFO(", %d * %dB\n\r", (int)SD_GetNumberBlocks(pSd),
				   (int)SD_GetBlockSize(pSd));
	}

	if (SD_GetCardType(pSd) & CARD_TYPE_bmSDIO) {
		TRACE_ERROR("-E- IO Card Detected \n\r");
	}
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
}

/**
 * Initialize driver instances.
 */
static void _ConfigureDrivers(void)
{
	uint32_t i;
	/* Initialize the DMA driver */
	XDMAD_Initialize(&dmaDrv, 0);

	/* Enable XDMA interrupt and give it priority over any other peripheral
	    interrupt */
	NVIC_ClearPendingIRQ(XDMAC_IRQn);
	NVIC_SetPriority(XDMAC_IRQn, XDMAC_PRI);
	NVIC_EnableIRQ(XDMAC_IRQn);

	/* Initialize the HSMCI driver */
	MCID_Init(&mciDrv[0], HSMCI, ID_HSMCI, BOARD_MCK, &dmaDrv, 0);

	/* Enable MCI interrupt and give it priority lower than DMA*/
	NVIC_ClearPendingIRQ(HSMCI_IRQn);
	NVIC_SetPriority(HSMCI_IRQn, HSMCI_PRI);
	NVIC_EnableIRQ(HSMCI_IRQn);

	/* Initialize SD driver */
	for (i = 0; i < BOARD_NUM_MCI; i ++)
		SDD_InitializeSdmmcMode(&sdDrv[i], &mciDrv[i], 0);
}

/*----------------------------------------------------------------------------
 *         Internal functions
 *----------------------------------------------------------------------------*/
static void SDDiskInit(sSdCard *pSd)
{
	uint8_t sdConnected;
	pSd = &sdDrv[0];
	/* Infinite loop */
	sdConnected = 0;

	if (CardIsConnected(0)) {
		if (sdConnected == 0) {
			sdConnected = 1;
			printf("-I- connect to solt n\r");
			CardInit(pSd);

			SD_DumpCID(pSd->CID);
			SD_DumpCSD(pSd->CSD);
			SD_DumpExtCSD(pSd->EXT);
			MEDSdusb_Initialize(&medias[DRV_SDMMC], pSd);
		}
	} else if (sdConnected) {
		sdConnected = 0;
		printf("** Card Disconnected\n\r");
	}

	LUN_Init(&(luns[DRV_SDMMC]),
			 &(medias[DRV_SDMMC]),
			 mSdBuffer, MSD_BUFFER_SIZE,
			 0, 0, 0, 0,
			 MSDCallbacks_Data);
}

/**
 * Initialize MSD Media & LUNs
 */
static void _MemoriesInitialize(sSdCard *pSd)
{
	uint32_t i;

	/* Reset all LUNs */
	for (i = 0; i < MAX_LUNS; i ++)
		LUN_Init(&luns[i], 0, 0, 0, 0, 0, 0, 0, 0);

	BOARD_ConfigureSdram();
	/*Initialize SD Card  */
	SDDiskInit(pSd);

	gNbMedias = 2;
}

/*---------------------------------------------------------------------------
 *         VBus monitoring (optional)
 *---------------------------------------------------------------------------*/
#ifdef VBUS_DETECTION

/** VBus pin instance. */
static const Pin pinVbus = PIN_USB_VBUS;
/**
 * Handles interrupts coming from PIO controllers.
 */
static void ISR_Vbus(const Pin *pPin)
{
	/* Check current level on VBus */
	if (PIO_Get(&pinVbus)) {

		TRACE_INFO("VBUS conn\n\r");
		USBD_Connect();
	}
	else {

		TRACE_INFO("VBUS discon\n\r");
		USBD_Disconnect();
	}
}
#endif


/**
 * Configures the VBus pin to trigger an interrupt when the level on that pin
 * changes if it exists.
 */
static void VBus_Configure( void )
{
	TRACE_INFO("VBus configuration\n\r");

#ifdef VBUS_DETECTION
	/* Configure PIO */
	PIO_Configure(&pinVbus, PIO_LISTSIZE(pinVbus));
	PIO_ConfigureIt(&pinVbus, ISR_Vbus);
	PIO_EnableIt(&pinVbus);

	/* Check current level on VBus */
	if (PIO_Get(&pinVbus)) {

		/* if VBUS present, force the connect */
		TRACE_INFO("conn\n\r");
		USBD_Connect();
	}
	else {
		USBD_Disconnect();
	}
#else
	printf("No VBus Monitor\n\r");
	USBD_Connect();

#endif

}

/**
 * \brief compress image in buffer to jpeg file
 * \param pImageSrc     pointer to image buffer to compressed
 * \param pImageDst     pointer to buffer to store the jpeg
 * \param wImageWidth   image width
 * \param wImageHeight  image height
 * \param pwDstLength   pointer to the jpeg length
 *
 */
static void jpeg_compress(void *pImageSrc, void *pImageDst,
						  uint32_t wImageWidth, uint32_t wImageHeight,
						  uint32_t *pwDstLength)
{
	SJpegData sJpegData;

	*pwDstLength = 0;

	/* Compress image*/
	JpegData_Init(&sJpegData);

	/* use captured buffer as direct source*/
	JpegData_SetSource(&sJpegData, pImageSrc, wImageWidth * wImageHeight * 2);
	JpegData_SetDestination(&sJpegData, pImageDst,
							 wImageWidth * wImageHeight * 2);
	JpegData_SetDimensions(&sJpegData, wImageWidth, wImageHeight, 2);
	JpegData_SetParameters(&sJpegData, 75, JPG_DATA_YCbCr, JPG_METHOD_IFAST);

	if (ijg_compress_raw_no_padding(&sJpegData) == 0) {
		*pwDstLength = sJpegData.dwDstLength;
		printf("-I- compress ok!\r\n");
	}
}

/**
 * \brief mount file system
 */
static FRESULT mount_file_system(void)
{
	FRESULT res;
	const TCHAR Drv_Num = ID_DRV;
	DIR dirs;

	static uint32_t mount = 0;

	if (0 == mount) {
		gNbMedias = 1;
		/** Mount disk*/
		TRACE_INFO("Mount disk %d\n\r", ID_DRV);
		/** Clear file system object*/
		memset(&fs, 0, sizeof(FATFS));

		/* Mount the drive */
		res = f_mount(&fs, &Drv_Num, 1);

		if (res != FR_OK) {
			/* If No file system found, format it */
			if (res == FR_NO_FILESYSTEM) {
				TRACE_INFO("No file System found\n\r");
				return res;
			} else {
				TRACE_ERROR("f_mount pb: 0x%X\n\r", res);
				return res;
			}
		}

		/** Test if the disk is formatted*/
		res = f_opendir (&dirs, STR_ROOT_DIRECTORY);

		if (res != FR_OK) {
			/** erase sdcard to re-format it ?*/
			TRACE_INFO("The disk is not formatted.\n\r");
			return res;
		}

		mount = 1;
	}

	return FR_OK;
}

/**
 * \brief save compressed image to jpeg file
 */
static uint32_t image_save_to_jpeg(void *pbuffer, uint32_t size)
{
#if _FS_TINY == 0
	uint32_t ByteWritten;
#endif

	FRESULT res;
	FIL FileObject;

	if (FR_OK != mount_file_system())
		return 0;

#if _FS_TINY == 0
	/** Create a new file*/
	sprintf(FileName, "%s%08x.jpg", STR_ROOT_DIRECTORY, GetTicks());

	printf("Create file \"%s\"\n\r", FileName);
	res = f_open(&FileObject, FileName, FA_CREATE_ALWAYS | FA_WRITE);

	if (res != FR_OK) {
		TRACE_ERROR("f_open create pb: 0x%X\n\r", res);
		return 0;
	}

	/** Write compressed data to JPEG file */
	res = f_write( &FileObject, pJpegBuffer, size, &ByteWritten);

	if (res != FR_OK) {
		TRACE_ERROR("f_write pb: 0x%X\n\r", res);
		return 0;
	}

	/** Close the file*/
	res = f_close(&FileObject);

	if (res != FR_OK) {
		TRACE_ERROR("f_close pb: 0x%X\n\r", res);
		return 0;
	}

	printf("Create file \"%s\" OK!\n\r", FileName);
#endif
	return 0;
}

/**
 * \brief write the buffer to SD file
 */
static uint32_t flush_write_buf(FIL *pFile)
{
	FRESULT res;
	uint32_t ByteWritten;

	if (writeBufIdx >= sizeof(writeBuffer)) {
		res = f_write(pFile, writeBuffer, writeBufIdx, &ByteWritten);
		writeBufIdx = 0;

		if (res != FR_OK) {
			TRACE_ERROR("f_write pb: 0x%X\n\r", res);
			return 1;
		}
	}

	return 0;
}

/**
 * \brief save image to BMP file
 */
static uint32_t image_save_to_bmp(void *pbuffer, uint32_t wImageWidth,
								  uint32_t wImageHeight)
{
#if _FS_TINY == 0
	uint32_t ByteWritten;
#endif

	uint32_t i, j;
	FRESULT res;
	FIL FileObject;
	uint16_t color;
	uint16_t *pRGB565;

	if (FR_OK != mount_file_system())
		return 0;

	WriteBMPheader((uint32_t *)writeBuffer, wImageWidth, wImageHeight, 3);
	writeBufIdx = BMP_HEADER_LENGTH;

#if _FS_TINY == 0
	/** Create a new file*/
	sprintf(FileName, "%s%08x.bmp", STR_ROOT_DIRECTORY, GetTicks());

	printf("Create file \"%s\"\n\r", FileName);
	res = f_open(&FileObject, FileName, FA_CREATE_ALWAYS | FA_WRITE);

	if (res != FR_OK) {
		TRACE_ERROR("f_open create pb: 0x%X\n\r", res);
		return 0;
	}

	/** Write BMP header: header is stored in buffer temperately, write to file later */

	/** Write BMP body */
	for (i = 0; i < wImageHeight; i++) {
		pRGB565 = pbuffer;
		pRGB565 += (wImageHeight - 1 - i) * wImageWidth;

		for (j = 0; j < wImageWidth; j++) {
			color = *pRGB565++;

			writeBuffer[writeBufIdx++] = (uint8_t)((color >>  0) & 0x1F) << 3;

			if (0 != flush_write_buf(&FileObject))
				return 0;

			writeBuffer[writeBufIdx++] = (uint8_t)((color >>  5) & 0x3F) << 2;

			if (0 != flush_write_buf(&FileObject))
				return 0;

			writeBuffer[writeBufIdx++] = (uint8_t)((color >> 11) & 0x1F) << 3;

			if (0 != flush_write_buf(&FileObject))
				return 0;
		}

		if (0 == (i & 0x07))
			printf(".");
	}

	if (writeBufIdx) {
		res = f_write( &FileObject, writeBuffer, writeBufIdx, &ByteWritten);
		writeBufIdx = 0;

		if (res != FR_OK) {
			TRACE_ERROR("f_write pb: 0x%X\n\r", res);
			return 0;
		}
	}

	/** Close the file*/
	res = f_close(&FileObject);

	if (res != FR_OK) {
		TRACE_ERROR("f_close pb: 0x%X\n\r", res);
		return 0;
	}

	printf("\r\nCreate file \"%s\" OK!\n\r", FileName);
#endif
	return 0;
}

/**
 * \brief capture image and compress it, then save to jpeg file
 */
static void capture_image_to_jpeg(void)
{
	uint32_t wJpegLength;

	printf("-I- Capture Image!\n\r");
	isi_capture_codec(pIsiBuffer);

	SCB_CleanInvalidateDCache();

	printf("-I- compress start!\n\r");
	__disable_irq();
	jpeg_compress(pIsiBuffer, pJpegBuffer,
				  wImageWidth, wImageHeight, &wJpegLength);
	__enable_irq();

	if (0 != wJpegLength)
		image_save_to_jpeg(pJpegBuffer, wJpegLength);
}

/**
 * \brief capture image and compress it, then save to jpeg file
 */
static void capture_image_to_bmp(void)
{
	printf("-I- Capture Image!\n\r");
	isi_capture_preview(pIsiBuffer);

	SCB_CleanInvalidateDCache();

	image_save_to_bmp(pIsiBuffer, wImageWidth, wImageHeight);
}

/*----------------------------------------------------------------------------
 *        Exported functions
 *----------------------------------------------------------------------------*/

/**
 * \brief usb_massstorage Application entry point.
 *
 * Configures UART,
 * Configures TC0, USB MSD Driver and run it.
 *
 * \return Unused (ANSI-C compatibility).
 */
int main(void)
{
	uint8_t key;
	sSdCard *pSd = 0;

	/* Disable watchdog */
	WDT_Disable(WDT);

	SCB_EnableICache();
	SCB_EnableDCache();

	printf("-- USB ISI MSD Example %s --\n\r", SOFTPACK_VERSION);
	printf("-- %s\n\r", BOARD_NAME);
	printf("-- Compiled: %s %s With %s--\n\r", __DATE__, __TIME__ ,
			COMPILER_NAME);

	/* If they are present, configure Vbus & Wake-up pins */
	PIO_InitializeInterrupts(0);

	/* Enable SDRAM for Frame buffer */
	BOARD_ConfigureSdram();

	/* Configure Time Tick */
	TimeTick_Configure();

	/* Initialize PIO pins */
	_ConfigurePIOs();

	/* Initialize drivers */
	_ConfigureDrivers();

	_MemoriesInitialize(pSd);

	/* BOT driver initialization */
	MSDDriver_Initialize(&msdDriverDescriptors, luns, MAX_LUNS);

	printf("Press [c|u] to select the function:\n\r");
	printf("- 'c' Capture image and save to BMP or JPEG \n\r");
	printf("- 'u' Act as an USB mass storage device to view files on SD card\n\r");

	for (;;) {
		key = DBG_GetChar();

		if (('c' == key) || ('C' == key)) {
			printf("Current only omnivision 7740 is supported!\n\r");

			if (0 != isi_init(3, &wImageWidth, &wImageHeight)) {
				printf("-E- Sensor setup failed.");

				while (1);
			} else  {
				printf("-I- Sensor setup OK!\r\n");
				printf(" Press 'b' or 'B' to capture image and save as BMP file.\n\r");
				printf(" Press 'j' or 'J' to capture image and save as JPEG file.\n\r");

				while (1) {
					key = DBG_GetChar();

					if (('b' == key) || ('B' == key))
						capture_image_to_bmp();
					else if (('j' == key) || ('J' == key))
						capture_image_to_jpeg();
					else {
					}

					printf("\n\r");
				}
			}
		} else {
			printf("Connect to PC to view the files on SD card!\n\r");
			break;
		}
	}

	/* Start USB stack to authorize VBus monitoring */
	VBus_Configure();

	while (1) {
		/* Mass storage state machine */
		if (USBD_GetState() < USBD_STATE_CONFIGURED) {}
		else {
			MSDDriver_StateMachine();

			if (msdRefresh) {
				msdRefresh = 0;

				if (msdWriteTotal < 50 * 1000) {
					/* Flush Disk Media */
				}

				msdWriteTotal = 0;
			}
		}
	}
}
/** \endcond */
