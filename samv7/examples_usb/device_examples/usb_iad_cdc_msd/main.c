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

/** \cond usb_iad_cdc_msd
 * \page usb_iad_cdc_msd USB CDC(Serial)+MSD Example
 *
 * \section Purpose
 *
 * The USB CDCMSD Project will help you to get familiar with the
 * USB Device Port(UDP)interface and also some of the other interfaces in
 * SAMV7/E7 Microcontrollers. Also it can help you to be familiar with the USB
 * Framework that is used for rapid development of USB-compliant class
 * drivers such as USB Communication Device class (CDC), and how to combine
 * two USB functions to a single CDCMSD device (such as CDC + MSD).
 *
 * \section Requirements
 *
 * This package can be used with SAMV71 Xplained Ultra board or SAME70 Xplained board
 * that have UDP interface, depending on the functions included.
 *
 *  \section win_drv_update Windows Driver Update
 *
 * The composite device is generally supported by Microsoft windows, but some
 * patches are needed for muti-interface functions such as CDC & Audio.
 *
 * \section Description
 *
 * The demo simulates a USB composite device that integrates USB CDC Serial
 * RS232 Converter function and USB Disk function.
 *
 * When the board running this program connected to a host (PC for example),
 * with USB cable, host will notice the attachment of a USB device. No device
 * driver offered for the device now.
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
 *     -- USB CDC MSD Device Project xxx --
 *     -- SAMxxxxx-xx
 *     -- Compiled: xxx xx xxxx xx:xx:xx --
 *     \endcode
 * -# When connecting USB cable to windows, the LED blinks, and the host
 *    reports a new USB device attachment.
 * -# For the windows driver installation and the test functions, please
 *      refer to "USB CDC serial converter" &
 *      "USB Device Mass Storage Project".
 * -# You can use the inf file
 *    libraries\\usb\\device\\composite\\drv\\CompositeCDCSerial.inf
 *    to install the CDC serial  port.
 *
 * \section References
 *
 * - usb_iad_cdc_msd/main.c
 * - pio: Pin configurations and peripheral configure.
 * - memories: Storage Media interface for MSD
 * - usb: USB Device Framework, USB CDC driver and UDP interface driver
 *    - \ref usbd_framework
 *        - \ref usbd_api
 *    - \ref usbd_composite "composite"
 *       - \ref usbd_composite_drv
 *    - \ref usbd_msd "massstorage"
 *       - \ref usbd_msd_drv
 *    - \ref usbd_cdc "cdc-serial"
 *       - \ref usbd_cdc_serial_drv
 * - projects:
 *    - \ref usb_massstorage
 *    - \ref usb_cdc_serial
 */

/**
 * \file
 *
 * \section Purpose
 *
 * This file contains all the specific code for the
 * usb_iad_cdc_msd project
 */

/*---------------------------------------------------------------------------
 *         Headers
 *---------------------------------------------------------------------------*/

#include "board.h"

#include "libstoragemedia.h"

#include <USBD_Config.h>
#include <CDCMSDDriver.h>
#include <CDCDSerial.h>
#include <MSDFunction.h>

#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <stdbool.h>

/*---------------------------------------------------------------------------
 *      Definitions
 *---------------------------------------------------------------------------*/

/** Size in bytes of the packet used for reading data from USB */
#define DATAPACKETSIZE \
	CHIP_USB_ENDPOINTS_MAXPACKETSIZE(CDCDSerialDriverDescriptors_DATAIN)

/** Size in bytes of the buffer used for reading data from the USB & USART */
#define DATABUFFERSIZE (DATAPACKETSIZE+2)

/** Maximum number of LUNs which can be defined. */
#define MAX_LUNS            1
/** Media index for different disks */
#define DRV_RAMDISK         0    /**< RAM disk */
#define DRV_SDMMC           1    /**< SD card */

/** Size of one block in bytes. */
#define BLOCK_SIZE          512

/** Size of the MSD IO buffer in bytes (6K, more the better). */
#define MSD_BUFFER_SIZE     (12*BLOCK_SIZE)

/** RamDisk size: (WinXP can not format the disk if lower than 20K) */
#define RAMDISK_SIZE        128*1024

COMPILER_SECTION("ramdisk_region") static uint8_t
ramdisk_reserved[RAMDISK_SIZE];
#define RAMDISK_BASE_ADDR ((uint32_t)ramdisk_reserved)

/** Delay for MSD refresh (*4ms) */
#define MSD_REFRESH_DELAY    250

/** Delay for waiting DBGU input (*4ms) */
#define INPUT_DELAY          (2*250)

/*---------------------------------------------------------------------------
 *      External variables
 *---------------------------------------------------------------------------*/

/** Descriptor list for the device to bring up */
extern const USBDDriverDescriptors cdcmsddDriverDescriptors;

/*---------------------------------------------------------------------------
 *      Internal variables
 *---------------------------------------------------------------------------*/
/** Buffer for storing incoming USB data. */
static uint8_t usbSerialBuffer0[DATABUFFERSIZE];

/** Serial port opened */
static uint8_t isSerialPortON = 0;



/*- MSD */
/** Available media. */
sMedia medias[MAX_LUNS];

/** Device LUNs. */
MSDLun luns[MAX_LUNS];

/** LUN read/write buffer. */
uint8_t msdBuffer[MSD_BUFFER_SIZE];

/** Total data write to disk */
uint32_t msdWriteTotal = 0;
/** Delay for data write refresh */
uint32_t msdDelay = MSD_REFRESH_DELAY;
/** Delay TO event */
uint8_t  msdRefresh = 0;


/*-----------------------------------------------------------------------------
 *         Callback re-implementation
 *-----------------------------------------------------------------------------*/
/**
 * Invoked when the configuration of the device changes. Parse used endpoints.
 * \param cfgnum New configuration number.
 */
void USBDDriverCallbacks_ConfigurationChanged(uint8_t cfgnum)
{
	CDCMSDDriver_ConfigurationChangedHandler(cfgnum);
}

/**
 * Invoked when a new SETUP request is received from the host. Forwards the
 * request to the Mass Storage device driver handler function.
 * \param request  Pointer to a USBGenericRequest instance.
 */
void USBDCallbacks_RequestReceived(const USBGenericRequest *request)
{
	CDCMSDDriver_RequestHandler(request);
}

/*----------------------------------------------------------------------------
 *         Callbacks
 *----------------------------------------------------------------------------*/

/**
 * The callback function of CDCDSerial_Read.
 * Call CDCDSerial_Read here to keep the device in receive state and this is 
 * essential for Linux host.
 */
static void _CDCDSerial_Read_Callback( void)
{

	CDCDSerial_Read(usbSerialBuffer0, DATAPACKETSIZE,(TransferCallback) _CDCDSerial_Read_Callback, 0);

}

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

/**
 * Invoked when the CDC ControlLineState is changed
 * \param DTR   New DTR value.
 * \param RTS   New RTS value.
 */
void CDCDSerial_ControlLineStateChanged(uint8_t DTR,
										uint8_t RTS)
{
	isSerialPortON = DTR;
	RTS = RTS; /* dummy */
}

/**
 * Invoked when the CDC LineCoding is requested to changed
 * \param pLineCoding   Pointer to new LineCoding settings.
 * \return USBRC_SUCCESS if ready to receive the line coding.
 */
uint8_t CDCDSerial_LineCodingIsToChange(CDCLineCoding *pLineCoding)
{
	pLineCoding = pLineCoding; /*dummy */
	return USBD_STATUS_SUCCESS;
}

/*---------------------------------------------------------------------------
 *         Internal functions
 *---------------------------------------------------------------------------*/
/**
 * Configure USBHS settings for USB device
 */

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
 * Initialize DDRAM to assign RamDisk block
 */
static void RamDiskInit(void)
{
	BOARD_ConfigureSdram();

	printf("RamDisk @ %x, size %d\n\r", (unsigned int)RAMDISK_BASE_ADDR, RAMDISK_SIZE);

	MEDRamDisk_Initialize(&(medias[DRV_RAMDISK]),
						  BLOCK_SIZE,
						  (RAMDISK_BASE_ADDR) / BLOCK_SIZE,
						  RAMDISK_SIZE / BLOCK_SIZE,
						  1);
	LUN_Init(&(luns[DRV_RAMDISK]),
			 &(medias[DRV_RAMDISK]),
			 msdBuffer, MSD_BUFFER_SIZE,
			 0, 0, 0, 0,
			 MSDCallbacks_Data);

	gNbMedias = 1;
}

/**
 * Initialize MSD Media & LUNs
 */
static void _MemoriesInitialize(void)
{
	uint32_t i;

	/* Reset all LUNs */
	for (i = 0; i < MAX_LUNS; i ++)
		LUN_Init(&luns[i], 0, 0, 0, 0, 0, 0, 0, 0);

	/* TODO: Add LUN Init here */

	/* RAM disk initialize */
	RamDiskInit();
	/* Nand Flash Init */
	/* SD Card Init */
}

/*---------------------------------------------------------------------------
 *         Exported function
 *---------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------
 *          Main
 *---------------------------------------------------------------------------*/

/**
 * Initializes drivers and start the USB CDCMSD device.
 */
int main(void)
{
	uint8_t usbConnected = 0, serialON = 0;

	/* Disable watchdog */
	WDT_Disable(WDT);

	SCB_EnableICache();
	SCB_EnableDCache();

	printf("-- USB CDCMSD Device Project %s --\n\r", SOFTPACK_VERSION);
	printf("-- %s\n\r", BOARD_NAME);
	printf("-- Compiled: %s %s With %s--\n\r", __DATE__, __TIME__ ,
			COMPILER_NAME);

	/* If they are present, configure Vbus & Wake-up pins */
	PIO_InitializeInterrupts(0);

	/* ----- MSD Function Initialize */
	/* Configure memories */
	_MemoriesInitialize();

	/* USB CDCMSD driver initialization */
	CDCMSDDriver_Initialize(&cdcmsddDriverDescriptors, luns, MAX_LUNS);

	/* Start USB stack to authorize VBus monitoring */
	VBus_Configure();

	/* Driver loop */
	while (1) {
		/* Device is not configured */
		if (USBD_GetState() < USBD_STATE_CONFIGURED) {
			if (usbConnected) {
				printf("-I- USB Disconnect/Suspend\n\r");
				usbConnected = 0;

				/* Serial port closed */
				isSerialPortON = 0;
			}
		} else {
			if (usbConnected == 0) {
				printf("-I- USB Connect\n\r");
				usbConnected = 1;
			}

			if (!serialON && isSerialPortON) {
				printf("-I- SerialPort ON\n\r");
				/* Start receiving data on the USART */
				/* Start receiving data on the USB */
				CDCDSerial_Read(usbSerialBuffer0, DATAPACKETSIZE,(TransferCallback) _CDCDSerial_Read_Callback, 0);
				serialON = 1;
			}else if(serialON && !isSerialPortON){
				memory_sync();
				/* Remove the data in the banks when the port has been closed on the host */
				while (!USBHS_IsBankFree(USBHS, CDCD_Descriptors_DATAIN0)) {
					USBHS_KillBank(USBHS, CDCD_Descriptors_DATAIN0);
					while (USBHS_IsBankKilled(USBHS, CDCD_Descriptors_DATAIN0));
				}

				printf("-I- SeriaoPort OFF\n\r");
				serialON = 0;
			}

			MSDFunction_StateMachine();

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
