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


/** \file
 * \addtogroup usbd_eem
 *@{
 */

/*------------------------------------------------------------------------------
 *         Headers
 *------------------------------------------------------------------------------*/

#include "board.h"
#include "USBD_Config.h"
#include "CDCDEEMDriver.h"

/*------------------------------------------------------------------------------
 *         Definitions
 *------------------------------------------------------------------------------*/

/** \addtogroup usbd_cdc_serial_device_ids CDC EEM Device IDs
 *      @{
 * This page lists the IDs used in the CDC EEM Device Descriptor.
 *
 * \section IDs
 * - CDCDEEMDriverDescriptors_PRODUCTID
 * - CDCDEEMDriverDescriptors_VENDORID
 * - CDCDEEMDriverDescriptors_RELEASE
 */

/** Device product ID. */
#define CDCDEEMDriverDescriptors_PRODUCTID       USBD_PID_CDCEEM
/** Device vendor ID (Atmel). */
#define CDCDEEMDriverDescriptors_VENDORID        USBD_VID_ATMEL
/** Device release number. */
#define CDCDEEMDriverDescriptors_RELEASE         USBD_RELEASE_1_00
/**      @}*/

/*------------------------------------------------------------------------------
 *         Macros
 *------------------------------------------------------------------------------*/

/** Returns the minimum between two values. */
#define MIN(a, b)       ((a < b) ? a : b)

/*------------------------------------------------------------------------------
 *         Exported variables
 *------------------------------------------------------------------------------*/

/** Standard USB device descriptor for the CDC serial driver */
const USBDeviceDescriptor deviceDescriptor = {

	sizeof(USBDeviceDescriptor),
	USBGenericDescriptor_DEVICE,
	USBDeviceDescriptor_USB2_00,
	CDCDeviceDescriptor_CLASS,
	CDCDeviceDescriptor_SUBCLASS,
	CDCDeviceDescriptor_PROTOCOL,
	CHIP_USB_ENDPOINTS_MAXPACKETSIZE(0),
	CDCDEEMDriverDescriptors_VENDORID,
	CDCDEEMDriverDescriptors_PRODUCTID,
	CDCDEEMDriverDescriptors_RELEASE,
	0, /* No string descriptor for manufacturer */
	1, /* Index of product string descriptor is #1 */
	0, /* No string descriptor for serial number */
	1 /* Device has 1 possible configuration */
};

/** Standard USB configuration descriptor for the CDC serial driver */
const CDCDEEMDriverConfigurationDescriptors configurationDescriptorsFS = {

	/* Standard configuration descriptor */
	{
		sizeof(USBConfigurationDescriptor),
		USBGenericDescriptor_CONFIGURATION,
		sizeof(CDCDEEMDriverConfigurationDescriptors),
		1, /* There is one interface in this configuration */
		1, /* This is configuration #1 */
		0, /* No string descriptor for this configuration */
		USBD_BMATTRIBUTES,
		USBConfigurationDescriptor_POWER(100)
	},
	/* Communication class interface standard descriptor */
	{
		sizeof(USBInterfaceDescriptor),
		USBGenericDescriptor_INTERFACE,
		0, /* This is interface #0 */
		0, /* This is alternate setting #0 for this interface */
		2, /* This interface uses 2 endpoints */
		CDCCommunicationInterfaceDescriptor_CLASS,
		CDCCommunicationInterfaceDescriptor_ETHERNETEMULATIONMODEL,
		CDCCommunicationInterfaceDescriptor_EEMPROTOCOL,
		0  /* No string descriptor for this interface */
	},
	/* Bulk-OUT endpoint standard descriptor */
	{
		sizeof(USBEndpointDescriptor),
		USBGenericDescriptor_ENDPOINT,
		USBEndpointDescriptor_ADDRESS(USBEndpointDescriptor_OUT,
		CDCDEEMDriverDescriptors_BULKOUT),
		USBEndpointDescriptor_BULK,
		MIN(CHIP_USB_ENDPOINTS_MAXPACKETSIZE(CDCDEEMDriverDescriptors_BULKOUT),
		USBEndpointDescriptor_MAXBULKSIZE_FS),
		0 /* Must be 0 for full-speed bulk endpoints */
	},
	/* Bulk-IN endpoint descriptor */
	{
		sizeof(USBEndpointDescriptor),
		USBGenericDescriptor_ENDPOINT,
		USBEndpointDescriptor_ADDRESS(USBEndpointDescriptor_IN,
		CDCDEEMDriverDescriptors_BULKIN),
		USBEndpointDescriptor_BULK,
		MIN(CHIP_USB_ENDPOINTS_MAXPACKETSIZE(CDCDEEMDriverDescriptors_BULKIN),
		USBEndpointDescriptor_MAXBULKSIZE_FS),
		0 /* Must be 0 for full-speed bulk endpoints */
	}
};

/** Other-speed configuration descriptor (when in full-speed). */
const CDCDEEMDriverConfigurationDescriptors otherSpeedDescriptorsFS = {

	/* Standard configuration descriptor */
	{
		sizeof(USBConfigurationDescriptor),
		USBGenericDescriptor_OTHERSPEEDCONFIGURATION,
		sizeof(CDCDEEMDriverConfigurationDescriptors),
		1, /* There is one interface in this configuration */
		1, /* This is configuration #1 */
		0, /* No string descriptor for this configuration */
		BOARD_USB_BMATTRIBUTES,
		USBConfigurationDescriptor_POWER(100)
	},
	/* Communication class interface standard descriptor */
	{
		sizeof(USBInterfaceDescriptor),
		USBGenericDescriptor_INTERFACE,
		0, /* This is interface #0 */
		0, /* This is alternate setting #0 for this interface */
		2, /* This interface uses 2 endpoints */
		CDCCommunicationInterfaceDescriptor_CLASS,
		CDCCommunicationInterfaceDescriptor_ETHERNETEMULATIONMODEL,
		CDCCommunicationInterfaceDescriptor_EEMPROTOCOL,
		0  /* No string descriptor for this interface */
	},
	/* Bulk-OUT endpoint standard descriptor */
	{
		sizeof(USBEndpointDescriptor),
		USBGenericDescriptor_ENDPOINT,
		USBEndpointDescriptor_ADDRESS(USBEndpointDescriptor_OUT,
		CDCDEEMDriverDescriptors_BULKOUT),
		USBEndpointDescriptor_BULK,
		MIN(CHIP_USB_ENDPOINTS_MAXPACKETSIZE(CDCDEEMDriverDescriptors_BULKOUT),
		USBEndpointDescriptor_MAXBULKSIZE_HS),
		0 /* Must be 0 for full-speed bulk endpoints */
	},
	/* Bulk-IN endpoint descriptor */
	{
		sizeof(USBEndpointDescriptor),
		USBGenericDescriptor_ENDPOINT,
		USBEndpointDescriptor_ADDRESS(USBEndpointDescriptor_IN,
		CDCDEEMDriverDescriptors_BULKIN),
		USBEndpointDescriptor_BULK,
		MIN(CHIP_USB_ENDPOINTS_MAXPACKETSIZE(CDCDEEMDriverDescriptors_BULKIN),
		USBEndpointDescriptor_MAXBULKSIZE_HS),
		0 /* Must be 0 for full-speed bulk endpoints */
	}
};

/** Configuration descriptor (when in high-speed). */
const CDCDEEMDriverConfigurationDescriptors configurationDescriptorsHS = {

	/* Standard configuration descriptor */
	{
		sizeof(USBConfigurationDescriptor),
		USBGenericDescriptor_CONFIGURATION,
		sizeof(CDCDEEMDriverConfigurationDescriptors),
		1, /* There is one interface in this configuration */
		1, /* This is configuration #1 */
		0, /* No string descriptor for this configuration */
		BOARD_USB_BMATTRIBUTES,
		USBConfigurationDescriptor_POWER(100)
	},
	/* Communication class interface standard descriptor */
	{
		sizeof(USBInterfaceDescriptor),
		USBGenericDescriptor_INTERFACE,
		0, /* This is interface #0 */
		0, /* This is alternate setting #0 for this interface */
		2, /* This interface uses 2 endpoints */
		CDCCommunicationInterfaceDescriptor_CLASS,
		CDCCommunicationInterfaceDescriptor_ETHERNETEMULATIONMODEL,
		CDCCommunicationInterfaceDescriptor_EEMPROTOCOL,
		0  /* No string descriptor for this interface */
	},
	/* Bulk-OUT endpoint standard descriptor */
	{
		sizeof(USBEndpointDescriptor),
		USBGenericDescriptor_ENDPOINT,
		USBEndpointDescriptor_ADDRESS(USBEndpointDescriptor_OUT,
		CDCDEEMDriverDescriptors_BULKOUT),
		USBEndpointDescriptor_BULK,
		MIN(CHIP_USB_ENDPOINTS_MAXPACKETSIZE(CDCDEEMDriverDescriptors_BULKOUT),
		USBEndpointDescriptor_MAXBULKSIZE_HS),
		0 /* Must be 0 for full-speed bulk endpoints */
	},
	/* Bulk-IN endpoint descriptor */
	{
		sizeof(USBEndpointDescriptor),
		USBGenericDescriptor_ENDPOINT,
		USBEndpointDescriptor_ADDRESS(USBEndpointDescriptor_IN,
		CDCDEEMDriverDescriptors_BULKIN),
		USBEndpointDescriptor_BULK,
		MIN(CHIP_USB_ENDPOINTS_MAXPACKETSIZE(CDCDEEMDriverDescriptors_BULKIN),
		USBEndpointDescriptor_MAXBULKSIZE_HS),
		0 /* Must be 0 for full-speed bulk endpoints */
	}
};

/** Other-speed configuration descriptor (when in high-speed). */
const CDCDEEMDriverConfigurationDescriptors otherSpeedDescriptorsHS = {

	/* Standard configuration descriptor */
	{
		sizeof(USBConfigurationDescriptor),
		USBGenericDescriptor_OTHERSPEEDCONFIGURATION,
		sizeof(CDCDEEMDriverConfigurationDescriptors),
		1, /* There is one interface in this configuration */
		1, /* This is configuration #1 */
		0, /* No string descriptor for this configuration */
		BOARD_USB_BMATTRIBUTES,
		USBConfigurationDescriptor_POWER(100)
	},
	/* Communication class interface standard descriptor */
	{
		sizeof(USBInterfaceDescriptor),
		USBGenericDescriptor_INTERFACE,
		0, /* This is interface #0 */
		0, /* This is alternate setting #0 for this interface */
		2, /* This interface uses 2 endpoints */
		CDCCommunicationInterfaceDescriptor_CLASS,
		CDCCommunicationInterfaceDescriptor_ETHERNETEMULATIONMODEL,
		CDCCommunicationInterfaceDescriptor_EEMPROTOCOL,
		0  /* No string descriptor for this interface */
	},
	/* Bulk-OUT endpoint standard descriptor */
	{
		sizeof(USBEndpointDescriptor),
		USBGenericDescriptor_ENDPOINT,
		USBEndpointDescriptor_ADDRESS(USBEndpointDescriptor_OUT,
		CDCDEEMDriverDescriptors_BULKOUT),
		USBEndpointDescriptor_BULK,
		MIN(CHIP_USB_ENDPOINTS_MAXPACKETSIZE(CDCDEEMDriverDescriptors_BULKOUT),
		USBEndpointDescriptor_MAXBULKSIZE_FS),
		0 /* Must be 0 for full-speed bulk endpoints */
	},
	/* Bulk-IN endpoint descriptor */
	{
		sizeof(USBEndpointDescriptor),
		USBGenericDescriptor_ENDPOINT,
		USBEndpointDescriptor_ADDRESS(USBEndpointDescriptor_IN,
		CDCDEEMDriverDescriptors_BULKIN),
		USBEndpointDescriptor_BULK,
		MIN(CHIP_USB_ENDPOINTS_MAXPACKETSIZE(CDCDEEMDriverDescriptors_BULKIN),
		USBEndpointDescriptor_MAXBULKSIZE_FS),
		0 /* Must be 0 for full-speed bulk endpoints */
	}
};

/** Language ID string descriptor */
const unsigned char languageIdStringDescriptor[] = {

	USBStringDescriptor_LENGTH(1),
	USBGenericDescriptor_STRING,
	USBStringDescriptor_ENGLISH_US
};

/** Product string descriptor */
const unsigned char productStringDescriptor[] = {

	USBStringDescriptor_LENGTH(12),
	USBGenericDescriptor_STRING,
	USBStringDescriptor_UNICODE('A'),
	USBStringDescriptor_UNICODE('T'),
	USBStringDescriptor_UNICODE('9'),
	USBStringDescriptor_UNICODE('1'),
	USBStringDescriptor_UNICODE('E'),
	USBStringDescriptor_UNICODE('t'),
	USBStringDescriptor_UNICODE('h'),
	USBStringDescriptor_UNICODE('e'),
	USBStringDescriptor_UNICODE('r'),
	USBStringDescriptor_UNICODE('n'),
	USBStringDescriptor_UNICODE('e'),
	USBStringDescriptor_UNICODE('t')
};

/** List of string descriptors used by the device */
const unsigned char *stringDescriptors[] = {

	languageIdStringDescriptor,
	productStringDescriptor,
};

/** List of standard descriptors for the serial driver. */
WEAK const USBDDriverDescriptors cdcdEEMDriverDescriptors = {

	&deviceDescriptor,
	(USBConfigurationDescriptor *) &(configurationDescriptorsFS),
	0, /* No full-speed device qualifier descriptor */
	(USBConfigurationDescriptor *) &(otherSpeedDescriptorsFS),
	0, /* No high-speed device descriptor (uses FS one) */
	(USBConfigurationDescriptor *) &(configurationDescriptorsHS),
	0, /* No high-speed device qualifier descriptor */
	(USBConfigurationDescriptor *) &(otherSpeedDescriptorsHS),
	stringDescriptors,
	2 /* 2 string descriptors in list */
};

/**@}*/
