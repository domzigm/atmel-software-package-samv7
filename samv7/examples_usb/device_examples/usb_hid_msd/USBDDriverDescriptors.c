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

/** \addtogroup usbd_composite_hidmsd
 *@{
 */

/*----------------------------------------------------------------------------
 *      Headers
 *----------------------------------------------------------------------------*/

#include <USBD_Config.h>

#include <HIDMSDDriver.h>
#include <HIDDKeyboard.h>
#include <MSDFunction.h>

/*---------------------------------------------------------------------------
 *         Definitions
 *---------------------------------------------------------------------------*/

/** Device product ID. */
#define HIDMSDDDriverDescriptors_PRODUCTID       USBD_PID_HIDMSD
/** Device vendor ID (Atmel). */
#define HIDMSDDDriverDescriptors_VENDORID        USBD_VID_ATMEL
/** Device release number. */
#define HIDMSDDDriverDescriptors_RELEASE         0x0003


/*---------------------------------------------------------------------------
 *         Macros
 *---------------------------------------------------------------------------*/

/** Returns the minimum between two values. */
#define MIN(a, b)       ((a < b) ? a : b)

/*---------------------------------------------------------------------------
 *         Internal structures
 *---------------------------------------------------------------------------*/


/*----------------------------------------------------------------------------
 *         Internal variables
 *----------------------------------------------------------------------------*/

/** Standard USB device descriptor for the composite device driver */
static const USBDeviceDescriptor deviceDescriptor = {

	sizeof(USBDeviceDescriptor),
	USBGenericDescriptor_DEVICE,
	USBDeviceDescriptor_USB2_00,
	0x00,
	0x00,
	0x00,
	CHIP_USB_ENDPOINTS_MAXPACKETSIZE(0),
	HIDMSDDDriverDescriptors_VENDORID,
	HIDMSDDDriverDescriptors_PRODUCTID,
	HIDMSDDDriverDescriptors_RELEASE,
	1, /* No string descriptor for manufacturer */
	2, /* Index of product string descriptor is #1 */
	3, /* No string descriptor for serial number */
	1 /* Device has 1 possible configuration */
};

/** Device qualifier descriptor (Necessary to pass USB test). */
static const USBDeviceQualifierDescriptor qualifierDescriptor = {

	sizeof(USBDeviceQualifierDescriptor),
	USBGenericDescriptor_DEVICEQUALIFIER,
	USBDeviceDescriptor_USB2_00,
	0x00,
	0x00,
	0x00,
	CHIP_USB_ENDPOINTS_MAXPACKETSIZE(0),
	0, // Device has one possible configuration.
	0x00
};

/** USB FS configuration descriptors for the composite device driver */
static const HidMsdDriverConfigurationDescriptors configurationDescriptorsFS = {

	/* Standard configuration descriptor */
	{
		sizeof(USBConfigurationDescriptor),
		USBGenericDescriptor_CONFIGURATION,
		sizeof(HidMsdDriverConfigurationDescriptors),
		HIDMSDDriverDescriptors_NUMINTERFACE,
		1, /* This is configuration #1 */
		0, /* No string descriptor for this configuration */
		USBD_BMATTRIBUTES,
		USBConfigurationDescriptor_POWER(100)
	},

	/* Interface descriptor */
	{
		sizeof(USBInterfaceDescriptor),
		USBGenericDescriptor_INTERFACE,
		HIDMSDDriverDescriptors_HID_INTERFACE,
		0, /* This is alternate setting #0 */
		2, /* Two endpoints used */
		HIDInterfaceDescriptor_CLASS,
		HIDInterfaceDescriptor_SUBCLASS_NONE,
		HIDInterfaceDescriptor_PROTOCOL_NONE,
		0  /* No associated string descriptor */
	},
	/* HID descriptor */
	{
		sizeof(HIDDescriptor1),
		HIDGenericDescriptor_HID,
		HIDDescriptor_HID1_11,
		0, /* Device is not localized, no country code */
		1, /* One HID-specific descriptor (apart from this one) */
		HIDGenericDescriptor_REPORT,
		{HIDDKeyboard_REPORTDESCRIPTORSIZE}
	},
	/* Interrupt IN endpoint descriptor */
	{
		sizeof(USBEndpointDescriptor),
		USBGenericDescriptor_ENDPOINT,
		USBEndpointDescriptor_ADDRESS(
			USBEndpointDescriptor_IN,
			HIDMSDD_Descriptors_INTERRUPTIN),
		USBEndpointDescriptor_INTERRUPT,
		sizeof(HIDDKeyboardInputReport),
		HIDDKeyboardDescriptors_INTERRUPTIN_POLLING_FS
	},
	/* Interrupt OUT endpoint descriptor */
	{
		sizeof(USBEndpointDescriptor),
		USBGenericDescriptor_ENDPOINT,
		USBEndpointDescriptor_ADDRESS(
			USBEndpointDescriptor_OUT,
			HIDMSDD_Descriptors_INTERRUPTOUT),
		USBEndpointDescriptor_INTERRUPT,
		sizeof(HIDDKeyboardOutputReport),
		HIDDKeyboardDescriptors_INTERRUPTIN_POLLING_FS
	},

	/* Mass Storage interface descriptor. */
	{
		sizeof(USBInterfaceDescriptor),
		USBGenericDescriptor_INTERFACE,
		HIDMSDDriverDescriptors_MSD_INTERFACE,
		0, /* This is alternate setting #0. */
		2, /* Interface uses two endpoints. */
		MSInterfaceDescriptor_CLASS,
		MSInterfaceDescriptor_SCSI,
		MSInterfaceDescriptor_BULKONLY,
		0 /* No string descriptor for interface. */
	},
	/* Bulk-OUT endpoint descriptor */
	{
		sizeof(USBEndpointDescriptor),
		USBGenericDescriptor_ENDPOINT,
		USBEndpointDescriptor_ADDRESS(
			USBEndpointDescriptor_OUT,
			HIDMSDD_Descriptors_BULKOUT),
		USBEndpointDescriptor_BULK,
		MIN(CHIP_USB_ENDPOINTS_MAXPACKETSIZE(HIDMSDD_Descriptors_BULKOUT),
		USBEndpointDescriptor_MAXBULKSIZE_FS),
		0 /* No string descriptor for endpoint. */
	},
	/* Bulk-IN endpoint descriptor */
	{
		sizeof(USBEndpointDescriptor),
		USBGenericDescriptor_ENDPOINT,
		USBEndpointDescriptor_ADDRESS(
			USBEndpointDescriptor_IN,
			HIDMSDD_Descriptors_BULKIN),
		USBEndpointDescriptor_BULK,
		MIN(CHIP_USB_ENDPOINTS_MAXPACKETSIZE(HIDMSDD_Descriptors_BULKIN),
		USBEndpointDescriptor_MAXBULKSIZE_FS),
		0 /* No string descriptor for endpoint. */
	}

};


/** USB FS configuration descriptors for the composite device driver */
static const HidMsdDriverConfigurationDescriptors configurationDescriptorsHS = {

	/* Standard configuration descriptor */
	{
		sizeof(USBConfigurationDescriptor),
		USBGenericDescriptor_CONFIGURATION,
		sizeof(HidMsdDriverConfigurationDescriptors),
		HIDMSDDriverDescriptors_NUMINTERFACE,
		1, /* This is configuration #1 */
		0, /* No string descriptor for this configuration */
		USBD_BMATTRIBUTES,
		USBConfigurationDescriptor_POWER(100)
	},

	/* Interface descriptor */
	{
		sizeof(USBInterfaceDescriptor),
		USBGenericDescriptor_INTERFACE,
		HIDMSDDriverDescriptors_HID_INTERFACE,
		0, /* This is alternate setting #0 */
		2, /* Two endpoints used */
		HIDInterfaceDescriptor_CLASS,
		HIDInterfaceDescriptor_SUBCLASS_NONE,
		HIDInterfaceDescriptor_PROTOCOL_NONE,
		0  /* No associated string descriptor */
	},
	/* HID descriptor */
	{
		sizeof(HIDDescriptor1),
		HIDGenericDescriptor_HID,
		HIDDescriptor_HID1_11,
		0, /* Device is not localized, no country code */
		1, /* One HID-specific descriptor (apart from this one) */
		HIDGenericDescriptor_REPORT,
		{HIDDKeyboard_REPORTDESCRIPTORSIZE}
	},
	/* Interrupt IN endpoint descriptor */
	{
		sizeof(USBEndpointDescriptor),
		USBGenericDescriptor_ENDPOINT,
		USBEndpointDescriptor_ADDRESS(
			USBEndpointDescriptor_IN,
			HIDMSDD_Descriptors_INTERRUPTIN),
		USBEndpointDescriptor_INTERRUPT,
		sizeof(HIDDKeyboardInputReport),
		HIDDKeyboardDescriptors_INTERRUPTIN_POLLING_FS
	},
	/* Interrupt OUT endpoint descriptor */
	{
		sizeof(USBEndpointDescriptor),
		USBGenericDescriptor_ENDPOINT,
		USBEndpointDescriptor_ADDRESS(
			USBEndpointDescriptor_OUT,
			HIDMSDD_Descriptors_INTERRUPTOUT),
		USBEndpointDescriptor_INTERRUPT,
		sizeof(HIDDKeyboardOutputReport),
		HIDDKeyboardDescriptors_INTERRUPTIN_POLLING_FS
	},

	/* Mass Storage interface descriptor. */
	{
		sizeof(USBInterfaceDescriptor),
		USBGenericDescriptor_INTERFACE,
		HIDMSDDriverDescriptors_MSD_INTERFACE,
		0, /* This is alternate setting #0. */
		2, /* Interface uses two endpoints. */
		MSInterfaceDescriptor_CLASS,
		MSInterfaceDescriptor_SCSI,
		MSInterfaceDescriptor_BULKONLY,
		0 /* No string descriptor for interface. */
	},
	/* Bulk-OUT endpoint descriptor */
	{
		sizeof(USBEndpointDescriptor),
		USBGenericDescriptor_ENDPOINT,
		USBEndpointDescriptor_ADDRESS(
			USBEndpointDescriptor_OUT,
			HIDMSDD_Descriptors_BULKOUT),
		USBEndpointDescriptor_BULK,
		MIN(CHIP_USB_ENDPOINTS_MAXPACKETSIZE(HIDMSDD_Descriptors_BULKOUT),
		USBEndpointDescriptor_MAXBULKSIZE_HS),
		0 /* No string descriptor for endpoint. */
	},
	/* Bulk-IN endpoint descriptor */
	{
		sizeof(USBEndpointDescriptor),
		USBGenericDescriptor_ENDPOINT,
		USBEndpointDescriptor_ADDRESS(
			USBEndpointDescriptor_IN,
			HIDMSDD_Descriptors_BULKIN),
		USBEndpointDescriptor_BULK,
		MIN(CHIP_USB_ENDPOINTS_MAXPACKETSIZE(HIDMSDD_Descriptors_BULKIN),
		USBEndpointDescriptor_MAXBULKSIZE_HS),
		0 /* No string descriptor for endpoint. */
	}

};

/** String descriptor with the supported languages. */
static const unsigned char languageIdDescriptor[] = {

	USBStringDescriptor_LENGTH(1),
	USBGenericDescriptor_STRING,
	USBStringDescriptor_ENGLISH_US
};

/** Manufacturer name. */
static const unsigned char manufacturerDescriptor[] = {

	USBStringDescriptor_LENGTH(5),
	USBGenericDescriptor_STRING,
	USBStringDescriptor_UNICODE('A'),
	USBStringDescriptor_UNICODE('t'),
	USBStringDescriptor_UNICODE('m'),
	USBStringDescriptor_UNICODE('e'),
	USBStringDescriptor_UNICODE('l')
};

/** Product name. */
static const unsigned char productDescriptor[] = {

	USBStringDescriptor_LENGTH(14),
	USBGenericDescriptor_STRING,
	USBStringDescriptor_UNICODE('C'),
	USBStringDescriptor_UNICODE('o'),
	USBStringDescriptor_UNICODE('m'),
	USBStringDescriptor_UNICODE('p'),
	USBStringDescriptor_UNICODE('o'),
	USBStringDescriptor_UNICODE('s'),
	USBStringDescriptor_UNICODE('i'),
	USBStringDescriptor_UNICODE('t'),
	USBStringDescriptor_UNICODE('e'),
	USBStringDescriptor_UNICODE(' '),
	USBStringDescriptor_UNICODE('D'),
	USBStringDescriptor_UNICODE('e'),
	USBStringDescriptor_UNICODE('m'),
	USBStringDescriptor_UNICODE('o')
};

/** Product serial number. */
static const unsigned char serialNumberDescriptor[] = {

	USBStringDescriptor_LENGTH(12),
	USBGenericDescriptor_STRING,
	USBStringDescriptor_UNICODE('0'),
	USBStringDescriptor_UNICODE('1'),
	USBStringDescriptor_UNICODE('2'),
	USBStringDescriptor_UNICODE('3'),
	USBStringDescriptor_UNICODE('4'),
	USBStringDescriptor_UNICODE('5'),
	USBStringDescriptor_UNICODE('6'),
	USBStringDescriptor_UNICODE('7'),
	USBStringDescriptor_UNICODE('8'),
	USBStringDescriptor_UNICODE('9'),
	USBStringDescriptor_UNICODE('A'),
	USBStringDescriptor_UNICODE('B')
};

/** Array of pointers to the four string descriptors. */
static const unsigned char *stringDescriptors[] = {

	languageIdDescriptor,
	manufacturerDescriptor,
	productDescriptor,
	serialNumberDescriptor,
};

/*----------------------------------------------------------------------------
 *         Exported variables
 *----------------------------------------------------------------------------*/

/** List of descriptors required by an USB audio speaker device driver. */
const USBDDriverDescriptors hidmsddDriverDescriptors = {

	&deviceDescriptor,
	(const USBConfigurationDescriptor *) &configurationDescriptorsFS,
	&qualifierDescriptor,
	0, 0,
	(const USBConfigurationDescriptor *) &configurationDescriptorsHS,
	&qualifierDescriptor,
	0,
	stringDescriptors,
	4 /* Number of string descriptors */
};
/**@}*/

