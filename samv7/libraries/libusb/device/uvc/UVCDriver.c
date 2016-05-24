/* ----------------------------------------------------------------------------
 *         SAM Software Package License
 * ----------------------------------------------------------------------------
 * Copyright (c) 2015, Atmel Corporation
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * - Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the disclaimer below.
 *
 * Atmel's name may not be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * DISCLAIMER: THIS SOFTWARE IS PROVIDED BY ATMEL "AS IS" AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT ARE
 * DISCLAIMED. IN NO EVENT SHALL ATMEL BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
 * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * ----------------------------------------------------------------------------
 */

/*------------------------------------------------------------------------------
 *      Includes
 *------------------------------------------------------------------------------*/

#include <UVCDriver.h>

#include <UVCFunction.h>

/*-----------------------------------------------------------------------------
 *         Internal variables
 *-----------------------------------------------------------------------------*/

/** Static instance of the UVC device driver. */
static struct _uvc_driver uvc_driver;
/*-----------------------------------------------------------------------------
 *      Exported functions
 *-----------------------------------------------------------------------------*/

void UVCDriver_Init(const USBDDriverDescriptors *descriptors, uint32_t *buff_addr)
{
	USBDDriver *pUsbd = USBD_GetDriver();

	/*Initial UVC driver instance */
	uvc_driver.frm_index = 0;
	uvc_driver.is_frame_xfring = 0;
	uvc_driver.frm_count = 0;
	uvc_driver.buf = buff_addr;
	uvc_driver.frm_width = VIDEO_WIDTH;
	uvc_driver.frm_height = VIDEO_HEIGHT;

	/* Function instance initialize */
	UVCFunc_Init(&uvc_driver);

	/* USB Driver Initialize */
	USBDDriver_Initialize(pUsbd, descriptors, uvc_driver.alternate_interfaces);

	/* Initialize the USB driver */
	USBD_Init();
}

/**
 * \brief Generic ISI & OV sensor initialization
 */
void _PreviewMode(uint32_t hSize,uint32_t vSize)
{
	ISI_SetSensorSize(hSize, vSize);
	ISI_setPreviewSize(hSize, vSize);
	/* calculate scaler factor automatically. */
	ISI_calcScalerFactor();
	ISI_DisableInterrupt(0xFFFFFFFF);
	ISI_DmaChannelDisable(ISI_DMA_CHDR_C_CH_DIS);
	ISI_DmaChannelEnable(ISI_DMA_CHER_P_CH_EN);
	ISI_EnableInterrupt(ISI_IER_PXFR_DONE);
	/* Configure ISI interrupts */
	NVIC_ClearPendingIRQ(ISI_IRQn);
	NVIC_EnableIRQ(ISI_IRQn);
	ISI_Enable();
}




/**
 *  Invoked whenever a SETUP request is received from the host. Forwards the
 *  request to the standard handler.
 */
void USBDCallbacks_RequestReceived(const USBGenericRequest *request)
{
	USBDDriver *pUsbd = USBD_GetDriver();

	/* STD requests */
	if (USBGenericRequest_GetType(request) != USBGenericRequest_CLASS) {
		USBDDriver_RequestHandler(pUsbd, request);
		return;
	}

	/* Video requests */
	TRACE_INFO_WP("Vid ");

	switch (USBGenericRequest_GetRequest(request)) {
	case VIDGenericRequest_SETCUR:  VIDD_SetCUR (request);  break;

	case VIDGenericRequest_GETCUR:  VIDD_GetCUR (request);  break;

	case VIDGenericRequest_GETDEF:  VIDD_GetDEF (request);  break;

	case VIDGenericRequest_GETINFO: VIDD_GetINFO(request);  break;

	case VIDGenericRequest_GETMIN:  VIDD_GetMIN (request);  break;

	case VIDGenericRequest_GETMAX:  VIDD_GetMAX (request);  break;

	case VIDGenericRequest_GETRES:  VIDD_GetRES (request);  break;

	default:
		TRACE_WARNING("REQ: %x %x %x %x\n\r",
					  USBGenericRequest_GetType(request),
					  USBGenericRequest_GetRequest(request),
					  USBGenericRequest_GetValue(request),
					  USBGenericRequest_GetLength(request));
		USBD_Stall(0);
	}

	TRACE_INFO_WP("\n\r");
}

/**
 * Invoked whenever the active setting of an interface is changed by the
 * host. Reset streaming interface.
 * \param interface Interface number.
 * \param setting Newly active setting.
 */
void USBDDriverCallbacks_InterfaceSettingChanged(uint8_t interface,
		uint8_t setting)
{
	if (interface != VIDCAMD_StreamInterfaceNum) return;

	if (setting) {
		uvc_driver.is_video_on = 1;
		uvc_driver.frm_count = 0;
		uvc_driver.frm_index = 0;
	} else {
		uvc_driver.is_video_on = 0;
		uvc_driver.is_frame_xfring = 0;
	}

	memory_sync();
	USBD_HAL_ResetEPs(1 << VIDCAMD_IsoInEndpointNum, USBRC_CANCELED, 1);
}

/**
 * Get the address of UVC device driver instance
 */
struct _uvc_driver* UVC_get_driver(void)
{
	return (struct _uvc_driver *)&uvc_driver;
}

/**
 * Get the status of the USB video device
 */
uint8_t UVC_is_video_on(void)
{
	return uvc_driver.is_video_on;
}

/**
 * Get the frame width 
 */
uint32_t UVC_frm_width(void)
{
	return uvc_driver.frm_width;
}

/**
 * Get the frame height 
 */
uint32_t UVC_frm_height(void)
{
	return uvc_driver.frm_height;
}

/**
 * Get the current index of DMA descriptor 
 */
uint32_t UVC_get_frm_index(void)

{
	return uvc_driver.frm_index;
}

/**
 * Set the index of DMA descriptor 
 */
void UVC_set_frm_index(uint32_t value)

{
	uvc_driver.frm_index = value;
	return;
}

/**@}*/
