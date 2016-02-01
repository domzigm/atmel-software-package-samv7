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
#include <UVCFunction.h>

/** Probe & Commit Controls */

static USBVideoProbeData viddProbeData = {
	0, /* bmHint: All parameters fixed: sent by host */
	0x01,   /* bFormatIndex: Format #1 */
	0x01,   /* bFrameIndex: Frame #1 */
	FRAME_INTERVALC(4), /* dwFrameInterval: in 100ns */
	0, /* wKeyFrameRate: not used */
	0, /* wPFrameRate: not used */
	10000, /* wCompQuality: highest */
	0, /* wCompWindowSize: ?K */
	100, /* wDelay: Internal VS latency in ms */
	FRAME_BUFFER_SIZEC(800, 600), /* dwMaxVideoFrameSize: in bytes */
	FRAME_PACKET_SIZE_FS /* dwMaxPayloadTransferSize: in bytes */
};


/*-----------------------------------------------------------------------------
 *         Internal variables
 *-----------------------------------------------------------------------------*/

/** Buffer for USB requests data */
COMPILER_ALIGNED(32) static uint8_t pControlBuffer[32];


/** Xfr Maximum packet size */
static uint32_t frmMaxPktSize = FRAME_PACKET_SIZE_HS * (ISO_HIGH_BW_MODE + 1);



static struct _uvc_driver *pUVC_driver;


/*-----------------------------------------------------------------------------
 *      Exported functions
 *-----------------------------------------------------------------------------*/

/*------------ USB Video Device Functions ------------*/

/**
 * Max packet size calculation for High bandwidth transfer\n
 * - Mode 1: last packet is <epSize+1> ~ <epSize*2> bytes\n
 * - Mode 2: last packet is <epSize*2+1> ~ <epSize*3> bytes
 */
void VIDD_UpdateHighBWMaxPacketSize(void)
{

#if (ISO_HIGH_BW_MODE == 1 || ISO_HIGH_BW_MODE == 2)
	uint32_t frmSiz = pUVC_driver->frm_width * pUVC_driver->frm_height * 2 + FRAME_PAYLOAD_HDR_SIZE;
	uint32_t pktSiz = FRAME_PACKET_SIZE_HS * (ISO_HIGH_BW_MODE + 1);
	uint32_t nbLast = frmSiz % pktSiz;

	while (1) {
		nbLast = frmSiz % pktSiz;

		if (nbLast == 0 || nbLast > (FRAME_PACKET_SIZE_HS * ISO_HIGH_BW_MODE))
			break;

		pktSiz --;
	}

	frmMaxPktSize = pktSiz;
#else
	frmMaxPktSize = FRAME_PACKET_SIZE_HS; // EP size
#endif
}

/**
 * Send USB control status.
 */
void VIDD_StatusStage(void)
{
	USBVideoProbeData *pProbe = (USBVideoProbeData *)pControlBuffer;
	viddProbeData.bFormatIndex = pProbe->bFormatIndex;
	viddProbeData.bFrameIndex  = pProbe->bFrameIndex;
	viddProbeData.dwFrameInterval = pProbe->dwFrameInterval;

	switch (pProbe->bFrameIndex) {
	case 1: 
		pUVC_driver->frm_width = VIDCAMD_FW_1; 
		pUVC_driver->frm_height = VIDCAMD_FH_1;
		break;

	case 2: 
		pUVC_driver->frm_width = VIDCAMD_FW_2; 
		pUVC_driver->frm_height = VIDCAMD_FH_2; 
		break;

	case 3: 
		pUVC_driver->frm_width = VIDCAMD_FW_3; 
		pUVC_driver->frm_height = VIDCAMD_FH_3; 
		break;
	}

	VIDD_UpdateHighBWMaxPacketSize();
	USBD_Write(0, 0, 0, 0, 0);
}

/**
 * Handle SetCUR request for USB Video Device.
 */
void VIDD_SetCUR(const USBGenericRequest *pReq)
{
	uint8_t bCS = USBVideoRequest_GetControlSelector(pReq);
	uint32_t len;
	TRACE_INFO_WP("SetCUR(%d) ", pReq->wLength);

	if (pReq->wIndex == VIDCAMD_StreamInterfaceNum) {
		TRACE_INFO_WP("VS ");

		switch (bCS) {
		case VS_PROBE_CONTROL:
			TRACE_INFO_WP("PROBE ");
			len = sizeof(USBVideoProbeData);

			if (pReq->wLength < len) len = pReq->wLength;

			USBD_Read(0, pControlBuffer, len, (TransferCallback)VIDD_StatusStage, 0);
			break;

		case VS_COMMIT_CONTROL:
			TRACE_INFO_WP("COMMIT ");
			len = sizeof(USBVideoProbeData);

			if (pReq->wLength < len) len = pReq->wLength;

			USBD_Read(0, pControlBuffer, len, (TransferCallback)VIDD_StatusStage, 0);

		default: USBD_Stall(0);
		}
	} else if (pReq->wIndex == VIDCAMD_ControlInterfaceNum){
		TRACE_INFO_WP("VC ");
	} else {
		USBD_Stall(0);
	}
}

/**
 * Handle GetCUR request for USB Video Device.
 */
void VIDD_GetCUR(const USBGenericRequest *pReq)
{
	uint8_t bCS = USBVideoRequest_GetControlSelector(pReq);
	uint32_t len;
	TRACE_INFO_WP("GetCUR(%d) ", pReq->wLength);

	if (pReq->wIndex == VIDCAMD_StreamInterfaceNum) {
		TRACE_INFO_WP("VS ");

		switch (bCS) {
		case VS_PROBE_CONTROL:
			TRACE_INFO_WP("PROBE ");
			len = sizeof(USBVideoProbeData);

			if (pReq->wLength < len) len = pReq->wLength;

			USBD_Write(0, &viddProbeData, len, 0, 0);
			break;

		case VS_COMMIT_CONTROL: /* Returns current state of VS I/F */
			TRACE_INFO_WP("COMMIT ");
			USBD_Write(0,  &(pUVC_driver->alternate_interfaces[VIDCAMD_StreamInterfaceNum]), 1, 0, 0);
			break;

		default: USBD_Stall(0);
		}
	} else if (pReq->wIndex == VIDCAMD_ControlInterfaceNum) {
		TRACE_INFO_WP("VC ");
	} else {
		USBD_Stall(0);
	}
}

/**
 * Handle GetDEF request for USB Video Device.
 */
void VIDD_GetDEF(const USBGenericRequest *pReq)
{
	printf("GetDEF(%x,%x,%d)\n\r", pReq->wIndex, pReq->wValue, pReq->wLength);
}

/**
 * Handle GetINFO request for USB Video Device.
 */
void VIDD_GetINFO(const USBGenericRequest *pReq)
{
	printf("GetINFO(%x,%x,%d)\n\r", pReq->wIndex, pReq->wValue, pReq->wLength);
}

/**
 * Handle GetMIN request for USB Video Device.
 */
void VIDD_GetMIN(const USBGenericRequest *pReq)
{
	printf("GetMin(%x,%x,%d)\n\r", pReq->wIndex, pReq->wValue, pReq->wLength);
	VIDD_GetCUR(pReq);

}

/**
 * Handle GetMAX request for USB Video Device.
 */
void VIDD_GetMAX(const USBGenericRequest *pReq)
{
	printf("GetMax(%x,%x,%d)\n\r", pReq->wIndex, pReq->wValue, pReq->wLength);
	VIDD_GetCUR(pReq);
}

/**
 * Handle GetRES request for USB Video Device.
 */
void VIDD_GetRES(const USBGenericRequest *pReq)
{
	printf("GetRES(%x,%x,%d) ", pReq->wIndex, pReq->wValue, pReq->wLength);
}


/* Function instance initialize */
void UVCFunc_Init(struct _uvc_driver* uvc_drv)
{
	pUVC_driver = uvc_drv;
}


/*
 *  Get the value of frmMaxPktSize
 */
uint32_t UVC_get_frmMaxPktSize(void)
{
	return frmMaxPktSize;
}
/**@}*/

