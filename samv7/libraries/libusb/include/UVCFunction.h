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

/** \file
 *  USB Video Class function driver definitions.
 */

#ifndef UVCFUNCTION_H
#define UVCFUNCTION_H

/** \addtogroup usbd_uvc
 *@{
 */

/*------------------------------------------------------------------------------
 *         Headers
 *------------------------------------------------------------------------------*/

#include <stdint.h>
#include <UVCDriver.h>

/*------------------------------------------------------------------------------
 *      Global functions
 *------------------------------------------------------------------------------*/

extern void VIDD_UpdateHighBWMaxPacketSize(void);
extern void VIDD_StatusStage(void);
extern void VIDD_SetCUR(const USBGenericRequest *pReq);
extern void VIDD_GetCUR(const USBGenericRequest *pReq);
extern void VIDD_GetDEF(const USBGenericRequest *pReq);
extern void VIDD_GetINFO(const USBGenericRequest *pReq);
extern void VIDD_GetMIN(const USBGenericRequest *pReq);
extern void VIDD_GetMAX(const USBGenericRequest *pReq);
extern void VIDD_GetRES(const USBGenericRequest *pReq);
extern void VIDD_PayloadSent(void *pArg, uint8_t bStat);
extern uint32_t UVC_get_frmMaxPktSize(void);
extern void UVCFunc_Init(struct _uvc_driver* uvc_drv);

/**@}*/

#endif /* UVCDRIVER_H */
