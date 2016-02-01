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


#ifndef UVCDRIVER_H
#define UVCDRIVER_H

/** \addtogroup usbd_composite_hidmsd
 *@{
 */

/*---------------------------------------------------------------------------
 *         Headers
 *---------------------------------------------------------------------------*/

#include "board.h"

#include <USBDescriptors.h>
#include <USBRequests.h>
#include "USBD.h"
#include <USBD_HAL.h>
#include <USBDDriver.h>
#include <VIDEODescriptors.h>
#include <USBVideo.h>


/*-----------------------------------------------------------------------------
 *         Internal Types
 *-----------------------------------------------------------------------------*/

/**
 * \brief USB Video class driver struct.
 */
struct _uvc_driver {
	uint32_t *buf;

	/** USB Streaming interface ON */
	volatile uint8_t is_video_on;

	/** USB transferring frame data */
	volatile uint8_t is_frame_xfring;

	/** Frame size: Width, Height */
	uint32_t frm_width;
	uint32_t frm_height;

	/** Frame count */
	uint32_t frm_count;
	/** Byte index in frame */
	uint32_t frm_index;

	/** Array for storing the current setting of each interface */
	uint8_t alternate_interfaces[4];
};

/** ISI DMA buffer base address */
#define ISI_BASE    SDRAM_CS_ADDR

/** Frame Buffer Descriptors , it depends on size of external memory, more
        is better */
#define ISI_MAX_PREV_BUFFER    12

/** Frame size: Width, Height */
#define VIDEO_WIDTH     320
#define VIDEO_HEIGHT    240

/*---------------------------------------------------------------------------
 *         Exported functions
 *---------------------------------------------------------------------------*/
extern void UVCDriver_Init(const USBDDriverDescriptors *descriptors, uint32_t *buff_addr);
extern struct _uvc_driver* UVC_get_driver(void);
extern void _PreviewMode(uint32_t,uint32_t);
extern uint8_t UVC_is_video_on(void);
extern uint32_t UVC_frm_width(void);
extern uint32_t UVC_frm_height(void);

/**@}*/
#endif //#ifndef UVCDRIVER_H


