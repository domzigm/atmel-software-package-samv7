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
 * \file
 *
 * \brief Declaration of main function used by example
 *
 */
/*
 * Support and FAQ: visit <a href="http://www.atmel.com/design-support/">Atmel Support</a>
 */

#ifndef _MAIN_H_
#define _MAIN_H_


/*
 * This file includes all API header files for the selected drivers from ASF.
 * Note: There might be duplicate includes required by more than one driver.
 *
 * The file is automatically generated and will be re-written when
 * running the ASF driver selector tool. Any changes will be discarded.
 */

// From module: Common SAM compiler driver
//#include <compiler.h>
//#include <status_codes.h>

// From module: FatFS file system
#include "diskio.h"
#include "ff.h"
#include "ffconf.h"
#include "integer.h"
// From module: Memory Control Access Interface
#include "ctrl_access.h"


// From module: Supported ports of FatFS
#include "diskio.h"

#include "USBH.h"

//! \brief Notify that a SOF has been sent (each 1 ms)
void main_usb_sof_event(void);

/*! \brief Notify that a USB device has been connected or disconnected.
 *
 * \param dev         Pointer on USB device information
 * \param b_present   true, if the device has been connected
 */
void main_usb_connection_event(USBH_device_t *dev, bool b_present);

#endif // _MAIN_H_
