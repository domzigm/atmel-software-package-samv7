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
 * \brief Common User Interface for USB host MSC application
 */
/*
 * Support and FAQ: visit <a href="http://www.atmel.com/design-support/">Atmel Support</a>
 */

#ifndef _UI_H_
#define _UI_H_

#include "USBH.h"

//! \brief Initializes the user interface
void ui_init(void);

/*! \brief Notify that the USB mode are switched automatically.
 * This is possible only when ID pin is available.
 *
 * \param b_host_mode true, if the host mode has been selected
 */
void ui_usb_mode_change(bool b_host_mode);

/*! \brief Notify that a Vbus are changed
 * Available only in USB hardware with Vbus monitoring.
 *
 * \param b_vbus_present true, if Vbus is high.
 */
void ui_usb_vbus_change(bool b_vbus_present);

/*! \brief Notify that a Vbus error has occurred
 * Available only in USB hardware with Vbus monitoring.
 */
void ui_usb_vbus_error(void);

/*! \brief Notify that a USB device has been connected or disconnected.
 *
 * \param dev         Pointer on USB device information
 * \param b_present   true, if the device has been connected
 */
void ui_usb_connection_event(USBH_device_t *dev, bool b_present);

//! \brief Notify that a USB device or the host has wake up the USB line.
void ui_usb_wakeup_event(void);

//! \brief Notify that a SOF has been sent (each 1 ms)
void ui_usb_sof_event(void);

//! \brief Resets the status flag of the test
void ui_test_flag_reset(void);

/*! \brief Displays the result of the test
 *
 * \param b_success   true, if the test is successfull
 */
void ui_test_finish(bool b_success);

/*! \brief Notify the end of a USB device enumeration
 *
 * \param dev         Pointer on USB device information
 * \param status      Status of the USB enumeration
 */
void ui_usb_enum_event(USBH_device_t *dev, USBH_enum_status_t status);

//! \name Callback to show the MSC read and write accesses
//! Functions are already declared in conf_access.h.
//! @{
//! @}

#endif // _UI_H_
