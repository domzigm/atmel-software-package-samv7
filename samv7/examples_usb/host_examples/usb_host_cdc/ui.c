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
 * \defgroup UI User Interface
 *
 * Human interface on SAM V71 Xplained Ultra board.:
 * - SAM V71 USART used USART2 on J505 connector
 * - Led 0 is continuously on when a device is connected
 * - Led 0 blinks when USB host has checked and enabled CDC interface
 *   - The blink is slow (1s) with low speed device
 *   - The blink is normal (0.5s) with full speed device
 *   - The blink is fast (0.25s) with high speed device
 * - Led 1 is on during data transfer between CDC and UART
 */
#include "board.h"
#include "ui.h"
#include "conf_usb_host.h"

/**
 * \name Main user interface functions
 * @{
 */
void ui_init(void)
{
	/* Initialize LEDs */
	LED_Configure(LED_YELLOW0);
#if 2 == LED_NUM
	LED_Configure(LED_YELLOW1);
#endif
}

void ui_usb_mode_change(bool b_host_mode)
{
	(void)b_host_mode;
	ui_init();
}
/*! @} */

/**
 * \name Host mode user interface functions
 * @{
 */

/*! Status of device enumeration */
static USBH_enum_status_t ui_enum_status = UHC_ENUM_DISCONNECT;
/*! Blink frequency depending on device speed */
static uint16_t ui_device_speed_blink;

void ui_usb_vbus_change(bool b_vbus_present)
{
	if (b_vbus_present) {
		//LED_On(LED3_GPIO);
	} else {
		//LED_Off(LED3_GPIO);
	}
}

void ui_usb_vbus_error(void)
{
}

void ui_usb_connection_event(USBH_device_t *dev, bool b_present)
{
	UNUSED(dev);

	if (b_present)
		LED_Set(LED_YELLOW0);
	else {
		LED_Clear(LED_YELLOW0);
		ui_enum_status = UHC_ENUM_DISCONNECT;
	}
}

void ui_usb_enum_event(USBH_device_t *dev, USBH_enum_status_t status)
{
	ui_enum_status = status;

	switch (dev->speed) {
	case UHD_SPEED_HIGH:
		ui_device_speed_blink = 250;
		break;

	case UHD_SPEED_FULL:
		ui_device_speed_blink = 500;
		break;

	case UHD_SPEED_LOW:
	default:
		ui_device_speed_blink = 1000;
		break;
	}

	if (ui_enum_status == UHC_ENUM_SUCCESS) {
		/* USB Device CDC connected
		   Open and configure UART and USB CDC ports */
		CDCLineCoding cfg = {
			.dwDTERate   = (115200),
			.bCharFormat = CDCLineCoding_ONESTOPBIT,
			.bParityType = CDCLineCoding_NOPARITY,
			.bDataBits   = 8,
		};
		TRACE_INFO("USART OPEN");
		uart_open();
		uart_config(&cfg);
		uhi_cdc_open(0, &cfg);
	}
}

void ui_usb_wakeup_event(void)
{
}

void ui_usb_sof_event(void)
{
	static uint16_t counter_sof = 0;

	if (ui_enum_status == UHC_ENUM_SUCCESS) {
		/* Display device enumerated and in active mode */
		if (++counter_sof > ui_device_speed_blink) {
			counter_sof = 0;
			LED_Toggle(LED_YELLOW0);
		}
	}
}

void ui_com_rx_start(void)
{
#if 2 == LED_NUM
	LED_Set(LED_YELLOW1);
#endif
}

void ui_com_rx_stop(void)
{
#if 2 == LED_NUM
	LED_Clear(LED_YELLOW1);
#endif
}

void ui_com_tx_start(void)
{
#if 2 == LED_NUM
	LED_Set(LED_YELLOW1);
#endif
}

void ui_com_tx_stop(void)
{
#if 2 == LED_NUM
	LED_Clear(LED_YELLOW1);
#endif
}

void ui_com_error(void)
{
}

void ui_com_overflow(void)
{
}

/*! @} */


