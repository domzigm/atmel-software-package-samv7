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
 * Human interface on  SAM V71 Xplained Ultra board:
 * - Led 0 is continuously on when a device is connected
 * - Led 0 blinks when the device is enumerated and USB in idle mode
 *   - The blink is slow (1s) with low speed device
 *   - The blink is normal (0.5s) with full speed device
 *   - The blink is fast (0.25s) with high speed device
 * - Led 1 is on when a read or write access is on going or the mouse moves.
 */

#include "board.h"
#include "ui.h"
#include "ctrl_access.h"
#include "uhi_msc_mem.h"


/**
 * \name Internal routines to manage asynchronous interrupt pin change
 * This interrupt is connected to a switch and allows to wakeup CPU in low sleep
 * mode.
 * This wakeup the USB devices connected via a downstream resume.
 * @{
 */
static void ui_disable_asynchronous_interrupt(void);

/* Interrupt on "pin change" from RIGHT CLICK to do wakeup on USB
 *  Note:
 *  This interrupt is enable when the USB host enable remotewakeup feature
 *  This interrupt wakeup the CPU if this one is in idle mode */

/**
 * \brief Disables interrupt pin change
 */
static void ui_disable_asynchronous_interrupt(void)
{
}

/*! @} */

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
	UNUSED(b_host_mode);
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
/*! Result of the MSC test */
static bool ui_test_result;
/*! Manages device mouse moving */
static int8_t ui_x, ui_y, ui_scroll;


void ui_usb_vbus_change(bool b_vbus_present)
{
	b_vbus_present = b_vbus_present;
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
	UNUSED(dev);
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

}
void ui_uhi_hid_mouse_change(USBH_device_t *dev, bool b_plug)
{
	UNUSED(dev);
    UNUSED(b_plug);
}

void ui_uhi_msc_change(USBH_device_t *dev, bool b_plug)
{
	UNUSED(dev);
    UNUSED(b_plug);
}

void ui_usb_wakeup_event(void)
{
	ui_disable_asynchronous_interrupt();
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

		/* Power on a LED when the mouse move */
		if (!ui_x && !ui_y && !ui_scroll) {
#if 2 == LED_NUM
			LED_Clear(LED_YELLOW1);
#endif
		} else {
			ui_x = ui_y = ui_scroll = 0;
#if 2 == LED_NUM
			LED_Set(LED_YELLOW1);
#endif
		}
	}
}

static void ui_uhi_hid_mouse_btn(bool b_state)
{
	static uint8_t nb_down = 0;

	if (b_state)
		nb_down++;
	else
		nb_down--;
}

void ui_test_flag_reset(void)
{
	LED_Clear(LED_YELLOW0);
}

void ui_test_finish(bool b_success)
{
	ui_test_result = b_success;

	if (ui_test_result) {
#if 2 == LED_NUM
		LED_Set(LED_YELLOW1);
#endif
	}
}

/*! @} */

/*! \name Callback to mange the HID mouse events
 *  @{ */
void ui_uhi_hid_mouse_btn_left(bool b_state)
{
	TRACE_INFO_WP("\r\t\t\t\tLeft Button clicked              ");
	ui_uhi_hid_mouse_btn(b_state);
}

void ui_uhi_hid_mouse_btn_right(bool b_state)
{
	TRACE_INFO_WP("\r\t\t\t\tRight Button clicked             ");
	ui_uhi_hid_mouse_btn(b_state);
}

void ui_uhi_hid_mouse_btn_middle(bool b_state)
{
	ui_uhi_hid_mouse_btn(b_state);
}

void ui_uhi_hid_mouse_move(int8_t x, int8_t y, int8_t scroll)
{
	ui_x = x;
	ui_y = y;
	TRACE_INFO_WP("\r Mouse at X:%d, Y:%d      ", ui_x, ui_y);
	ui_scroll = scroll;
}
/*! @} */

/*! \name Callback to show the MSC read and write access
 *  @{ */
void ui_start_read(void)
{
#if 2 == LED_NUM
	LED_Set(LED_YELLOW1);
#endif
}

void ui_stop_read(void)
{
#if 2 == LED_NUM
	LED_Clear(LED_YELLOW1);
#endif
}

void ui_start_write(void)
{
#if 2 == LED_NUM
	LED_Set(LED_YELLOW1);
#endif
}

void ui_stop_write(void)
{
#if 2 == LED_NUM
	LED_Clear(LED_YELLOW1);
#endif
}

/*! @} */
