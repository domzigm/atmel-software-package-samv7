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
 *  - Human interface on SAM V71 Xplained Ultra board.:
 *   - Led 0 is continuously on when a device is connected
 *   - Led 0 blinks when USB host has checked and enabled HID interface
 *   - The blink is slow (1s) with low speed device
 *   - The blink is normal (0.5s) with full speed device
 *   - The blink is fast (0.25s) with high speed device
 *   - Led 1 is on when the mouse move
 */

#include "board.h"
#include "ui.h"

/* Wakeup pin is RIGHT CLICK (fast wakeup 14) */
#define  RESUME_PMC_FSTT (PMC_FSMR_FSTT14)
#define  RESUME_PIN      (GPIO_PUSH_BUTTON_2)
#define  RESUME_PIO      (PIN_PUSHBUTTON_2_PIO)
#define  RESUME_PIO_ID   (PIN_PUSHBUTTON_2_ID)
#define  RESUME_PIO_MASK (PIN_PUSHBUTTON_2_MASK)
#define  RESUME_PIO_ATTR (PIN_PUSHBUTTON_2_ATTR)

/**
 * \name Internal routines to manage asynchronous interrupt pin change
 * This interrupt is connected to a switch and allows to wakeup CPU in low sleep
 * mode.
 * This wakeup the USB devices connected via a downstream resume.
 * @{
 */
static void ui_enable_asynchronous_interrupt(void);
static void ui_disable_asynchronous_interrupt(void);


/**
 * \brief Initializes and enables interrupt pin change
 */
static void ui_enable_asynchronous_interrupt(void)
{

}

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
/*! Manages device mouse moving */
static int8_t ui_x, ui_y, ui_scroll;

void ui_usb_vbus_change(bool b_vbus_present)
{
	b_vbus_present = b_vbus_present;

}

void ui_usb_vbus_error(void)
{
}

void ui_usb_connection_event(USBH_device_t *pDev, bool b_present)
{
	(void)pDev;

	if (b_present)
		LED_Set(LED_YELLOW0);
	else {
		LED_Clear(LED_YELLOW0);
		ui_enum_status = UHC_ENUM_DISCONNECT;
	}
}

void ui_usb_enum_event(USBH_device_t *pDev, USBH_enum_status_t status)
{
	(void)pDev;
	ui_enum_status = status;

	if (ui_enum_status == UHC_ENUM_SUCCESS) {
		ui_x = 0, ui_y = 0, ui_scroll = 0;

		switch (pDev->speed) {
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
}

void ui_usb_wakeup_event(void)
{
	ui_disable_asynchronous_interrupt();
}

void ui_usb_sof_event(void)
{
	bool b_btn_state;
	static bool btn_suspend_and_remotewakeup = false;
	static uint16_t counter_sof = 0;

	if (ui_enum_status == UHC_ENUM_SUCCESS) {
		/* Display device enumerated and in active mode */
		if (++counter_sof > ui_device_speed_blink) {
			counter_sof = 0;
			LED_Toggle(LED_YELLOW0);
		}

		/* Scan button to enter in suspend mode and remote wakeup */
		/*b_btn_state = (!gpio_pin_is_high(GPIO_PUSH_BUTTON_1)) ?
		        true : false;*/
		b_btn_state = true;

		if (b_btn_state != btn_suspend_and_remotewakeup) {
			/* Button have changed */
			btn_suspend_and_remotewakeup = b_btn_state;

			if (b_btn_state) {
				/* Button has been pressed */
				ui_enable_asynchronous_interrupt();
				USBH_suspend(true);
				return;
			}
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

	if (nb_down) {
		// do something
	} else {
		//do something
	}
}

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

