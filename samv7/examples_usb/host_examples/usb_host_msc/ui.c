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
 * - Led 1 is on when a read or write access is on going
 * - Led 1 is on when a LUN test is success
 * - Led 1 blinks when a LUN test is unsuccess
 */

#include "board.h"
#include "ui.h"
#include "ctrl_access.h"
#include "uhi_msc_mem.h"

/* Wakeup pin is RIGHT CLICK (fast wakeup 14) */
#define  RESUME_PMC_FSTT (PMC_FSMR_FSTT14)
#define  RESUME_PIN      (GPIO_PUSH_BUTTON_2)
#define  RESUME_PIO      (PIN_PUSHBUTTON_2_PIO)
#define  RESUME_PIO_ID   (PIN_PUSHBUTTON_2_ID)
#define  RESUME_PIO_MASK (PIN_PUSHBUTTON_2_MASK)
#define  RESUME_PIO_ATTR (PIN_PUSHBUTTON_2_ATTR)


static void ui_disable_asynchronous_interrupt(void);

/**
 * \brief Disables interrupt pin change
 */
static void ui_disable_asynchronous_interrupt(void)
{
	/* Disable interrupt for button pin */
	//pio_disable_pin_interrupt(GPIO_PUSH_BUTTON_2);
	//pio_get_interrupt_status(PIOB);
	/* Enable fastwakeup for button pin */
	//pmc_clr_fast_startup_input(PMC_FSMR_FSTT14);
}

/*! @} */

/**
 * \name Main user interface functions
 * @{
 */
void ui_init(void)
{
	/* Enable PIO clock for button inputs */
	//pmc_enable_periph_clk(ID_PIOB);
	//pmc_enable_periph_clk(ID_PIOE);
	/* Set handler for wakeup */
	//pio_handler_set(RESUME_PIO, RESUME_PIO_ID, RESUME_PIO_MASK,
	//  RESUME_PIO_ATTR, ui_wakeup_handler);
	/* Enable IRQ for button (PIOB) */
	//NVIC_EnableIRQ((IRQn_Type)RESUME_PIO_ID);
	/* Enable interrupt for button pin */
	//pio_get_interrupt_status(RESUME_PIO);
	//pio_configure_pin(RESUME_PIN, RESUME_PIO_ATTR);
	//pio_enable_pin_interrupt(RESUME_PIN);
	/* Enable fastwakeup for button pin */
	//pmc_set_fast_startup_input(RESUME_PMC_FSTT);
	/* Initialize LEDs */
	LED_Configure(LED_YELLOW0);
	LED_Clear(LED_YELLOW0);
#if 2 == LED_NUM
	LED_Configure(LED_YELLOW1);
	LED_Clear(LED_YELLOW1);
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
/*! Status of the MSC test */
static bool ui_test_done;
/*! Result of the MSC test */
static bool ui_test_result;

void ui_usb_vbus_change(bool b_vbus_present)
{
	b_vbus_present = b_vbus_present;
	/*if (b_vbus_present) {
	    //LED_Set(LED3_GPIO);
	} else {
	    //LED_Clear(LED3_GPIO);
	}*/
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

	ui_test_done = false;
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

			if (ui_test_done && !ui_test_result) {
				/* Test fail then blink led */
#if 2 == LED_NUM
				LED_Toggle(LED_YELLOW1);
#endif
			}
		}
	}
}

void ui_test_flag_reset(void)
{
	ui_test_done = false;
	LED_Clear(LED_YELLOW0);
}

void ui_test_finish(bool b_success)
{
	ui_test_done = true;
	ui_test_result = b_success;

	if (b_success) {
#if 2 == LED_NUM
		LED_Set(LED_YELLOW1);
#endif
	}
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
