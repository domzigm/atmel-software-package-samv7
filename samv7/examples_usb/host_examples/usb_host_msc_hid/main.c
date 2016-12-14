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
 *  \page usb_host_msc_hid USB Host HID + MSD Example
 *
 *  \section Purpose
 *
 *  This application shows how to implement a USB
 *  host compound mass storage & HID mouse on SAM V71 Xplained Ultra board.
 *
 *  \section Requirements
 *  This package can be used with SAM V71 Xplained Ultra board.
 *
 *  \section Description
 *
 *  Connect the board to a U-Disk (FAT/FAT32 are supported). This example will
 *  creates a file "SAMx7_USBHostTest.txt" on all present U-disks.
 *  Connect the HID mouse device to board. This example will display events of
 *  the mouse.
 *
 *  \section Usage
 *
 *  -# Build the program and download it inside the board.
 *    Please refer to the Getting Started with SAM V71/E70 Microcontrollers.pdf
 *  -# On the computer, open and configure a terminal application
 *     (e.g. HyperTerminal on Microsoft Windows) with these settings:
 *    - 115200 baud rate
 *    - 8 bits of data
 *    - No parity
 *    - 1 stop bit
 *    - No flow control
 *  -# Connect the U-disk/USB mouse to SAM V71 Xplained Ultra board or SAME70 Xplained board
 *     with the OTG wire.
 *  -# Start the application. In the terminal window, the
 *     following text should appear (values depend on the board and chip used):
 *     \code
 *      -- USB Host Mass Storage Example xxx --
 *      -- xxxxxx-xx
 *      -- Compiled: xxx xx xxxx xx:xx:xx --
 *     \endcode
 *  -# LED0 is on when a device is connected and blinks with different speed when
 *  the device is enumerated and USB is in idle mode.
 *  -# LED1 is on when a read or write access is on going or the mouse moves.
 *  -# In addition, a file "SAMx7_USBHostTest.txt" with the text "Test:- SAMV7/E7
 *  USB Host MSC" is created in the U-disk.
 *
 *  \section References
 *  - usb_host_msc/main.c
 *  - usb_host_msc/ui.c
 *  - led.c
 *  - USBH.c
 *  - USBH_HAL.c
 *  - uhi_msc.c
 *  - uhi_msc_mem.c
 *  - uhi_hid_mouse.c
 */


#include "board.h"
#include "conf_usb_host.h"
#include "ui.h"
#include "main.h"
#include "string.h"

#define MAX_DRIVE      _VOLUMES
#define TEST_FILE_NAME "0:SAMx7_USBHostTest.txt"
#define MSG_TEST "Test:- SAMV7/E7 USB Host MSC\n"

typedef enum test_state {
	TEST_NULL,
	TEST_OK,
	TEST_NO_PRESENT,
	TEST_ERROR
} test_state_t;

static volatile uint16_t main_usb_sof_counter = 0;

static test_state_t lun_states[MAX_DRIVE];

static FATFS fs; // Re-use fs for LUNs to reduce memory footprint
static FIL file_object;

static char test_file_name[] = {
	TEST_FILE_NAME
};

static void main_reset_states(void);
static int main_count_states(test_state_t state);


/*! \brief Main function. Execution starts here.
 */
int main(void)
{
	/* Disable watchdog*/
	WDT_Disable(WDT);

	SCB_EnableICache();
	SCB_EnableDCache();

	/* Output example information */
	printf("-- USB Host Mass Storage Example %s --\n\r", SOFTPACK_VERSION);
	printf("-- %s\n\r", BOARD_NAME);
	printf("-- Compiled: %s %s  With %s--\n\r", __DATE__, __TIME__ ,
			COMPILER_NAME);

	TimeTick_Configure();

	ui_init();

	// Start USB host stack
	USBH_start();

	// The USB management is entirely managed by interrupts.
	// As a consequence, the user application does only have :
	// - to play with the power modes
	// - to create a file on each new LUN connected
	while (true) {
		//sleepmgr_enter_sleep();
		if (main_usb_sof_counter > 2000) {
			main_usb_sof_counter = 0;
			volatile uint8_t lun;
			FRESULT res;

			for (lun = LUN_ID_USB; (lun < LUN_ID_USB +
									uhi_msc_mem_get_lun()) &&
				 (lun < MAX_DRIVE); lun++) {
				// Check if LUN has been already tested
				if (TEST_OK == lun_states[lun] ||
					TEST_ERROR == lun_states[lun])
					continue;
				else
					printf("LUN ok\n\r");

				// Mount drive
				memset(&fs, 0, sizeof(FATFS));
				res = f_mount(&fs, (TCHAR const *)&lun, 1);

				if (FR_INVALID_DRIVE == res) {
					// LUN is not present
					lun_states[lun] = TEST_NO_PRESENT;
					printf("did not mount ok\n\r");
					continue;
				} else
					printf("Mount ok\n\r");

				// Create a test file on the disk
				test_file_name[0] = lun + '0';
				res = f_open(&file_object,
							 (TCHAR const *)test_file_name,
							 FA_CREATE_ALWAYS | FA_WRITE);

				if (res == FR_NOT_READY) {
					// LUN not ready
					lun_states[lun] = TEST_NO_PRESENT;
					f_close(&file_object);
					printf("File create error\n\r");
					continue;
				} else
					printf("File create ok\n\r");

				if (res != FR_OK) {
					// LUN test error
					lun_states[lun] = TEST_ERROR;
					f_close(&file_object);
					printf("File system error\n\r");
					continue;
				}

				// Write to test file
				f_puts((const TCHAR *)MSG_TEST, &file_object);
				printf("File Writing\n\r");
				// LUN test OK
				lun_states[lun] = TEST_OK;
				f_close(&file_object);
				printf("File close\n\r");
			}

			if (main_count_states(TEST_NO_PRESENT) == MAX_DRIVE) {
				ui_test_finish(false); // Test fail
			} else if (MAX_DRIVE != main_count_states(TEST_NULL)) {
				if (main_count_states(TEST_ERROR)) {
					ui_test_finish(false); // Test fail
				} else if (main_count_states(TEST_OK)) {
					ui_test_flag_reset();
					ui_test_finish(true); // Test OK
				}
			} else
				ui_test_flag_reset();
		}
	}
}

void main_usb_sof_event(void)
{
	main_usb_sof_counter++;
	ui_usb_sof_event();
}

void main_usb_connection_event(USBH_device_t *dev, bool b_present)
{
	if (!b_present) {
		main_reset_states(); // LUN is unplugged, reset flags
	}

	ui_usb_connection_event(dev, b_present);
}

static void main_reset_states(void)
{
	int i;

	for (i = 0; i < MAX_DRIVE; i ++)
		lun_states[i] = TEST_NULL;
}

static int main_count_states(test_state_t state)
{
	int i, count = 0;

	for (i = 0; i < MAX_DRIVE; i ++) {
		if (lun_states[i] == state)
			count ++;
	}

	return count;
}
