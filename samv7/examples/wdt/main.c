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
 * \page wdt Watchdog with IRQ Interrupt Example
 *
 * \section Purpose
 *
 * This example demonstrates user to trigger a watchdog interrupt
 * if the software becomes trapped in a deadlock.
 *
 * \section Requirements
 *
 *  This package can be used with SAMV71 Xplained Ultra board or SAME70 Xplained board.
 *
 * \section Description
 *
 * When launched, this program reloads the watchdog at regular intervals
 * before the timer underflow occurs, a LED is blinked. User could press
 * button1 to make the program run in a infinite loop without
 * reloading the watchdog. So a watchdog interrupt will be triggered, and
 * "Enter watchdog interrupt." will print to terminal.
 *
 * \note
 * -# User can enable a watchdog reset instead of an interrupt by setting
 * WDRSTEN bit in WDT_MR register.
 *
 * \section Usage
 *
 * -# Build the program and download it inside the board.
 * Please refer to the Getting Started with SAM V71/E70 Microcontrollers.pdf
 * -# On the computer, open and configure a terminal application
 *    (e.g. HyperTerminal on Microsoft Windows) with these settings:
 *   - 115200 baud rate
 *   - 8 bits of data
 *   - No parity
 *   - 1 stop bit
 *   - No flow control
 * -# Start the application.
 * -# In the terminal window, the following text should appear:
 *    \code
 *     -- Watchdog with IRQ Interrupt Example xxx --
 *     -- xxxxxx-xx
 *     -- Compiled: xxx xx xxxx xx:xx:xx --
 *    \endcode
 *
 * The user could press the button1 to trigger a watchdog interrupt.
 *
 * \section References
 * - wdt/main.c
 * - wdt.c
 * - wdt.h
 */

/**
 * \file
 *
 * This file contains all the specific code for the wdt example.
 */

/*----------------------------------------------------------------------------
 *        Headers
 *----------------------------------------------------------------------------*/

#include "board.h"

/* These headers were introduced in C99 by working group ISO/IEC JTC1/SC22/WG14. */
#include <stdbool.h>
#include <stdio.h>

/*----------------------------------------------------------------------------
 *        Local definitions
 *----------------------------------------------------------------------------*/

/** LED used in this program */
#define LED_ID    1

/** LED blink time, in ms */
#define BLINK_PERIOD        300

/** Watchdog period, in ms */
#define WDT_PERIOD           3000
/** Watchdog restart period, in ms */
#define WDT_RESTART_PERIOD   2000

/*----------------------------------------------------------------------------
 *        Local variables
 *----------------------------------------------------------------------------*/

/** Pushbutton \#1 pin instance. */
const Pin pinPB1 = PIN_PUSHBUTTON_0;

/** Pushbutton \#1 pin event flag. */
volatile bool button1Evt = false;

volatile uint32_t gSystick = 0;
/*----------------------------------------------------------------------------
 *        Local functions
 *----------------------------------------------------------------------------*/


/**
 *  \brief Handler for Button 1 rising edge interrupt.
 *
 *  Set button1 event flag (button1Evt).
 */
static void _Button1_Handler(const Pin* pPin)
{
	if (pPin == &pinPB1)
		button1Evt = true;
}

/**
 *  \brief Handler for watchdog interrupt.
 */
void WDT_Handler(void)
{
	Wdt *pWdt = WDT;
	volatile uint32_t dummy;

	/* Clear status bit to acknowledge interrupt */
	dummy = pWdt->WDT_SR;

	printf("Enter watchdog interrupt.\n\r");
#ifdef sram
	WDT_Restart(WDT);
	printf("The watchdog timer was restarted.\n\r");
#else
	printf("Processor reset\n\n\n\r");
	RSTC_ExtReset();
#endif
}

/**
 *  \brief Configure the Pushbuttons
 *
 *  Configure the PIO as inputs and generate corresponding interrupt when
 *  pressed or released.
 */
static void _ConfigureButtons(void)
{
	/* Configure PIO as inputs. */
	PIO_Configure(&pinPB1, 1);

	/* Adjust PIO debounce filter parameters, uses 10 Hz filter. */
	PIO_SetDebounceFilter(&pinPB1, 10);

	/* Initialize PIO interrupt handlers, see PIO definition in board.h. */
	PIO_ConfigureIt(&pinPB1, _Button1_Handler); /* Interrupt on rising edge  */

	/* Enable PIO controller IRQs. */
	NVIC_EnableIRQ((IRQn_Type)pinPB1.id);

	/* Enable PIO line interrupts. */
	PIO_EnableIt(&pinPB1);
}

/**
 *  \brief Configure LEDs
 *
 *  Configures LED (cleared by default).
 */
static void _ConfigureLeds(void)
{
	LED_Configure(LED_ID);
}

/*----------------------------------------------------------------------------
 *         Global functions
 *----------------------------------------------------------------------------*/

/**
 * \brief Application entry point for wdg_irq example.
 *
 * \return Unused (ANSI-C compatibility).
 */
extern int main( void )
{
	uint32_t dwPeriod;
	uint32_t startTime;

	SCB_EnableICache();
	SCB_EnableDCache();

	/* Output example information */
	printf("-- Watchdog with IRQ Interrupt Example %s --\n\r", SOFTPACK_VERSION);
	printf("-- %s\n\r", BOARD_NAME);
	printf("-- Compiled: %s %s With %s--\n\r", __DATE__, __TIME__, COMPILER_NAME);

	/* Sys tick configuration. */
	printf("Configure sys tick to get 1ms tick period.\n\r");

	TimeTick_Configure();

	/* PIO configuration for LEDs and Buttons. */
	PIO_InitializeInterrupts(0);
	_ConfigureLeds();
	_ConfigureButtons();

	/* Configure WDT to trigger a interrupt (or reset) */
	printf("Enable watchdog with %u millisecond period\n\r",
			(unsigned int)WDT_PERIOD);
	dwPeriod = WDT_GetPeriod(WDT_PERIOD);

#if 1 /* trigger a watchdog interrupt */
	WDT_Enable(WDT, WDT_MR_WDFIEN | WDT_MR_WDDBGHLT
			| WDT_MR_WDIDLEHLT | (dwPeriod << 16) | dwPeriod);
	NVIC_DisableIRQ(WDT_IRQn);
	NVIC_ClearPendingIRQ(WDT_IRQn);
	NVIC_SetPriority(WDT_IRQn, 0);
	NVIC_EnableIRQ(WDT_IRQn);
#else /* trigger a watchdog reset */
	WDT_Enable(WDT, WDT_MR_WDRSTEN | WDT_MR_WDDBGHLT
			| WDT_MR_WDIDLEHLT | (dwPeriod << 16) | dwPeriod);
#endif
	startTime = GetTicks();
	printf("Press USRPB0 to simulate a deadlock loop.\n\r");

	while (1) {
		if ((gSystick != GetTicks()) &&
			((GetDelayInTicks( startTime, GetTicks())) < 0xFFFFFFFF)) {
			gSystick = GetTicks();
			/* Toggle led at given period */
			if ((GetTicks() % BLINK_PERIOD) == 0)
				LED_Toggle(LED_ID);
			/* Restart watchdog at given period */
			if ((GetTicks() % WDT_RESTART_PERIOD) == 0)
				WDT_Restart(WDT);
		}

		/* Simulate deadlock when button be pressed */
		if (button1Evt == true) {
			printf( "Program enter infinite loop for triggering watchdog \
				interrupt.\n\r" );
			while (1);
		}
	}
}

