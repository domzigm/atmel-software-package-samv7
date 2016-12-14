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
 * \page tc_capture_waveform TC Capture Waveform Example
 *
 * \section Purpose
 *
 * This example indicates how to use TC in capture mode to measure the pulse
 * frequency and count the total pulse number of an external signal injected
 * on TIOA pin.
 *
 * \section Requirements
 *
 * This package can be used with SAMV71 Xplained Ultra board or SAME70 Xplained board.
 * It generates a waveform from one channel TIOAx , and it capture wave from another
 * channel TIOAy. To measure the waveform on TIOAx, you shall connect TIOAx to
 * TIOAy, configure TIOAx as output pin and TIOAy as input pin.
 *
 *
 * \section Descriptions
 *
 * This example shows how to configure TC in waveform and capture mode.
 * In capture mode, pulse signal is set as an input, RA and RB will be loaded when
 * programmed event occurs. When TC interrupt happens, we could read RA and RB
 * value for calculating pulse frequency and pulse number be increased. The current
 * pulse frequency and total pulse number is output on DBGU.
 *
 * The code can be roughly broken down as follows:
 * <ul>
 * <li>Select pre-defined waveform frequency and duty cycle to be generated.
 * <li>Configure TC channel a as waveform output.
 * <li>Configure TC channel b as capture input.
 * <li>Configure capture Register A be loaded when rising edge of TIOA occurs.
 * <li>Configure capture Register B be loaded when failing edge of TIOA occurs.
 * <li>Configure an interrupt for TC and enable the RB load interrupt.
 * <li> 'c' start capture.
 * <li> 's' will stop capture,and dump the informations what have been captured.
 * </ul>
 *
 * \section Usage
 *
 * -# Build the program and download it inside the board.
 *    Please refer to the Getting Started with SAM V71/E70 Microcontrollers.pdf
 * -# Connect Pin07 with Pin16 on EXT1 on the board.
 * -# On the computer, open and configure a terminal application
 *    (e.g. HyperTerminal on Microsoft Windows) with these settings:
 *   - 115200 baud rate
 *   - 8 bits of data
 *   - No parity
 *   - 1 stop bit
 *   - No flow control
 * -# Start the application.
 * -# In the terminal window, the following text should appear (values depend
 * on the board and chip used):
 *    \code
 *     -- TC Capture Waveform Example  xxx --
 *     -- SAMxxxxx-xx
 *     -- Compiled: xxx xx xxxx xx:xx:xx --
 *    \endcode
 * -# Choose the item in the following menu to test.
 *    \code
 *     Menu :
 *     ------
 *       Output waveform property:
 *       0: Set Frequency =  400 Hz, Duty Cycle = 30%
 *       1: Set Frequency =  500 Hz, Duty Cycle = 50%
 *       2: Set Frequency =  800 Hz, Duty Cycle = 75%
 *       3: Set Frequency =  1000 Hz, Duty Cycle = 80%
 *       4: Set Frequency =  4000 Hz, Duty Cycle = 55%
*       -------------------------------------------
*       c: Capture waveform from TC capture channel
*       s: Stop capture and display informations what have been captured
*       h: Display menu
*     ------
*    \endcode
*
* \section References
* - tc_capture_waveform/main.c
*/
/**
 * \file
 *
 * This file contains all the specific code for the tc capture waveform example.
 */


/*----------------------------------------------------------------------------
 *        Headers
 *----------------------------------------------------------------------------*/

#include "board.h"
#include <stdio.h>

/*----------------------------------------------------------------------------
 *        Local definitions
 *----------------------------------------------------------------------------*/
#define PIN_TC_TIOA_OUT   {PIO_PA0B_TIOA0, PIOA, ID_PIOA, PIO_PERIPH_B, PIO_DEFAULT}
#define PIN_TC_TIOA_IN    {PIO_PD21C_TIOA11, PIOD, ID_PIOD, PIO_PERIPH_C, PIO_DEFAULT}

#define TC_WAVE_BASE       TC0
#define TC_WAVE_ID         ID_TC0
#define TC_WAVE_CHANNEL    0

#define TC_CAPTURE_BASE    TC3
#define TC_CAPTURE_ID      ID_TC11
#define TC_CAPTURE_CHANNEL 2
#define TC_Handler         TC11_Handler
#define TC_IRQn            TC11_IRQn
/*----------------------------------------------------------------------------
 *        Types
 *----------------------------------------------------------------------------*/
/** Describes a possible Timer configuration as waveform mode */
struct WaveformConfiguration {
	/** Internal clock signals selection. */
	uint32_t clockSelection;
	/** Waveform frequency (in Hz). */
	uint16_t frequency;
	/** Duty cycle in percent (positive)*/
	uint16_t dutyCycle;
};

/*----------------------------------------------------------------------------
 *        Local variables
 *----------------------------------------------------------------------------*/

/** PIOs for TC0 */
static const Pin pTcPins[] = {PIN_TC_TIOA_OUT, PIN_TC_TIOA_IN};

static const struct WaveformConfiguration waveformConfigurations[] = {

	{TC_CMR_TCCLKS_TIMER_CLOCK4, 400, 30},
	{TC_CMR_TCCLKS_TIMER_CLOCK3, 500, 50},
	{TC_CMR_TCCLKS_TIMER_CLOCK3, 800, 75},
	{TC_CMR_TCCLKS_TIMER_CLOCK2, 1000, 80},
	{TC_CMR_TCCLKS_TIMER_CLOCK2, 4000, 55}
};

/** Current wave configuration*/
static uint8_t configuration = 0;

/** Number of available wave configurations */
const uint8_t numConfigurations = sizeof(waveformConfigurations) /
		sizeof(struct WaveformConfiguration);
/** Capture status*/
static uint32_t _dwCaptured_pulses;
static uint32_t _dwCaptured_ra;
static uint32_t _dwCaptured_rb;
const uint32_t divisors[5] = {2, 8, 32, 128, BOARD_MCK / 32768};

/*----------------------------------------------------------------------------
 *        Local functions
 *----------------------------------------------------------------------------*/

/**
 * \brief Displays the user menu on the DBGU.
 */
static void DisplayMenu(void)
{
	uint8_t i;

	printf("\n\rMenu :\n\r");
	printf("------\n\r");
	printf("  Output waveform property:\n\r");

	for (i = 0; i < numConfigurations; i++) {
		printf("  %d: Set Frequency = %4u Hz, Duty Cycle = %2u%%\n\r",
				i,
				(unsigned int)waveformConfigurations[i].frequency,
				(unsigned int)waveformConfigurations[i].dutyCycle);
	}
	printf("  -------------------------------------------\n\r");
	printf("  c: Capture waveform from TC capture channel \n\r");
	printf("  s: Stop capture and display informations what have been captured \n\r");
	printf("  h: Display menu \n\r");
	printf("------\n\r\n\r");
}


/**
 * \brief Interrupt handler for the TC capture.
 */
void TC_Handler(void)
{
	uint32_t status;
	status = TC_CAPTURE_BASE->TC_CHANNEL[TC_CAPTURE_CHANNEL].TC_SR;

	if ((status & TC_SR_LDRBS) == TC_SR_LDRBS) {
		_dwCaptured_pulses++;
		_dwCaptured_ra = TC_CAPTURE_BASE->TC_CHANNEL[TC_CAPTURE_CHANNEL].TC_RA;
		_dwCaptured_rb = TC_CAPTURE_BASE->TC_CHANNEL[TC_CAPTURE_CHANNEL].TC_RB;
	}
}

/**
 * \brief Configure clock, frequency and dutycycle for TC0 channel 1 in waveform mode.
 */
static void TcWaveformConfigure(void)
{
	uint32_t ra, rc;

	/*  Set channel 1 as waveform mode*/
	TC_WAVE_BASE->TC_CHANNEL[TC_WAVE_CHANNEL].TC_CMR =
		waveformConfigurations[configuration].clockSelection/* Waveform Clock Selection */
		| TC_CMR_WAVE                                    /* Waveform mode is enabled */
		| TC_CMR_ACPA_SET                                /* RA Compare Effect: set */
		| TC_CMR_ACPC_CLEAR                              /* RC Compare Effect: clear */
		| TC_CMR_CPCTRG;                                 /* UP mode with automatic trigger on RC Compare */
	rc = (BOARD_MCK / divisors[waveformConfigurations[configuration].clockSelection]) \
		 / waveformConfigurations[configuration].frequency;
	TC_WAVE_BASE->TC_CHANNEL[TC_WAVE_CHANNEL].TC_RC = rc;
	ra = (100 - waveformConfigurations[configuration].dutyCycle) * rc / 100;
	TC_WAVE_BASE->TC_CHANNEL[TC_WAVE_CHANNEL].TC_RA = ra;
}

/**
 * \brief Configure TC with waveform operating mode.
 */
static void TcWaveformInitialize(void)
{
	/* Configure the PMC to enable the Timer Counter clock for TC wave */
	PMC_EnablePeripheral(TC_WAVE_ID);
	/*  Disable TC clock */
	TC_WAVE_BASE->TC_CHANNEL[TC_WAVE_CHANNEL].TC_CCR = TC_CCR_CLKDIS;
	/*  Disable interrupts */
	TC_WAVE_BASE->TC_CHANNEL[TC_WAVE_CHANNEL].TC_IDR = 0xFFFFFFFF;
	/*  Clear status register */
	TC_WAVE_BASE->TC_CHANNEL[TC_WAVE_CHANNEL].TC_SR;
	/* Configure waveform frequency and duty cycle */
	TcWaveformConfigure();
	/* Enable TC0 channel 0 */
	TC_WAVE_BASE->TC_CHANNEL[TC_WAVE_CHANNEL].TC_CCR =  TC_CCR_CLKEN | TC_CCR_SWTRG;
	printf ("Start waveform: Frequency = %d Hz,Duty Cycle = %2d%%\n\r",
			waveformConfigurations[configuration].frequency,
			waveformConfigurations[configuration].dutyCycle);
}

/**
 * \brief Configure TC with capture operating mode.
 */
static void TcCaptureInitialize(void)
{
	/* Configure the PMC to enable the Timer Counter clock */
	PMC_EnablePeripheral(TC_CAPTURE_ID);
	/*  Disable TC clock */
	TC_CAPTURE_BASE->TC_CHANNEL[TC_CAPTURE_CHANNEL].TC_CCR = TC_CCR_CLKDIS;
	/*  Disable interrupts */
	TC_CAPTURE_BASE->TC_CHANNEL[TC_CAPTURE_CHANNEL].TC_IDR = 0xFFFFFFFF;
	/*  Clear status register */
	TC_CAPTURE_BASE->TC_CHANNEL[TC_CAPTURE_CHANNEL].TC_SR;
	/*  Set channel 2 as capture mode */
	TC_CAPTURE_BASE->TC_CHANNEL[TC_CAPTURE_CHANNEL].TC_CMR = \
		( TC_CMR_TCCLKS_TIMER_CLOCK2 /* Clock Selection */
		| TC_CMR_LDRA_RISING        /* RA Loading Selection: rising edge of TIOA */
		| TC_CMR_LDRB_FALLING       /* RB Loading Selection: falling edge of TIOA */
		| TC_CMR_ABETRG            /* External Trigger Selection: TIOA */
		| TC_CMR_ETRGEDG_FALLING    /* External Trigger Edge Selection: Falling edge */
		);
}

/*----------------------------------------------------------------------------
 *         Global functions
 *----------------------------------------------------------------------------*/
/**
 * \brief Application entry point for tc_capture_waveform example.
 *
 * \return Unused (ANSI-C compatibility).
 */
int main( void )
{
	uint8_t ucKey;
	uint16_t frequence, dutyCycle;

	/* Disable watchdog */
	WDT_Disable(WDT);

	/* Enable I and D cache */
	SCB_EnableICache();
	SCB_EnableDCache();

	/* Output example information */
	printf("-- TC capture waveform example %s --\n\r", SOFTPACK_VERSION);
	printf("-- %s\n\r", BOARD_NAME);
	printf("-- Compiled: %s %s With %s--\n\r", __DATE__, __TIME__, COMPILER_NAME);

	/* Configure PIO Pins for TC0 */
	PIO_Configure( pTcPins, PIO_LISTSIZE(pTcPins));
	/* Configure one TC as waveform operating mode */
	printf("Configure TC channel %d as waveform operating mode \n\r",
			TC_WAVE_CHANNEL);
	TcWaveformInitialize();
	/* Configure one TC channel as capture operating mode */
	printf("Configure TC channel %d as capture operating mode \n\r",
			TC_CAPTURE_CHANNEL);
	TcCaptureInitialize();

	/* Configure TC interrupts */
	NVIC_ClearPendingIRQ(TC_IRQn);
	NVIC_EnableIRQ(TC_IRQn);

	/* Display menu */
	DisplayMenu();

	while (1) {
		ucKey = DBG_GetChar();

		switch (ucKey) {
		case 'h':
			DisplayMenu();
			break;

		case 's':
			if (_dwCaptured_pulses) {
				TC_CAPTURE_BASE->TC_CHANNEL[TC_CAPTURE_CHANNEL].TC_IDR = TC_IDR_LDRBS;
				printf( "Captured %u pulses from TC capture channel , RA = %u, RB = %u \n\r",
						(unsigned int)_dwCaptured_pulses,
						(unsigned int)_dwCaptured_ra,
						(unsigned int)_dwCaptured_rb );

				frequence = (BOARD_MCK / 8) / _dwCaptured_rb;
				dutyCycle = (_dwCaptured_rb - _dwCaptured_ra) * 100 / _dwCaptured_rb;
				printf( "Captured wave frequency = %d Hz, Duty cycle = %d%% \n\r",
						frequence, dutyCycle );
				_dwCaptured_pulses = 0;
				_dwCaptured_ra = 0;
				_dwCaptured_rb = 0;
			} else
				printf("No waveform has been captured\n\r");
			printf("\n\rPress 'h' to display menu\n\r");
			break;

		case 'c':
			printf("Start capture, press 's' to stop \n\r");
			TC_CAPTURE_BASE->TC_CHANNEL[TC_CAPTURE_CHANNEL].TC_IER = TC_IER_LDRBS;
			/* Reset and enable the timer counter for TC capture channel  */
			TC_CAPTURE_BASE->TC_CHANNEL[TC_CAPTURE_CHANNEL].TC_CCR =  TC_CCR_CLKEN |
						TC_CCR_SWTRG;
			break;
		default :
			/* Set waveform configuration #n */
			if ((ucKey >= '0') && (ucKey <= ('0' + numConfigurations - 1 ))) {
				if (!_dwCaptured_pulses ) {
					configuration = ucKey - '0';
					TcWaveformInitialize();
				} else
					printf("In capturing ... , press 's' to stop capture first \n\r");
			}
			break;
		}
	}
}
/** \endcond */
