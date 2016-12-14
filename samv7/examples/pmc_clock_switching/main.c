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
 * \page pmc_clock_switching PMC Clock Switching Example
 *
 * \section Purpose
 * This example shows how to switch system clock from one to another (PLLA,
 * SLCK, MAINCK) or change to fast RC.
 *
 * \section Requirements
 *
 * This package can be used with SAMV71 Xplained Ultra board or SAME70 Xplained board.
 *
 * \section Description
 *
 * Upon startup, the program configure PIOs for DBGU, PCK and buttons. The baud
 * rate of DBGU is configured as 2400 bps. The application prints the current
 * configuration (except 32Khz slow clock ) and waits for button pressed or
 * input from PC terminal application to switch the system clock to next
 * configuration. PCK1 Outputs can be selected from the clocks provided by the
 * clock (PLLA, SLCK, MAINCK) and driven on the pin PCK
 * (Peripheral B).
 * After the clock switches, the PCK1 output signal can be measured by scope
 * compared with the clock configuration.
 *
 * <ul>
 * <li> The Clock Generator integrates a 32,768 Hz low-power oscillator.
 * The user can select the crystal oscillator to be the slow clock source,
 * as it provides a more accurate frequency. The command is made by function
 * PmcSlowClockSwitchXtalOsc().</li>
 * <li> The Master Clock is selected from one of the clocks provided by the
 * Clock Generator.
 * Selecting the Slow Clock provides a Slow Clock signal to the whole device.
 * Selecting the Main Clock saves power consumption of the PLLs.
 * Function _PmcMasterClockSelection() describes the detail how to switch master
 * clock between difference source.</li>
 * </ul>
 * \section Usage
 *
 *  -# Build the program and download it inside the board.
 *     Please refer to the Getting Started with SAM V71/E70 Microcontrollers.pdf
 * -# On the computer, open and configure a terminal application
 *    (e.g. HyperTerminal on Microsoft Windows) with these settings:
 *   - 2400 baud rate
 *   - 8 bits of data
 *   - No parity
 *   - 1 stop bit
 *   - No flow control
 * -# Start the application.
 * -# In the terminal window, the following text should appear:
 *    \code
 *     -- PMC clock Switching Example xxx --
 *     -- SAMxxxxx-xx
 *     -- Compiled: xxx xx xxxx xx:xx:xx --
 *    \endcode
 * -# Press one of the buttons listed in the menu to perform the corresponding
 *    action or type "'" to do the same thing if no button is available.
 *
 * \section References
 * - pmc_clock_switching/main.c
 * - pio.h
 * - pio_it.h
 */

/**
 * \file
 *
 * This file contains all the specific code for the pmc clock switching example.
 */

/*----------------------------------------------------------------------------
 *        Headers
 *----------------------------------------------------------------------------*/

#include "board.h"

#include <stdint.h>
#include <stdio.h>
#include <stdarg.h>

/*----------------------------------------------------------------------------
 *        Local definitions
 *----------------------------------------------------------------------------*/

/** Baud rate used for hints */
#define TEST_BAUDRATE   2400

/*----------------------------------------------------------------------------
 *        Local variables
 *----------------------------------------------------------------------------*/

/** Pin PCK1 (PIO_PD31B_PCK1 Peripheral C) */
const Pin pinPCK = PIN_PCK0;

/** No push button, monitor DBGU RX */
const char* strWaitHint =
"-I- Press ` to switch next clock configuration...\n\r";

/** Mutual semaphore. */
static volatile uint8_t inTesting = 0;

/** Clock backup values */
static uint32_t masterClk;
static uint32_t pllaClk;
static uint32_t masterClkDivision;
static uint32_t pllMultiplier;
static uint32_t masterClkPrescaler;

/*----------------------------------------------------------------------------
 *        Local functions
 *----------------------------------------------------------------------------*/
static void waitKey(void)
{
	printf("-I- Press any key to switch next clock configuration...\n\r");
	while (1) {
		if (DBG_GetChar()!= 0)
			break;
	}
}

static void _DumpPmcConfiguration(void)
{
	uint8_t ucChar[5];
	printf("\n\r========================= PMC ==========================\n\r" );
	printf("Main clock\n\r");
	ucChar[0] = ((PMC->CKGR_MOR & CKGR_MOR_MOSCSEL) == CKGR_MOR_MOSCSEL) ?
			' ' : 'X';
	ucChar[1] = ((PMC->CKGR_MOR & CKGR_MOR_MOSCSEL) == CKGR_MOR_MOSCSEL) ?
			'X' : ' ';
	printf("  On-Chip 12MHz RC oscillator[%c] 12MHz crystal oscillator[%c]\n\r",
			ucChar[0], ucChar[1]);

	printf("  PLLA Multiplier=%u PLLA clock=%uMHz\n\r",
			(unsigned int)pllMultiplier, (unsigned int)(pllaClk/1000000));
	printf("Master clock\n\r");
	ucChar[0] = ((PMC->PMC_MCKR & PMC_MCKR_CSS_Msk) == PMC_MCKR_CSS_SLOW_CLK) ?
			'X' : ' ';
	ucChar[1] = ((PMC->PMC_MCKR & PMC_MCKR_CSS_Msk ) == PMC_MCKR_CSS_MAIN_CLK) ?
			'X' : ' ';
	ucChar[2] = ((PMC->PMC_MCKR & PMC_MCKR_CSS_Msk) == PMC_MCKR_CSS_PLLA_CLK) ?
			'X' : ' ';
	ucChar[3] = ((PMC->PMC_MCKR & PMC_MCKR_CSS_Msk) == PMC_MCKR_CSS_UPLL_CLK) ?
			'X' : ' ';
	printf("  SLOW_CLK [%c], MAIN_CLK[%c], PLLA_CLK[%c] UPLL_CLK[%c] \n\r",
			ucChar[0],ucChar[1],ucChar[2],ucChar[3]);
	printf("  masterClkPrescaler=%u, Master clock=%uMHz \n\r",
			(unsigned int)masterClkPrescaler, (unsigned int)masterClk/1000000);
	printf("Programmable clock\n\r");
	ucChar[0] = ((PMC->PMC_PCK[1] & PMC_PCK_CSS_Msk) == PMC_PCK_CSS_SLOW_CLK) ?
			'X' : ' ';
	ucChar[1] = ((PMC->PMC_PCK[1] & PMC_PCK_CSS_Msk) == PMC_PCK_CSS_MAIN_CLK) ?
			'X' : ' ';
	ucChar[2] = ((PMC->PMC_PCK[1] & PMC_PCK_CSS_Msk) == PMC_PCK_CSS_PLLA_CLK) ?
			'X' : ' ';
	ucChar[3] = ((PMC->PMC_PCK[1] & PMC_PCK_CSS_Msk) == PMC_PCK_CSS_UPLL_CLK) ?
			'X' : ' ';
	ucChar[4] = ((PMC->PMC_PCK[1] & PMC_PCK_CSS_Msk) == PMC_PCK_CSS_MCK) ?
			'X' : ' ';
	printf("  SLOW_CLK [%c], MAIN_CLK[%c], PLLA_CLK[%c] UPLL_CLK[%c] MCK_CLK[%c]\n\r",
			ucChar[0], ucChar[1], ucChar[2], ucChar[3], ucChar[4]);
	printf("==============================================================\n\r");
}

/**
 * \brief Configure DBUG with given master clock, and Configure PCK with given
 *  divider source of master clock and prescaler.
 *
 * \param css  The master clock divider source.
 * \param pres Master Clock prescaler.
 * \param clk frequency of the master clock (in Hz).
 */
static void _ConfigureDbguAndPck(uint32_t css, uint32_t pres, uint32_t clk)
{
	/* Configure DBGU baud rate as 1200 bps (except slow clock)*/
	if (clk > 32768) {
		DBG_Configure(TEST_BAUDRATE, clk);
	}
	/* Disable programmable clock 1 output */
	REG_PMC_SCDR = PMC_SCER_PCK0;
	/* Enable the DAC master clock */
	PMC->PMC_PCK[0] = css | pres;
	/* Enable programmable clock 1 output */
	REG_PMC_SCER = PMC_SCER_PCK0;
	/* Wait for the PCKRDY1 bit to be set in the PMC_SR register*/
	while ((REG_PMC_SR & PMC_SR_PCKRDY0) == 0 );
	Wait(50);
}


static void _calcPmcParam(void)
{
	uint32_t onChipRC;
	if (PMC->CKGR_MOR & CKGR_MOR_MOSCSEL) {
		pllMultiplier = (PMC->CKGR_PLLAR & CKGR_PLLAR_MULA_Msk) >> CKGR_PLLAR_MULA_Pos;
		pllaClk = BOARD_MAINOSC * (pllMultiplier + 1);
		switch (PMC->PMC_MCKR & PMC_MCKR_MDIV_Msk) {
		case PMC_MCKR_MDIV_EQ_PCK:
			masterClkDivision = 1;
			break;
		case PMC_MCKR_MDIV_PCK_DIV2:
			masterClkDivision = 2;
			break;
		case PMC_MCKR_MDIV_PCK_DIV4:
			masterClkDivision = 4;
			break;
		case PMC_MCKR_MDIV_PCK_DIV3:
			masterClkDivision = 3;
			break;
		default:
			masterClkDivision = 3;
		}
		masterClkPrescaler = 1 << ((PMC->PMC_MCKR & PMC_MCKR_PRES_Msk) >>
					PMC_MCKR_PRES_Pos);
		masterClk = pllaClk / masterClkDivision / masterClkPrescaler;
	} else {
		onChipRC = (PMC->CKGR_MOR & CKGR_MOR_MOSCRCF_Msk);
		masterClk = ((onChipRC == CKGR_MOR_MOSCRCF_12_MHz)? 12 :
				((onChipRC == CKGR_MOR_MOSCRCF_8_MHz) ? 8 : 4)) * 1000000;
	}
}

/*----------------------------------------------------------------------------
 *         Global functions
 *----------------------------------------------------------------------------*/
/**
 * \brief Application entry point for pmc_clock switch example.
 *
 * \return Unused (ANSI-C compatibility).
 */
extern int main( void )
{
	/* Disable watchdog */
	WDT_Disable(WDT);

	/* Enable I and D cache */
	SCB_EnableICache();
	SCB_EnableDCache();

	DBG_Configure(TEST_BAUDRATE, BOARD_MCK);

	/* Output example information */
	printf("-- PMC Clock Switching Example %s --\n\r", SOFTPACK_VERSION);
	printf("-- %s\n\r", BOARD_NAME);
	printf("-- Compiled: %s %s With %s--\n\r", __DATE__, __TIME__, COMPILER_NAME);
	/* Configure PCK as peripheral */
	PIO_Configure(&pinPCK, 1);
	TimeTick_Configure();

	_calcPmcParam();
	_ConfigureDbguAndPck(PMC_PCK_CSS_MAIN_CLK, PMC_PCK_PRES(0), masterClk);
	printf("\n\r --- Current PMC clock from start-up configuration --- \n\r");
	_DumpPmcConfiguration();
	printf("-I- Please measure external 12MHz main clock on PCK1 ...\n\r");
	waitKey();

	printf("-I- Please measure external PLL/32 clock on PCK1 ...\n\r");
	_ConfigureDbguAndPck(PMC_PCK_CSS_PLLA_CLK, (31 << PMC_PCK_PRES_Pos), masterClk);
	waitKey();

	printf("\n\r --- Change PLL clock --- \n\r");
	PMC_ConfigureMckWithPlla(0xE, 0x1, PMC_MCKR_PRES_CLK_1);
	_calcPmcParam();
	_ConfigureDbguAndPck(PMC_PCK_CSS_PLLA_CLK, (31 << PMC_PCK_PRES_Pos), masterClk);
	_DumpPmcConfiguration();
	printf("-I- Please measure external PLL/32 clock on PCK1 ...\n\r");
	waitKey();

	printf("\n\r-I- Switch to On-chip 8MHz RC oscillator  \n\r");
	PMC_SetMckSelection(PMC_MCKR_CSS_SLOW_CLK, PMC_MCKR_PRES_CLK_1);
	PMC_EnableIntRC4_8_12MHz(CKGR_MOR_MOSCRCF_8_MHz);
	PMC_SetMckSelection(PMC_MCKR_CSS_MAIN_CLK, PMC_MCKR_PRES_CLK_1);
	_calcPmcParam();
	_ConfigureDbguAndPck(PMC_PCK_CSS_MAIN_CLK, PMC_PCK_PRES(0), masterClk);
	printf("-I- Please measure on-chip 8MHz main clock on PCK1 ...\n\r");
	waitKey();

	printf("\n\r-I- Switch to On-chip 12MHz RC oscillator  \n\r");
	PMC_SetMckSelection(PMC_MCKR_CSS_SLOW_CLK, PMC_MCKR_PRES_CLK_1);
	PMC_EnableIntRC4_8_12MHz(CKGR_MOR_MOSCRCF_12_MHz);
	PMC_SetMckSelection(PMC_MCKR_CSS_MAIN_CLK, PMC_MCKR_PRES_CLK_1);
	_calcPmcParam();
	_ConfigureDbguAndPck(PMC_PCK_CSS_MAIN_CLK, PMC_PCK_PRES(0), masterClk);
	printf("-I- Please measure on-chip 12MHz main clock on PCK1 ...\n\r");
	waitKey();
	printf("\n\r-I- Switch to slow clock  \n\r");
	printf("-I- Please measure slow clock on PCK1 ...\n\r");
	printf("-I- The end \n\r");
	PMC_SetMckSelection(PMC_MCKR_CSS_SLOW_CLK, PMC_MCKR_PRES_CLK_1);
	_ConfigureDbguAndPck(PMC_PCK_CSS_SLOW_CLK, PMC_PCK_PRES(0), masterClk);
	return 0;
}
