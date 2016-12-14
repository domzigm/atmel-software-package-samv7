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
 * \page low_power Low-power Example
 *
 * \section Purpose
 * This example allows to measure the consumption of the core in different modes
 * (sleep mode, wait mode, backup mode).
 *
 * \section Requirements
 *
 *  This package can be used with SAMV71 Xplained Ultra board or SAME70 Xplained board.
 *  Note: the package should run on flash.
 *
 * \section Description
 *
 * At start-up, the program configures all the PIOs as input to avoid parasite
 * consumption. Then a menu is displayed. It allows user to enter in a mode to
 * measure consumption.
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
 *     -- Low_power Example xxx --
 *     -- xxxxxx-xx
 *     -- Compiled: xxx xx xxxx xx:xx:xx --
 *    \endcode
 * -# Press one of the keys listed in the menu to perform the corresponding action.
 *
 * \section References
 * - low_power/main.c
 */

/**
 * \file
 *
 * This file contains all the specific code for the low-power example.
 */

/*----------------------------------------------------------------------------
 *        Headers
 *----------------------------------------------------------------------------*/

#include "board.h"

#include <stdint.h>
#include <stdio.h>

/*----------------------------------------------------------------------------
 *        Local definitions
 *----------------------------------------------------------------------------*/

/* (SCR) Sleep deep bit */
#define SCR_SLEEPDEEP   (0x1 <<  2)

#define CONSOLE_EDBG

/*----------------------------------------------------------------------------
 *        Local variables
 *----------------------------------------------------------------------------*/
/* Wakeup PIN Index definition */
const uint32_t gWakeUpPinId = (1 << WKUP_IDX);

/*----------------------------------------------------------------------------
 *        Local functions
 *----------------------------------------------------------------------------*/

#if defined CONSOLE_EDBG
/**
 * \brief USART ISR for wakeup from sleep mode
 */
void USART1_Handler(void)
{
	USART_GetChar(USART1);
}
#else
void UART0_Handler(void)
{
	UART_GetChar(UART0);
}
#endif

/**
 * \brief Initialize the chip.
 */
static void _InitChip(void)
{
	uint32_t tmp;

	/* All PIO in input mode */
	PIOA -> PIO_ODR = 0xFFFFFFFF;
	PIOB -> PIO_ODR = 0xFFFFFFFF;
	PIOC -> PIO_ODR = 0xFFFFFFFF;
	PIOD -> PIO_ODR = 0xFFFFFFFF;
	PIOE -> PIO_ODR = 0xFFFFFFFF;

	PIOA ->PIO_PPDDR = 0xFFFFFFFF;     // pulldown disable
	PIOA ->PIO_PUDR = 0xFFFFFFFF;      // pullup   disable
	PIOA ->PIO_PPDER = 0;              // pulldown Enable
	PIOA ->PIO_PUER = 0xFFFFFFFF;      // pullup   Enable

	PIOB ->PIO_PPDDR = 0xFFFFFFFF;     // pulldown disable
	PIOB ->PIO_PUDR = 0xFFFFFFFF;      // pullup   disable
	PIOB ->PIO_PPDER = 0x0;            // pulldown Enable
	PIOB ->PIO_PUER = 0xFFFFFFFF;      // pullup   Enable

	PIOC ->PIO_PPDDR = 0xFFFFFFFF;     // pulldown disable
	PIOC ->PIO_PUDR = 0xFFFFFFFF;      // pullup   disable
	PIOC ->PIO_PPDER = 0;              // pulldown Enable
	PIOC ->PIO_PUER = 0xFFFFFFFF;      // pullup   Enable

	PIOD ->PIO_PPDDR = 0xFFFFFFFF;     // pulldown disable
	PIOD ->PIO_PUDR = 0xFFFFFFFF;      // pullup   disable
	PIOD ->PIO_PPDER = 0;              // pulldown Enable
	PIOD ->PIO_PUER = 0xFFFFFFFF;      // pullup   Enable

	PIOE ->PIO_PPDDR = 0xFFFFFFFF;     // pulldown disable
	PIOE ->PIO_PUDR = 0xFFFFFFFF;      // pullup   disable
	PIOE ->PIO_PPDER = 0;              // pulldown Enable
	PIOE ->PIO_PUER = 0xFFFFFFFF;      // pullup   Enable

	tmp = SUPC_MR_KEY_PASSWD;
	tmp |=  ((SUPC -> SUPC_MR) &( ~(1u << 17)));
	SUPC->SUPC_MR |= tmp;

	/* Disable UTMI PLL clock */
	PMC->CKGR_UCKR &= ~CKGR_UCKR_UPLLEN;

	/* Disable PCK */
	PMC->PMC_SCDR = 0xFFFFFFFF;

	/* Disable all the peripheral clocks */
	PMC_DisableAllPeripherals();
}

/**
 * \brief Sets the wake-up inputs for fast start-up mode registers.
 *
 * \param inputs  Wake up inputs to enable.
 */
static void _SetFastStartupInput(uint32_t dwInputs)
{
	PMC->PMC_FSMR &= (uint32_t)~0xFFF00000;
	PMC->PMC_FSMR |= dwInputs;
}

/**
 * \brief Enter Wait Mode.
 * Enter condition: WFE + (SLEEPDEEP bit = 0) + (LPM bit = 1)
 */
static void enterWaitMode(void)
{
	printf("-I- Enter in Wait Mode\n\r");
	printf("-I- Press WAKE UP button to wakeup\n\r");
	/* Configure 4Mhz fast RC oscillator */
	/* First switch MCK to Slow clock  */
	PMC_SetMckSelection(PMC_MCKR_CSS_SLOW_CLK, PMC_MCKR_PRES(0));
	/* Then, enable Fast RC */
	PMC_EnableIntRC4_8_12MHz(CKGR_MOR_MOSCRCF_4_MHz);
	/* Then, Switch MCK to main clock  */
	PMC_SetMckSelection(PMC_MCKR_CSS_MAIN_CLK, PMC_PCK_PRES(0));
	/* Disable unused clock to save power (optional) */
	PMC_SetPllaClock(0, 0);

	/* Set wakeup input for fast start-up */
	_SetFastStartupInput( gWakeUpPinId );
	/* Configure the FLPM field in the PMC Fast Start-up Mode Register(PMC_FSMR).*/
	PMC->PMC_FSMR |= PMC_FSMR_FLPM_FLASH_DEEP_POWERDOWN | PMC_FSMR_LPM;
	/* Set Flash Wait State at 0*/
	EFC->EEFC_FMR = EEFC_FMR_FWS(0);
	/*  No SLEEPDEEP */
	SCB->SCR &= (uint32_t)~SCR_SLEEPDEEP;
	/* Set the WAITMODE bit in PMC Clock Generator Main Oscillator
		Register (CKGR_MOR)*/
	PMC->CKGR_MOR = CKGR_MOR_KEY_PASSWD | CKGR_MOR_WAITMODE;
	/* Wait for MCKRDY = 1 in the PMC Status Register (PMC_SR) */
	while (!(PMC->PMC_SR & PMC_SR_MCKRDY));
	__WFE();

	/* Restore previous clock and DBG */
	PMC_SetMckSelection(PMC_MCKR_CSS_SLOW_CLK, PMC_MCKR_PRES_CLK_1);
	LowLevelInit();
	DBG_Configure(115200, BOARD_MCK);

	printf("-I- Exit Wait Mode\n\r");
}

/**
 * \brief Test Sleep Mode
 */
static void enterSleepMode(void)
{
	printf("-I- Enter in Sleep Mode\n\r");
	printf("-I- Press any key to wakeup\n\r");
	/* The purpose of sleep mode is to optimize power consumption of the
		device versus response time.
	   In this mode, only the core clock is stopped. The peripheral clocks can
	   be enabled.
	   The current consumption in this mode is application-dependent.*/
	PMC->PMC_FSMR &= (uint32_t)~PMC_FSMR_LPM;
	SCB->SCR &= (uint32_t)~SCR_SLEEPDEEP;

	/* Processor wake-up is triggered by an interrupt if the WFI instruction
		of the Cortex-M processor is used.*/
#if defined CONSOLE_EDBG
	NVIC_ClearPendingIRQ(USART1_IRQn);
	NVIC_EnableIRQ(USART1_IRQn);
	USART_EnableIt(USART1,UART_IER_RXRDY);
#else
	NVIC_ClearPendingIRQ(UART0_IRQn);
	NVIC_EnableIRQ(UART0_IRQn);
	UART_EnableIt(UART0,UART_IER_RXRDY);
#endif

	/* This mode is entered using the instruction Wait for Interrupt (WFI).*/
	__WFI();
	printf("-I- Processor wake-up is triggered by an interrupt \n\r");
	printf("-I- Exit Sleep Mode\n\r" );
}

/**
 * \brief Test Backup Mode.
 */
static void enterBackupMode(void)
{
	printf( "-I- Enter in Backup Mode\n\r" );
	printf( "-I- Press WAKE UP button to wakeup\n\r");
	/* Enable the PIO for wake-up */
	SUPC->SUPC_WUIR = (gWakeUpPinId << 16) | gWakeUpPinId;
	PMC->PMC_FSMR   = PMC_FSMR_FLPM_FLASH_DEEP_POWERDOWN | gWakeUpPinId;
	/* Set the SLEEPDEEP bit of Cortex-M processor. */
	SCB->SCR |= SCR_SLEEPDEEP;
	/* Set the VROFF bit of SUPC_CR. */
	SUPC->SUPC_CR = SUPC_CR_KEY_PASSWD | SUPC_CR_VROFF_STOP_VREG;
	SUPC->SUPC_WUMR = (1 << 12);

	/* Wake Up Input 2 and 4 (L_CLICK and R_CLICK) Enable + (0) high to low
		level Transition. Exit from Backup mode occurs as a result of one of
		the following enabled wake-up events:
		WKUPEN0-13 pins (level transition, configurable denouncing)
		Supply Monitor alarm
		RTC alarm
		RTT alarm */
	printf("-I- Processor wake-up is triggered by WAKEUP button \n\r");
	printf("-I- Exit backup Mode\n\r");
	while (1);
}

/**
 * \brief Display test Core menu.
 */
static void _DisplayMenuCore( void )
{
	printf("\n\r");
	printf("===========================================================\n\r");
	printf("Menu: press a key to select low power mode.\n\r");
	printf("===========================================================\n\r");
	printf("Configure:\n\r");
	printf("  S : Sleep mode\n\r");
	printf("  W : Wait mode\n\r");
	printf("  B : Backup mode\n\r");
}

/**
 * \brief Test Core consumption
 */
static void lowPowerMode(void)
{
	uint8_t ucKey;

	while (1) {
		_DisplayMenuCore();
		ucKey = DBG_GetChar();
		switch (ucKey) {
		case 's':
		case 'S':
			enterSleepMode();
			break;

		case 'w':
		case 'W':
			enterWaitMode();
			break;

		case 'b':
		case 'B':
			enterBackupMode();
			break;

		default:
			printf("This menu does not exist !\n\r");
			break;
		} /* switch */
	}
}

/**
 * \brief Application entry point for low-power example.
 *
 * \return Unused (ANSI-C compatibility).
 */
int main(void)
{
	/* Disable watchdog */
	WDT_Disable(WDT);

	/* Enable I and D cache */
	SCB_EnableICache();
	SCB_EnableDCache();

	/* initialize the chip for the power consumption test */
	_InitChip();

	/* Output example information */
	printf("\n\r\n\r\n\r");
	printf("-- Low-power Example %s --\n\r", SOFTPACK_VERSION );
	printf("-- %s\n\r", BOARD_NAME);
	printf("-- Compiled: %s %s With %s--\n\r", __DATE__, __TIME__, COMPILER_NAME);

	/* Test core consumption */
	lowPowerMode();
	return 0;
}
