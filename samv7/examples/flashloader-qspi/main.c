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
 * \page flashloader-qspi QSPI Flash Loader Example
 *
 * \section Purpose
 *
 * This example demonstrates how to setup the QSPI Flash in XIP mode to execute
 * code from QSPI flash.
 *
 * \section Requirements
 *
 * This package can be used with SAMV7x evaluation kits.
 *
 * \section Description
 *
 * Ths code writes the coremark benchmark code into flash via SPI and enables
 * quad mode spi to read code and to execute from it.
 *
 * \section Usage
 *
 *  -# Build the program and download it inside the SAM V71 Xplained Ultra board.
 *     Please refer to the Getting Started with SAM V71 Microcontrollers.pdf
 * -# Optionally, on the computer, open and configure a terminal application
 *    (e.g. HyperTerminal on Microsoft Windows) with these settings:
 *   - 115200 bauds
 *   - 8 bits of data
 *   - No parity
 *   - 1 stop bit
 *   - No flow control
 * -# Start the application.
 * -# Upon startup, the application will output the following lines on the
 *  terminal window:
 *    \code
 *    -- QSPI XIP Example xxx --
 *    -- SAMxxxxx-xx
 *    -- Compiled: xxx xx xxxx xx:xx:xx --
 *    QSPI drivers initialized
 *    \endcode
 * \section References
 * - qspi_flash/main.c
 * - qspi.c
 * - s25fl1.c
 */

/**
 * \file
 *
 * This file contains all the specific code for the Qspi_serialflash example.
 */

/*----------------------------------------------------------------------------
 *        Headers
 *----------------------------------------------------------------------------*/

#include <board.h>
#include <dhry.h>
#include <stdio.h>
#include <assert.h>
#include <string.h>
#include "stdlib.h"


/*----------------------------------------------------------------------------
 *        Local definitions
 *----------------------------------------------------------------------------*/

/** SPI peripheral pins to configure to access the serial flash. */
#define QSPI_PINS        PINS_QSPI

/** Number of consecutive runs of the testloop() function. */
#define NUM_RUNS            150000
/** Defines the number of SLCK periods required to increment the Real-time timer. */
#define RTT_RTPRES          32

/*----------------------------------------------------------------------------
 *        Local variables
 *----------------------------------------------------------------------------*/
/** Pins to configure for the application. */
static Pin pins[] = QSPI_PINS;

/*----------------------------------------------------------------------------
 *        Local variables
 *----------------------------------------------------------------------------*/
static volatile uint32_t tc_Start_Time;
static volatile uint32_t tc_Stop_Time;
static volatile uint32_t rtt_Start_Time;
static volatile uint32_t rtt_Stop_Time;
static volatile uint32_t sysTick_Start_Time;
static volatile uint32_t sysTick_Stop_Time;

static uint32_t clockTick;

/*----------------------------------------------------------------------------
 *         Global functions
 *----------------------------------------------------------------------------*/

/*----------------------------------------------------------------------------
 *        Imported functions
 *----------------------------------------------------------------------------*/

extern void DHRY_testloop(int);


/* Interrupt handler for TC0 interrupt. */
void TC0_Handler(void)
{
    //uint32_t status;
    /* Acknowledge interrupt */
    REG_TC0_SR0;
    /* Increase tick */
    clockTick++;
}

static void start_time(void) {
    TC_Start(TC0, 0);
}

static void stop_time(void) {
    TC_Stop(TC0, 0);
}

/**
 *  Configure Timer Counter 0 to generate an interrupt every 250ms.
 */
static void _ConfigureTc(void)
{
    uint32_t div;
    uint32_t tcclks;

    /** Enable peripheral clock. */
    PMC_EnablePeripheral(ID_TC0);
    /** Configure TC for a 4Hz frequency and trigger on RC compare. */
    TC_FindMckDivisor( 1000, BOARD_MCK, &div, &tcclks, BOARD_MCK );

    TC_Configure( TC0, 0, tcclks | TC_CMR_CPCTRG );
    TC0->TC_CHANNEL[ 0 ].TC_RC = ( BOARD_MCK / div ) / 1000;

    /* Configure and enable interrupt on RC compare */
    NVIC_ClearPendingIRQ(TC0_IRQn);
    NVIC_EnableIRQ(TC0_IRQn);

    TC0->TC_CHANNEL[ 0 ].TC_IER = TC_IER_CPCS;
}


/*----------------------------------------------------------------------------
 *        Local functions
 *----------------------------------------------------------------------------*/

/* Check for NUM_RUNS */
#if (NUM_RUNS >= (0xFFFFFFFF / (32678 / RTT_RTPRES)))
#error "NUM_RUNS is overflow for calculation."
#endif

/**
 * \brief Computes the number of dhrystones per second that the device can produce
 * under the current configuration. Outputs four measures on the terminal.
 */
static void _ComputeDhrystonesPerSecond(void)
{

    uint32_t dhrystonesPerSecond;
    uint32_t dmips;
    uint32_t i;

    printf("Computing dhrystones per second ...\n\r");
#if defined USING_RTT
    /* Configure the RTT */
    RTT_SetPrescaler( RTT, RTT_RTPRES );
#else
    clockTick = 0;
    _ConfigureTc();
#endif

    /* Perform measures */
    for (i=0; i < 4; i++) {
#if defined USING_RTT
        /* Wait for the next second */
        for (j = 0; j < (32768 / RTT_RTPRES); j++) {
            startTime = RTT_GetTime(RTT);
            while (startTime == RTT_GetTime(RTT));
        }
#endif
        /* Go through test loop */
#if defined USING_RT
        rtt_Start_Time = RTT_GetTime(RTT);
        DHRY_testloop(NUM_RUNS);
        rtt_Stop_Time = RTT_GetTime(RTT);
        dhrystonesPerSecond = NUM_RUNS * (32768 / RTT_RTPRES) / (rtt_Stop_Time - rtt_Start_Time);
#else
        start_time();
        tc_Start_Time = clockTick;
        DHRY_testloop(NUM_RUNS);
        stop_time();
        tc_Stop_Time = clockTick;
        dhrystonesPerSecond = NUM_RUNS * 1000 / (tc_Stop_Time - tc_Start_Time);
#endif
        dmips = (dhrystonesPerSecond * 1000) / (1757 * 264);
        printf(" - %u dhrystones per second, ", (unsigned int)dhrystonesPerSecond);
        printf("%u.%03u Dhrystone MIPS/MHz\n\r", (unsigned int)(dmips / 1000),
                                                 (unsigned int)(dmips % 1000));
    }
    printf("Finished\n\r");
}

/**
 * \brief Displays the user menu on the terminal.
 */
static void _DisplayMenu(void)
{
    printf("\n\r");
    printf("=========================================================\n\r");
    printf("Menu: press a key to change the configuration.\n\r");
    printf("=========================================================\n\r");
    printf("  d : \tI/D cache disable.\n\r");
    printf("  e : \tI/D cache enable.\n\r");
    printf("  s : \tStart testing timer\n\r");
    printf("  p : \tStop timer\n\r");
    printf("  m : \tPerform measurements.\n\r");
    printf("  h : \tDisplay this menu again.\n\r");

    printf("---------------------------------------------------------\n\r");
    printf("\n\r");
}

/*----------------------------------------------------------------------------
 *         Global functions
 *----------------------------------------------------------------------------*/

/**
 * \brief Application entry point for Dhrystone test.
 *
 * \return Unused (ANSI-C compatibility).
 */
int main(void)
{
	uint32_t Buffer[4];
    uint8_t key;
    unsigned int tickElapsed;
    /* Disable watchdog */
    WDT_Disable(WDT);

	TimeTick_Configure();

	 /* Initialize the QSPI and serial flash */
	PIO_Configure(pins, PIO_LISTSIZE(pins));
	ENABLE_PERIPHERAL(ID_QSPI);

	S25FL1D_InitFlashInterface(1);
	printf("QSPI drivers initialized\n\r");

	/* enable quad mode */
	S25FL1D_QuadMode(ENABLE);
	S25FL1D_ReadQuadIO(Buffer, sizeof(Buffer), 0, 1, 0 );

    /* Output example information */
    printf("\n\r\n\r\n\r");
    printf("-- Dhrystone Test %s --\n\r", SOFTPACK_VERSION);
    printf("-- %s\n\r", BOARD_NAME);
    printf("-- Compiled: %s %s --\n\r", __DATE__, __TIME__);


    /* Display menu */
    _DisplayMenu();

    while (1) {
        /* Get keypress */
        key = DBG_GetChar();

        switch (key) {
            case 'd':
            case 'D':
                SCB_DisableICache();
                SCB_DisableDCache();
                _DisplayMenu();
                break;
            case 'e':
            case 'E':
                SCB_EnableICache();
                SCB_EnableDCache();
                _DisplayMenu();
                break;

            case 's': case 'S':
                TimeTick_Configure();
                RTT_SetPrescaler( RTT, RTT_RTPRES );
                _ConfigureTc();
                rtt_Start_Time = RTT_GetTime(RTT);
                start_time();
                tc_Start_Time = clockTick;
                sysTick_Start_Time = GetTicks();
                printf("-I- Start counting timer, press 'p' to stop...\r\n");
                break;
            case 'p': case 'P':
                rtt_Stop_Time = RTT_GetTime(RTT);
                stop_time();
                tc_Stop_Time = clockTick;
                sysTick_Stop_Time = GetTicks();
                printf("-I- TC start:%d end:%d elapsed: %d ms\r\n",tc_Start_Time, tc_Stop_Time, (tc_Stop_Time - tc_Start_Time));
                printf("-I- RTT start:%d end:%d elapsed %d ms \r\n",rtt_Start_Time, rtt_Stop_Time, (rtt_Stop_Time - rtt_Start_Time));
                tickElapsed=(unsigned int)(GetDelayInTicks(sysTick_Start_Time, sysTick_Stop_Time));
                printf("-I- SysTick start:%d end:%d elapsed %d ms\r\n",sysTick_Start_Time, sysTick_Stop_Time,tickElapsed);
                break;
            case 'm':
            case 'M':
                _ComputeDhrystonesPerSecond();
                break;

            case 'h':
            case 'H':
                _DisplayMenu();
                break;

            default:
                printf("This menu does not exist !\n\r");
                break;
        }
    }
}


















