/* ----------------------------------------------------------------------------
 *         ATMEL Microcontroller Software Support
 * ----------------------------------------------------------------------------
 * Copyright (c) 2010, Atmel Corporation
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * - Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the disclaimer below.
 *
 * Atmel's name may not be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * DISCLAIMER: THIS SOFTWARE IS PROVIDED BY ATMEL "AS IS" AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT ARE
 * DISCLAIMED. IN NO EVENT SHALL ATMEL BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
 * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * ----------------------------------------------------------------------------
 */

/**
 * \page dhrystone Dhrystone Test
 *
 * \section Purpose
 *
 * This application measures how many dhrystone per second that the chip
 * can produce under several different configurations.
 *
 * \section Description
 *
 * When launched, this program displays a menu on the terminal,
 * enabling the user to choose between several options:
 *   - Change the processor & master clock frequencies
 *   - Change the flash access mode 128-bit or 64-bit (if runs out of flash)
 *   - Change the flash wait state (if runs out of flash)
 *   - Start a dhrystone measurement
 *
 * \section Usage
 *
 *  -# Build the program and download it inside the SAM V71 Xplained Ultra board. Please
 *     refer to the Getting Started with SAM V71 Microcontrollers.pdf
 * -# On the computer, open and configure a terminal application
 *    (e.g. HyperTerminal on Microsoft Windows) with these settings:
 *   - 115200 bauds
 *   - 8 bits of data
 *   - No parity
 *   - 1 stop bit
 *   - No flow control
 * -# Start the application.
 * -# In the terminal window, the following text should appear:
 *    \code
 *    -- Dhrystone Test xxx --
 *    -- xxxxxx-xx
 *    -- Compiled: xxx xx xxxx xx:xx:xx --
  *
 * \section References
 * - dhrystone/main.c
 */

/**
 * \file
 *
 * This file contains all the specific code for the dhrystone test.
 */

/*----------------------------------------------------------------------------
 *        Headers
 *----------------------------------------------------------------------------*/

#include "board.h"
#include "dhry.h"
#include <stdint.h>
#include <stdio.h>
#include <stdarg.h>
#include <assert.h>

/*----------------------------------------------------------------------------
 *        Local definitions
 *----------------------------------------------------------------------------*/

/** Number of consecutive runs of the testloop() function. */
#define NUM_RUNS            150000
/** Defines the number of SLCK periods required to increment the Real-time timer. */
#define RTT_RTPRES          32

/*----------------------------------------------------------------------------
 *        Imported functions
 *----------------------------------------------------------------------------*/

extern void DHRY_testloop(int);

/*----------------------------------------------------------------------------
 *        Types
 *----------------------------------------------------------------------------*/

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

/* Interrupt handler for TC0 interrupt. */
void TC0_Handler(void)
{
    //uint32_t status;
    /* Acknowledge interrupt */
    REG_TC0_SR0;
    /* Increase tick */
    clockTick++;
}

void start_time(void) {
    TC_Start(TC0, 0);
}

void stop_time(void) {
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
    
    TC0->TC_CHANNEL[ 0 ].TC_IER = TC_IER_CPCS ;
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


/**
 * \brief Set default master access for speed up.
 * Here assume code is put at flash, data is put at sram.
 */
static void _SetDefaultMaster(void)
{
    Matrix *pMatrix = MATRIX;

    /* Set default master: SRAM0 (slave 0)-> Cortex-M3 System (Master 1) */
    pMatrix->MATRIX_SCFG[0] |= MATRIX_SCFG_FIXED_DEFMSTR(1) | /* Master 1 */
                               MATRIX_SCFG_DEFMSTR_TYPE(2);   /* Fixed Default Master */


    /* Set default master: SRAM1 (slave 1)-> Cortex-M3 System (Master 1) */
    pMatrix->MATRIX_SCFG[1] |= MATRIX_SCFG_FIXED_DEFMSTR(1) | /* Master 1 */
                                 MATRIX_SCFG_DEFMSTR_TYPE(2); /* Fixed Default Master */


    /* Set default master: Internal flash (slave 3) -> Cortex-M3 Instruction/Data (Master 0) */
    pMatrix->MATRIX_SCFG[3] |= MATRIX_SCFG_FIXED_DEFMSTR(0) | /* Master 0 */
                               MATRIX_SCFG_DEFMSTR_TYPE(2);   /* Fixed Default Master */
}
#define EE_TICKS_PER_SEC 1000
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
    uint8_t key;
    unsigned int tickElapsed;
    /* Disable watchdog */
    WDT_Disable(WDT);
  

    /* Output example information */
    printf("\n\r\n\r\n\r");
    printf("-- Dhrystone Test %s --\n\r", SOFTPACK_VERSION);
    printf("-- %s\n\r", BOARD_NAME);
    printf("-- Compiled: %s %s --\n\r", __DATE__, __TIME__);
    _SetDefaultMaster();
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
                sysTick_Start_Time = GetTickCount();
                printf("-I- Start counting timer, press 'p' to stop...\r\n");
                break;
            case 'p': case 'P':
                rtt_Stop_Time = RTT_GetTime(RTT);
                stop_time();
                tc_Stop_Time = clockTick;
                sysTick_Stop_Time = GetTickCount();
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
