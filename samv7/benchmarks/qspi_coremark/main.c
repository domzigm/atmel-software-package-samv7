/* ----------------------------------------------------------------------------
 *         SAM Software Package License 
 * ----------------------------------------------------------------------------
 * Copyright (c) 2014, Atmel Corporation
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
 * \page spi_serialflash SPI with Serialflash Example
 *
 * \section Purpose
 *
 * This example demonstrates how to setup the QSPI Flash in order to Write a benchmark code and run the code from QSPI flash.
 *
 * \section Requirements
 *
 * This package can be used with SAMV7x evaluation kits.
 *
 * \section Description
 *
 * Ths code writes the coremark benchmark code into flash via SPI and enables quad mode spi to read code and to execute from it.
 *
 * \section Usage
 *
 *  -# Build the program and download it inside the SAM V71 Xplained Ultra board. Please
 *     refer to the Getting Started with SAM V71 Microcontrollers.pdf
 * -# Optionally, on the computer, open and configure a terminal application
 *    (e.g. HyperTerminal on Microsoft Windows) with these settings:
 *   - 115200 bauds
 *   - 8 bits of data
 *   - No parity
 *   - 1 stop bit
 *   - No flow control
 * -# Start the application.
 * -# Upon startup, the application will output the following lines on the terminal window:
 *    \code
 *    -- QSPI Serialflash Example xxx --
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
#include <stdio.h>
#include <assert.h>
#include <string.h>
#include "stdlib.h"
#include "code_coremark.h"

/*----------------------------------------------------------------------------
 *        Local definitions
 *----------------------------------------------------------------------------*/

/** Maximum device page size in bytes. */
#define MAXPAGESIZE     256

/** SPI peripheral pins to configure to access the serial flash. */
#define QSPI_PINS        PINS_QSPI
   
/*----------------------------------------------------------------------------
 *        Local variables
 *----------------------------------------------------------------------------*/
/** Pins to configure for the application. */
static Pin pins[] = QSPI_PINS;

uint8_t *pCoreMarkCodeImage = pBuffercode;

/*----------------------------------------------------------------------------
 *         Global functions
 *----------------------------------------------------------------------------*/

/**
 * \brief Application entry point for SPI with Serialflash example.
 * Initializes the serial flash and performs several tests on it.
 *
 * \return Unused (ANSI-C compatibility).
 */

int main(void)
{
    uint32_t __Iar_Start, i;
    uint8_t Buffer[4];
    uint32_t *pQspiBuffer = (uint32_t *)QSPIMEM_ADDR;

    /* Disable watchdog */
    WDT_Disable( WDT0 ) ;
    WDT_Disable( WDT1 ) ;

     /* Output example information */
    printf( "-- QSPI Serialflash Example %s --\n\r", SOFTPACK_VERSION ) ;
    printf( "-- %s\n\r", BOARD_NAME ) ;
    printf( "-- Compiled: %s %s --\n\r", __DATE__, __TIME__ ) ;
    SCB_EnableICache();        
    SCB_EnableDCache();
    TimeTick_Configure();
    
     /* Initialize the SPI and serial flash */
    PIO_Configure(pins, PIO_LISTSIZE(pins));
    ENABLE_PERIPHERAL(ID_QSPI);
    QSPI_Configure(QSPI, (QSPI_MR_SMM_MEMORY | QSPI_MR_CSMODE_SYSTEMATICALLY | QSPI_MR_DLYCS(1000) ));
    QSPI_ConfigureClock(QSPI, (QSPI_SCR_SCBR(1) | QSPI_SCR_DLYBS(1000)) );    

    QSPI_Enable(QSPI); 
    
    S25FL1D_InitFlashInterface();
    printf("QSPI drivers initialized\n\r");

    
    /* erase entire chip  */
    S25FL1D_EraseChip;
      
    S25FL1D_Write(pBuffercode, 15355, 0);     
    
    S25FL1D_EnableQuadMode();

    
    S25FL1D_ReadQuadIO(Buffer, 50, 0, 1);  
    
    __Iar_Start = *(uint32_t *)(0X80000004);    
    asm("mov pc, r0");
    
    while(1);
 
}
