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
 *  \page $page_name$ $example_name$ Example
 *
 *  \section Purpose
 *
 *  $example_purpose$
 *
 *  \section Requirements
 *
 *  $example_requirements$
 *
 *  \section Description
 *
 *  $example_description$
 *
 *  \section Usage
 *
 *  -# Build the program and download it inside the SAM V71 Xplained Ultra board. Please
 *     refer to the Getting Started with SAM V71 Microcontrollers.pdf
 *  -# On the computer, open and configure a terminal application
 *     (e.g. HyperTerminal on Microsoft Windows) with these settings:
 *    - 115200 bauds
 *    - 8 bits of data
 *    - No parity
 *    - 1 stop bit
 *    - No flow control
 *  -# In the terminal window, the
 *     following text should appear (values depend on the board and chip used):
 *     \code
 *      -- $example_name$ Example xxx --
 *      -- AT91xxxxxx-xx
 *      -- Compiled: xxx xx xxxx xx:xx:xx --
 *      -- Menu Choices for this example--
 *      -- s: Set new DAC0 output voltage.--
 *      -- v: Get voltage on potentiometer.--
 *      -- m: Display this menu again.--
 *      \endcode
 *  -# Input command according to the menu.
 *
 *  \section References
 *  $example_references$
 */

/** \file
 *
 *  This file contains all the specific code for the $example_name$
 */

/*----------------------------------------------------------------------------
 *        Headers
 *----------------------------------------------------------------------------*/

#include <stdint.h>
#include <stdio.h>

/*----------------------------------------------------------------------------
 *        Local definitions
 *----------------------------------------------------------------------------*/


/*----------------------------------------------------------------------------
 *        Local variables
 *----------------------------------------------------------------------------*/

/*----------------------------------------------------------------------------
 *        Local functions
 *----------------------------------------------------------------------------*/

/*----------------------------------------------------------------------------
 *        Exported functions
 *----------------------------------------------------------------------------*/

/**
 *  \brief $page_name$ Application entry point.
 *
 *  \return Unused (ANSI-C compatibility).
 *  \callgraph
 */
int main( void )
{
    /* Disable watchdog */
    WDT_Disable( WDT ) ;

    /* Output example information */
    printf("-- $example_name$ Example %s --\n\r", SOFTPACK_VERSION ) ;
    printf("-- %s\n\r", BOARD_NAME);
    printf("-- Compiled: %s %s --\n\r", __DATE__, __TIME__);

    while( 1 )
    {
    }

}

