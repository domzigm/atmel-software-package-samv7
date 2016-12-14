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
 * \page eefc_pgm EEFC programming example
 *
 * \section Purpose
 * This basic example shows how to use the Enhance Embedded Flash (EEFC)
 * peripheral available on the newest Atmel samv7 Microcontrollers.
 * It details steps required to
 * program the internal flash, and manage secure and lock bits.
 *
 * \section Requirements
 *
 *  This package can be used with SAMV71 Xplained Ultra board or SAME70 Xplained board.
 *
 * \section Description
 * The samv7/e7 ROM code embeds small In Application Programming Procedure.
 * Since this function is executed from ROM, this allows Flash programming
 * (such as sector write) to be done by code running in Flash.\n
 *
 * \section Note
 * The IAP function entry point is retrieved by reading the NMI vector in ROM ().
 * This function takes two argument in parameter: bank index (0 or 1) and the
 * command to be sent to the EEFC.
 *    \code
 *    static uint32_t  (*IAP_PerformCommand)(uint32_t, uint32_t);
 *    IAP_PerformCommand = (uint32_t (*)(uint32_t, uint32_t)) *((uint32_t *)
 *    0x00100008);
 *    IAP_PerformCommand(0, (0x5A << 24) | (argument << 8) | command);
 *    \endcode
 * IAP function returns the value of the MC_FSR register.
 * The required steps are:
 * - Unlock a page.
 * - Program a page of the embedded flash with incremental values
 * (0x0, 0x1, 0x2, 0x3...) by using the IAP function.
 * - Check the flash is correctly programmed by reading all the values programmed.
 * - Lock the page.
 * - Set the security bit.
 *
 * The samv7/e7 features a security bit, based on a specific General Purpose NVM bit 0.
 * When the security is enabled, any access to the Flash, SRAM, Core Registers
 * and Internal Peripherals either through the ICE interface is forbidden.
 * This example will reproduce this scene.
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
 *     -- EEFC Programming Example xxx --
 *     -- xxxxxx-xx
 *     -- Compiled: xxx xx xxxx xx:xx:xx --
 *     -I- Unlocking last page
 *     -I- Writing last page with walking bit pattern
 *     -I- Checking page contents .................. ok
 *     -I- Locking last page
 *     -I- Try to program the locked page...
 *     -I- Please open Segger's JMem program
 *     -I- Read memory at address 0x0043FF00 to check contents
 *     -I- Press any key to continue...
 *     -I- Good job!
 *     -I- Now set the security bit
 *     -I- Press any key to continue to see what happened...
 *     -I- Setting GPNVM #0
 *     -I- All tests done
 *    \endcode
*
* \section References
* - eefc_pgm/main.c
* - efc.c
* - efc.h
* - flashd.c
* - flashd.h
*/
/**
 * \file
 *
 * This file contains all the specific code for the eefc_pgm example.
 *
 */
/*----------------------------------------------------------------------------
 *        Headers
 *----------------------------------------------------------------------------*/

#include "board.h"
#include <stdio.h>
#include <stdarg.h>
#include <assert.h>

/*----------------------------------------------------------------------------
 *         Global functions
 *----------------------------------------------------------------------------*/

/**
 * \brief Application entry point for EEFC programming example.
 *
 * \return Unused (ANSI-C compatibility).
 */

extern int main( void )
{
	uint32_t dwCnt;
	uint8_t  ucError;
	uint32_t adwBuffer[IFLASH_PAGE_SIZE / 4];
	uint32_t dwLastPageAddress;
	volatile uint32_t *pdwLastPageData;

	/* Disable watchdog */
	WDT_Disable(WDT);

	/* Enable I and D cache */
	SCB_EnableICache();
	SCB_EnableDCache();

	 /* Update internal flash Region to Full Access*/
	MPU_UpdateRegions(MPU_DEFAULT_IFLASH_REGION, IFLASH_START_ADDRESS, \
		MPU_AP_FULL_ACCESS |
		INNER_NORMAL_WB_NWA_TYPE( NON_SHAREABLE ) |
		MPU_CalMPURegionSize(IFLASH_END_ADDRESS - IFLASH_START_ADDRESS) |
		MPU_REGION_ENABLE);

	/* Set 6 WS for internal Flash writing (refer to errata) */
	EFC_SetWaitState(EFC, 6);

	/* Output example information */
	printf("\n\r\n\r\n\r");
	printf("EEFC Programming Example %s --\n\r", SOFTPACK_VERSION);
	printf("%s\n\r", BOARD_NAME);
	printf("-- Compiled: %s %s With %s--\n\r", __DATE__, __TIME__, COMPILER_NAME);

	/* Initialize flash driver */
	FLASHD_Initialize(BOARD_MCK, 0);

	/* Performs tests on last page (to avoid overriding existing program).*/
	dwLastPageAddress = IFLASH_ADDR + IFLASH_SIZE - IFLASH_PAGE_SIZE;
	pdwLastPageData = (volatile uint32_t *)dwLastPageAddress;

	/* Unlock page */
	printf("-I- Unlocking last page\n\r");
	ucError = FLASHD_Unlock(dwLastPageAddress, dwLastPageAddress + IFLASH_PAGE_SIZE,
			0, 0);
	assert( !ucError );

	/* Write page with walking bit pattern (0x00000001, 0x00000002, ...) */
	printf("-I- Writing last page with walking bit pattern\n\r");
	for (dwCnt = 0; dwCnt < (IFLASH_PAGE_SIZE / 4); dwCnt++){
		adwBuffer[dwCnt] = 1 << (dwCnt % 32);
	}
	ucError = FLASHD_Write(dwLastPageAddress, adwBuffer, IFLASH_PAGE_SIZE);
	assert(!ucError);

	/* Check page contents */
	printf("-I- Checking page contents ");
	for (dwCnt = 0; dwCnt < (IFLASH_PAGE_SIZE / 4); dwCnt++) {
		printf(".");
		if (pdwLastPageData[dwCnt] != (1u << (dwCnt % 32))) {
			printf("\n\r-F- Expected 0x%08X at address 0x%08X, found 0x%08X\n\r",
					(1u << (dwCnt % 32)), (unsigned int) &(pdwLastPageData[dwCnt]),
					(unsigned)(pdwLastPageData[dwCnt]));
			while (1);
		}
	}
	printf(" OK \n\r");

	/* Lock page */
	printf("-I- Locking last page\n\r");
	ucError = FLASHD_Lock(dwLastPageAddress, dwLastPageAddress + IFLASH_PAGE_SIZE,
			0, 0 );
	assert(!ucError);

	/* Check that associated region is locked*/
	printf("-I- Try to program the locked page... \n\r");
	ucError = FLASHD_Write(dwLastPageAddress, adwBuffer, IFLASH_PAGE_SIZE);
	if (ucError)
		printf("-I- The page to be programmed belongs to a locked region.\n\r");

	printf("-I- Please open Segger's JMem program \n\r");
	printf("-I- Read memory at address 0x%08x to check contents\n\r",
			(unsigned int)dwLastPageAddress);
	printf("-I- Press any key to continue...\n\r");
	while ( !DBG_GetChar());

	printf("-I- Good job!\n\r");
	printf("-I- Now set the security bit \n\r");
	printf("-I- Press any key to continue to see what happened...\n\r");
	while (!DBG_GetChar());

	/* Set GPNVM bit 0 (security bit) */

	/* SAMS7 features a security bit based on the GPNVM bit 0. When security is
		enabled, any access to the Flash, SRAM, core registers and internal
		peripherals, either through the SW-DP/JTAG-DP interface or through the
		Fast Flash Programming Interface, is forbidden. */

	printf("-I- Setting GPNVM #%d\n\r", 0);
	ucError = FLASHD_SetGPNVM(0);
	assert(!ucError);

	printf("-I- All tests done\n\r");

	/* Update internal flash Region to previous configurations */
	MPU_UpdateRegions(MPU_DEFAULT_IFLASH_REGION, IFLASH_START_ADDRESS, \
		MPU_AP_READONLY |
		INNER_NORMAL_WB_NWA_TYPE( NON_SHAREABLE ) |
		MPU_CalMPURegionSize(IFLASH_END_ADDRESS - IFLASH_START_ADDRESS) |
		MPU_REGION_ENABLE);
	return 0;
}

