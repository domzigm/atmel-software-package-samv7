/* ----------------------------------------------------------------------------
 *         SAM Software Package License
 * ----------------------------------------------------------------------------
 * Copyright (c) 2015, Atmel Corporation
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


/*----------------------------------------------------------------------------
 *        Headers
 *----------------------------------------------------------------------------*/
#include "board.h"
#include "loader.h"

/*----------------------------------------------------------------------------
 *        Imported variables
 *----------------------------------------------------------------------------*/

/*----------------------------------------------------------------------------
 *        Exported variables
 *----------------------------------------------------------------------------*/

/*----------------------------------------------------------------------------
 *        Local variables
 *----------------------------------------------------------------------------*/

/*----------------------------------------------------------------------------
 *        Local functions
 *----------------------------------------------------------------------------*/
/**
 * \brief Run Application which is flashed at given address.
 */
static void _RunApplication(uint32_t appAddress)
{
	uint32_t __Start_SP;
	uint32_t (*__Start_New)(void);
	uint32_t *pCode = (uint32_t *)appAddress;

	/* set PC and SP */
	__Start_New = (uint32_t(*) (void) ) pCode[1];
	__Start_SP = pCode[0];

	printf("\n\r Starting application at flash 0x%08x! \n\r", (unsigned int)appAddress);
	printf("========================================================= \n\r");

	WDT_Restart(WDT);
	__disable_irq();
	__set_MSP(__Start_SP);
	__Start_New();
}


/*----------------------------------------------------------------------------
 *        Exported functions
 *----------------------------------------------------------------------------*/
/**
 *  \brief Judge preconditions of applications and run one of them
 *
 *  \param pAppConfig   pointer to AppConfig instances.
 *  \param appNum       Number of applications
 *  \return void
 */
void LOADER_AppSwitch(AppConfig *pAppConfig, uint32_t appNum)
{
	uint32_t i;
	uint32_t rc = 0;
	for (i = 0; i < appNum; i++, pAppConfig++) {
		printf("\n\r APP %d: %s", (int)i, (char*)pAppConfig->appInfo);

		rc = (pAppConfig->funPreCondition) ? pAppConfig->funPreCondition() : 0;

		if ( 0 == rc) {
			printf("\n\r-I- preconditions satisfied.\n\r");
			_RunApplication(pAppConfig->appAddress);
		} else {
			printf("\n\r-I- preconditions not satisfied.\n\r");
		}
	}
	printf("\n\r-I- None of the preconditions are satisfied, applications won't be loaded.");
}

