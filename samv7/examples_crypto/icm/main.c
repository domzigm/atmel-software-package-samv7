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
 *  \page icm Integrity Check Monitor Example
 *
 *  \section Purpose
 *  This application demonstrates The Integrity Check Monitor (ICM)
 *  peripheral integrated in some samv7 microcontrollers family. The ICM
 * controller integrates two modes of operation. The first one is used to hash
 * a list of memory regions and save the digests to memory (ICM Hash Area).
 * The second operation mode is an active monitoring of the memory.
 *
 * \section Requirements
 *
 *  This package can be used with SAMV71 Xplained Ultra board or SAME70 Xplained board.
 *
 * \section Description
 * This example shows how to configure ICM to performs SHA-based memory hashing
 * over memory regions. When the ICM module is enabled, it sequentially
 * retrieves a circular list of region descriptors from the memory. Up to 4
 * regions may be monitored.
 *
 *  \section Usage
 *
 *  -# Build the program and download it inside the board.
 *     Please refer to the Getting Started with SAM V71/E70 Microcontrollers.pdf
 *  -# On the computer, open and configure a terminal application
 *     (e.g. HyperTerminal on Microsoft Windows) with these settings:
 *    - 115200 baud rate
 *    - 8 bits of data
 *    - No parity
 *    - 1 stop bit
 *    - No flow control
 *  -# In the terminal window, the
 *     following text should appear (values depend on the board and chip used):
 *     \code
 *      -- ICM Example xxx --
 *      -- SAMxxxxx-xx
 *      -- Compiled: xxx xx xxxx xx:xx:xx --
 *      \endcode
 *  -# Input command according to the menu.
 *
 * \section References
 * - icm/main.c
 * - icm.h
 * - icm.h
 */

/** \file
 *
 *  This file contains all the specific code for the ICM
 */

/*----------------------------------------------------------------------------
 *        Headers
 *----------------------------------------------------------------------------*/
#include <board.h>
#include <string.h>

/*----------------------------------------------------------------------------
 *        Local definitions
 *----------------------------------------------------------------------------*/
#define LEN_STRING_0      3
#define LEN_STRING_1      56
#define LEN_STRING_LONG   (1000000)

#define MSG_0_LENGTH      16
#define MSG_1_LENGTH      32
#define MSG_LONG_LENGTH   15626

#define ID_REGION0      0x01
#define ID_REGION1      0x02
#define ID_REGION2      0x04
#define ID_REGION3      0x08

/*----------------------------------------------------------------------------
 *        Local variables
 *----------------------------------------------------------------------------*/
static uint32_t *bufOutput;
static uint32_t *bufContext;
static volatile uint32_t regionHashCompleted, regionDigestMismatch;

static uint8_t msgRegion_0[LEN_STRING_0]   = "abc";
static uint8_t msgRegion_1[LEN_STRING_1]   =
	"abcdbcdecdefdefgefghfghighijhijkijkljklmklmnlmnomnopnopq";
static uint8_t msgRegion_mis1[LEN_STRING_1] =
	"aacdbcdecdefdefgefghfghighijhijkijkljklmklmnlmnomnopnopq";
/* Let the message M be the binary-coded form of the ASCII string which consists
	of 1,000,000 repetitions of the character ¡°a¡±. */
static uint8_t msgRegion_2 = 'a';

/*----------------------------------------------------------------------------
 *        Local functions
 *----------------------------------------------------------------------------*/

/**
 * \brief ICM interrupt hander.
 */
void ICM_Handler(void)
{
	uint32_t status;
	status = ICM_GetIntStatus();

	if (status & ICM_ISR_RHC_Msk) {
		regionHashCompleted |= (status & ICM_ISR_RHC_Msk);
		ICM_DisableIt(status & ICM_ISR_RHC_Msk);
	}

	if (status & ICM_ISR_RDM_Msk) {
		regionDigestMismatch |= ((status & ICM_ISR_RDM_Msk) >> ICM_ISR_RDM_Pos);
		ICM_DisableIt((status & ICM_ISR_RDM_Msk) >> ICM_ISR_RDM_Pos);
	}
}

/**
 * \brief Generate message for given ASCII string.
 * \param M pointer to string
 * \param message pointer to buffer to store generated message.
 * \param length string length in byte.
 * \param longMsg 1: long message otherwise 0.
 * \note Maximum length is 32-bits only.
 */
static void _buildMessage32(uint8_t *M, uint32_t *message, uint32_t length,
							uint8_t longMsg)
{
	uint32_t l;
	uint32_t l_high, l_low;
	uint32_t k;
	uint8_t *pBuf;
	pBuf = (uint8_t *)message;
	l = length * 8;
	k = ((512 + 448) - ( (l % 512) + 1)) % 512;

	if (longMsg)
		memset(pBuf, (*M), length);
	else
		memcpy(pBuf, M, length);

	pBuf += length;
	/* Append the bit '0' to the end of the message*/
	*pBuf++ = 0x80;
	/* followed by  k zero bits */
	memset(pBuf, 0, (k - 7) / 8 );
	/* Then append the 64-bit block length*/
	pBuf += (k - 7 ) / 8;
	l_high = 0;
	l_low = (uint32_t)(l & 0xffffffff);
	*pBuf++ = (l_high >> 24) & 0xFF;
	*pBuf++ = (l_high >> 16) & 0xFF;
	*pBuf++ = (l_high >> 8) & 0xFF;
	*pBuf++ = (l_high ) & 0xFF;
	*pBuf++ = (l_low >> 24) & 0xFF;
	*pBuf++ = (l_low >> 16) & 0xFF;
	*pBuf++ = (l_low >> 8) & 0xFF;
	*pBuf++ = (l_low ) & 0xFF;
}

/*----------------------------------------------------------------------------
 *        Exported functions
 *----------------------------------------------------------------------------*/

/**
 *  \brief ICM Application entry point.
 *
 *  \return Unused (ANSI-C compatibility).
 *  \callgraph
 */
int main(void)
{
	uint32_t mainListAddr;

	LinkedListDescriporIcmRegion *pMainList;
	/* Disable watchdog */
	WDT_Disable(WDT);

	/* Output example information */
	printf("-- ICM Example %s --\n\r", SOFTPACK_VERSION );
	printf("-- %s\n\r", BOARD_NAME);
	printf("-- Compiled: %s %s With %s--\n\r", __DATE__, __TIME__ ,
			COMPILER_NAME);

	/* Enable SDRAM */
	BOARD_ConfigureSdram();

	bufOutput = (uint32_t *)SDRAM_CS_ADDR;
	bufContext = (uint32_t *)(SDRAM_CS_ADDR + 0x100000);
	pMainList = (LinkedListDescriporIcmRegion *)((uint32_t *)(
					SDRAM_CS_ADDR + 0x20000));
	mainListAddr = (uint32_t)pMainList;

	/* Enable ICM peripheral clock */
	PMC_EnablePeripheral(ID_ICM);

	/* A software triggered hardware reset of the ICM interface is performed */
	ICM_SoftReset();
	/* Configure and enable ICM interrupt */
	NVIC_ClearPendingIRQ(ICM_IRQn);
	NVIC_EnableIRQ(ICM_IRQn);

	ICM_GetIntStatus();
	ICM_GetStatus();

	/* Build message */
	_buildMessage32(msgRegion_0, bufContext, LEN_STRING_0, 0);
	_buildMessage32(msgRegion_1, bufContext + MSG_0_LENGTH, LEN_STRING_1, 0);
	_buildMessage32(&msgRegion_2, bufContext + MSG_0_LENGTH + MSG_1_LENGTH, \
					LEN_STRING_LONG, 1);

	ICM_Configure(ICM_CFG_UALGO_SHA1 | ICM_CFG_SLBDIS);
	pMainList->icm_raddr = (uint32_t)bufContext;
	pMainList->icm_rcfg = ICM_RCFG_ALGO_SHA1;
	pMainList->icm_rctrl = MSG_0_LENGTH / 16 - 1;
	pMainList->icm_rnext = 0;
	pMainList++;
	pMainList->icm_raddr = (uint32_t)(bufContext + MSG_0_LENGTH);
	pMainList->icm_rcfg = ICM_RCFG_ALGO_SHA1;
	pMainList->icm_rctrl = MSG_1_LENGTH / 16 - 1;
	pMainList->icm_rnext = 0;
	pMainList++;
	pMainList->icm_raddr = (uint32_t)(bufContext + MSG_0_LENGTH + MSG_1_LENGTH);
	pMainList->icm_rcfg = ICM_RCFG_EOM | ICM_RCFG_ALGO_SHA1;
	pMainList->icm_rctrl = LEN_STRING_LONG / 4 / 16 - 1;
	pMainList->icm_rnext = 0;

	ICM_SetDescStartAddress((uint32_t)mainListAddr);
	ICM_SetHashStartAddress((uint32_t)bufOutput);
	regionHashCompleted = 0;
	regionDigestMismatch = 0;
	printf("-I- Enable ICM region(0-2)...\n\r");
	ICM_EnableIt(ICM_IER_RHC(ID_REGION0 | ID_REGION1 | ID_REGION2));
	ICM_Enable();

	while (regionHashCompleted != (ID_REGION0 | ID_REGION1 | ID_REGION2));

	printf("-I- When the desired number of blocks have been transferred, \
	the digest is whether moved to memory (write-back function) \n\r");
	ICM_Disable();

	regionHashCompleted = 0;
	regionDigestMismatch = 0;
	printf("-I- Configure ICM region(0-2) with compare function\n\r");
	printf("-I- When the desired number of blocks have been transferred, the \
	digest is compared with a digest reference located in system memory. \n\r");
	ICM_Configure(ICM_CFG_UALGO_SHA1 | ICM_CFG_SLBDIS | ICM_CFG_WBDIS);
	ICM_EnableIt(ICM_IER_RHC(ID_REGION0 | ID_REGION1 | ID_REGION2)
					| ICM_IER_RDM(ID_REGION0 | ID_REGION1 | ID_REGION2));
	printf("-I- Enable ICM region(0-2)...\n\r");
	ICM_Enable();

	while (regionHashCompleted != (ID_REGION0 | ID_REGION1 | ID_REGION2));

	if (regionDigestMismatch == 0)
		printf("-I- No digest mismatch occurs!\n\r");

	ICM_Disable();

	printf("-I- Change the context in region 1 for test...\n\r");
	_buildMessage32(msgRegion_mis1, bufContext + MSG_0_LENGTH, LEN_STRING_1, 0);
	ICM_Configure(ICM_CFG_UALGO_SHA1 | ICM_CFG_SLBDIS | ICM_CFG_WBDIS);
	regionHashCompleted = 0;
	regionDigestMismatch = 0;
	ICM_EnableMonitor((ID_REGION0 | ID_REGION1 | ID_REGION2));
	ICM_EnableIt(ICM_IER_RHC(ID_REGION0 | ID_REGION1 | ID_REGION2)
					| ICM_IER_RDM(ID_REGION0 | ID_REGION1 | ID_REGION2));
	ICM_Enable();

	while (regionHashCompleted != (ID_REGION0 | ID_REGION1 | ID_REGION2));

	printf("-I- Digest mismatch occurs @ region(s) %x \n\r",
			(unsigned int)regionDigestMismatch);
	ICM_Disable();

	printf("\n\r-I- ICM test done!\n\r");

	while (1);
}

