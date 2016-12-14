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
 *  \page xdma XDMA example
 *
 *  \section Purpose
 *
 *  The xdma example will help new users get familiar with Atmel's
 *  SAMV7 family of microcontrollers's XDMA peripheral.
 *
 *  \section Requirements
 *
 *  This package can be used with SAMV71 Xplained Ultra board or SAME70 Xplained board.
 *
 *  \section Description
 *
 *  This example proposes different configuration possible by XDMA and after
 *  configuring it user
 *  can start memory to memory transfer.
 *  Memory transfer result varies according to configuration
 *
 *  \section Usage
 *
 *  -# Build the program and download it inside the board.
 *  Please refer to the Getting Started with SAM V71/E70 Microcontrollers.pdf
 *  -# On the computer, open and configure a terminal application
 *     (e.g. HyperTerminal on Microsoft Windows) with these settings:
 *    - 115200 baud rate
 *    - 8 bits of data
 *    - No parity
 *    - 1 stop bit
 *    - No flow control
 *  -# Start the application.
 *  -# The following text should appear (values depend on the board and chip used):
 *     \code
 *      -- XDMA example xxx --
 *      -- xxxxxx-xx
 *      -- Compiled: xxx xx xxxx xx:xx:xx --
 *     \endcode
 *
 *  \section References
 *  - xdma/main.c
 *  - xdmac.c
 *  - xdmad.c
 *  - xdma_hardware_interface.c
 */


/** \file
 *
 *  This file contains all the specific code for the DMA example.
 *
 */


/*----------------------------------------------------------------------------
 *        Headers
 *----------------------------------------------------------------------------*/

#include <board.h>
#include <string.h>

/*----------------------------------------------------------------------------
 *         Local constants
 *----------------------------------------------------------------------------*/
#define XDMA_SINGLE          1
#define XDMA_MULTI           2
#define XDMA_LLI             3

/** Maximum size of Linked List Item  in this example*/
#define MAX_LLI_SIZE         2
/** Micro-block length for single transfer  */
#define MICROBLOCK_LEN       16
/** Buffer length */
#define BUFFER_LEN         128
/** Polling or interrupt mode */
#define POLLING_MODE   0

/*----------------------------------------------------------------------------
 *        Local variables
 *----------------------------------------------------------------------------*/
/** Global DMA driver instance for all DMA transfers in application. */
static sXdmad xdmad;
static sXdmadCfg xdmadCfg;

/** TX descriptors list */
COMPILER_ALIGNED(32) static LinkedListDescriporView1 LLIview1[MAX_LLI_SIZE];

/* DMA driver instance */
static uint32_t dmaChannel;


COMPILER_ALIGNED(32) static uint8_t sourceBuffer[512];

COMPILER_ALIGNED(32) static uint8_t destinationBuffer[512];

/* Current Programming DMAC mode for Multiple Buffer Transfers */
static uint8_t dmaProgrammingMode = 0;
static uint8_t ConfigFlag = 0;
static uint8_t dmaDataWidth = 0;
static uint8_t dmaSourceAddrMode = 0;
static uint8_t dmaDestAddrMode = 0;
static uint8_t dmaMemSet = 0;

/*----------------------------------------------------------------------------
 *         Local functions
 *----------------------------------------------------------------------------*/

/**
 * \brief Dump buffer to DBGU
 *
 */
static void _DumpBufferInfo(uint8_t* pcBuffer)
{
	uint32_t i = 0;

	while (i < BUFFER_LEN) {
		printf("%02x ", pcBuffer[i++]);
		if ((i % 16 == 0))
			printf("\n\r");
	}
	printf("\n\r");
}

/**
 * \brief Display main menu.
 */
static void _displayMenu(void)
{
	uint8_t ucChar[4];

	printf("\n\rxDMA Menu :\n\r");
	printf("\n\r|====== Channel Configuration ================================|\n\r");
	printf("| Press [a|b|c|d] to set Date width                           |\n\r");
	ucChar[0] = (dmaDataWidth == 0) ? 'X' : ' ';
	ucChar[1] = (dmaDataWidth == 1) ? 'X' : ' ';
	ucChar[2] = (dmaDataWidth == 2) ? 'X' : ' ';
	printf("|   a: BYTE[%c] b: HALFWORD[%c] c: WORD[%c]                      |\n\r",
			ucChar[0],ucChar[1],ucChar[2]);
	printf("| Press [0|1|2|3] to set Source Addressing Mode               |\n\r");
	ucChar[0] = (dmaSourceAddrMode == 0) ? 'X' : ' ';
	ucChar[1] = (dmaSourceAddrMode == 1) ? 'X' : ' ';
	ucChar[2] = (dmaSourceAddrMode == 2) ? 'X' : ' ';
	ucChar[3] = (dmaSourceAddrMode == 3) ? 'X' : ' ';
	printf("|   0: FIXED[%c] 1: INCR[%c] 2: AM[%c] 3: DS_AM[%c]               |\n\r",
			ucChar[0],ucChar[1],ucChar[2],ucChar[3]);
	printf("| Press [4|5|6|7] to set Destination Addressing Mode          |\n\r");
	ucChar[0] = (dmaDestAddrMode == 0) ? 'X' : ' ';
	ucChar[1] = (dmaDestAddrMode == 1) ? 'X' : ' ';
	ucChar[2] = (dmaDestAddrMode == 2) ? 'X' : ' ';
	ucChar[3] = (dmaDestAddrMode == 3) ? 'X' : ' ';
	printf("|   4: FIXED[%c] 5: INCR[%c] 6: AM[%c] 7: DS_AM[%c]               |\n\r",
			ucChar[0],ucChar[1],ucChar[2],ucChar[3]);
	printf("| Press [8|9| to set MEMSET Mode                              |\n\r");
	ucChar[0] = (dmaMemSet == 0) ? 'X' : ' ';
	ucChar[1] = (dmaMemSet == 1) ? 'X' : ' ';
	printf("|   8: NORMAL Mode[%c] 9: HW_MODE[%c]                           |\n\r",
			ucChar[0],ucChar[1]);
	printf("|=============================================================|\n\r");
	printf("\n\r- xDMA transfer type \n\r");
	printf("    S: Single Block with Single Micro-block transfer\n\r" );
	printf("    M: Single Block with Multiple Micro-block transfer  \n\r" );
	printf("    L: Linked List Master transfer\n\r" );
	printf("- t: Start DMA transfer\n\r");
	printf("- h: Display this menu\n\r");
	printf("\n\r");
}

/**
 * \brief Programming DMAC for Multiple Buffer Transfers.
 */
static uint8_t _configureTransferMode(void)
{
	uint32_t xdmaCndc, xdmaInt;
	uint8_t i;

	if (dmaProgrammingMode < XDMA_LLI) {
		xdmadCfg.mbr_ubc = MICROBLOCK_LEN;
		xdmadCfg.mbr_sa = (uint32_t)sourceBuffer;
		xdmadCfg.mbr_da = (uint32_t)destinationBuffer;
		xdmadCfg.mbr_cfg = XDMAC_CC_TYPE_MEM_TRAN |
			XDMA_GET_CC_MEMSET(dmaMemSet) |
			XDMAC_CC_MEMSET_NORMAL_MODE |
			XDMAC_CC_CSIZE_CHK_1 |
			XDMA_GET_DATASIZE(dmaDataWidth) |
			XDMAC_CC_SIF_AHB_IF0 |
			XDMAC_CC_DIF_AHB_IF0 |
			XDMA_GET_CC_SAM(dmaSourceAddrMode) |
			XDMA_GET_CC_DAM(dmaDestAddrMode);

		xdmadCfg.mbr_bc = (dmaProgrammingMode == XDMA_SINGLE) ? 0 : 1;
		xdmadCfg.mbr_ds =  0;
		xdmadCfg.mbr_sus = 0;
		xdmadCfg.mbr_dus = 0;

		/* Put all interrupts on for non LLI list set-up of DMA */
		xdmaInt = (XDMAC_CIE_BIE   |
				   XDMAC_CIE_DIE   |
				   XDMAC_CIE_FIE   |
				   XDMAC_CIE_RBIE  |
				   XDMAC_CIE_WBIE  |
				   XDMAC_CIE_ROIE);
		XDMAD_ConfigureTransfer(&xdmad, dmaChannel, &xdmadCfg, 0, 0, xdmaInt);
		printf("- Set Micro-block length to [ %u ] \n\r",
				(unsigned int)xdmadCfg.mbr_ubc);
		printf("- Set Block length [ %u ] \n\r", (unsigned int)xdmadCfg.mbr_bc );
		printf("- Set Data Stride/Pattern [ %u ] \n\r",
				(unsigned int)xdmadCfg.mbr_ds );
		printf("- Set Source Micro-block Stride  [ %u ] \n\r",
				(unsigned int) xdmadCfg.mbr_sus );
		printf("- Set Destination  Micro-block Stride [ %u ]\n\r",
				(unsigned int)xdmadCfg.mbr_dus );
		printf("- Press 't' to perform xDMA transfer...\n\r");

	}
	if (dmaProgrammingMode == XDMA_LLI) {
		xdmadCfg.mbr_cfg = XDMAC_CC_TYPE_MEM_TRAN |
			XDMAC_CC_MBSIZE_SINGLE |
			XDMA_GET_CC_MEMSET(dmaMemSet) |
			XDMAC_CC_CSIZE_CHK_1 |
			XDMA_GET_DATASIZE(dmaDataWidth) |
			XDMAC_CC_SIF_AHB_IF0 |
			XDMAC_CC_DIF_AHB_IF0 |
			XDMA_GET_CC_SAM(dmaSourceAddrMode) |
			XDMA_GET_CC_DAM(dmaDestAddrMode);
		xdmadCfg.mbr_bc = 0;
		for (i = 0; i < MAX_LLI_SIZE; i++) {
			LLIview1[i].mbr_ubc = XDMA_UBC_NVIEW_NDV1 |
				((i == 0) ? XDMA_UBC_NSEN_UPDATED : 0 ) |
				((i == 0) ? XDMA_UBC_NDEN_UPDATED : 0 ) |
				((i == MAX_LLI_SIZE- 1) ? 0 : XDMA_UBC_NDE_FETCH_EN) |
				MICROBLOCK_LEN;
			LLIview1[i].mbr_sa =
				((i == 0) ? (uint32_t)sourceBuffer : \
				((uint32_t)sourceBuffer + (BUFFER_LEN >> 1)));
			LLIview1[i].mbr_da = ((i == 0) ? (uint32_t)destinationBuffer : \
				((uint32_t)destinationBuffer + (BUFFER_LEN >> 1)) );
			LLIview1[i].mbr_nda =
				(i == ( MAX_LLI_SIZE - 1)) ? 0 : (uint32_t)&LLIview1[i + 1];
		}
		xdmaCndc = XDMAC_CNDC_NDVIEW_NDV1 |
			XDMAC_CNDC_NDE_DSCR_FETCH_EN |
			XDMAC_CNDC_NDSUP_SRC_PARAMS_UPDATED |
			XDMAC_CNDC_NDDUP_DST_PARAMS_UPDATED;
		SCB_CleanDCache_by_Addr((uint32_t *)LLIview1, sizeof(LLIview1));
		xdmaInt = XDMAC_CIE_LIE;
		XDMAD_ConfigureTransfer(&xdmad, dmaChannel, &xdmadCfg, xdmaCndc,
				(uint32_t)&LLIview1[0], xdmaInt);
		printf("- Press 't' to perform xDMA Master transfer...\n\r");
	}
	return 0;
}

/**
 * \brief Start DMAC Multiple Buffer Transfer.
 */
static uint8_t _startDmaTransfer( void )
{
	uint32_t i;
	/* Prepare source data to be transferred. */
	for (i = 0; i < BUFFER_LEN; i++) {
		sourceBuffer[i] = i;
		destinationBuffer[i] = 0xFF;
	}

	printf("-I- The Source Buffer 0 content before transfer\n\r");
	_DumpBufferInfo((uint8_t *)sourceBuffer);
	/* Start transfer */
	SCB_CleanDCache_by_Addr((uint32_t *)sourceBuffer, BUFFER_LEN);
	XDMAD_StartTransfer(&xdmad, dmaChannel);
	while (XDMAD_IsTransferDone(&xdmad, dmaChannel));
	SCB_InvalidateDCache_by_Addr((uint32_t *)destinationBuffer, BUFFER_LEN);
	printf("-I- The Destination Buffer content after transfer\n\r");
	_DumpBufferInfo((uint8_t *)destinationBuffer);
	return 0;
}


/**
 * \brief XDMA handler.
 */
void XDMAC_Handler(void)
{
	XDMAD_Handler(&xdmad);
}

/*----------------------------------------------------------------------------
 *         Global functions
 *----------------------------------------------------------------------------*/
/**
 *  \brief XDMA Application entry point
 *
 *  \return Unused (ANSI-C compatibility)
 */
extern int main( void )
{
	uint8_t key;

	/* Disable watchdog */
	WDT_Disable(WDT);

	/* Enable I and D cache */
	SCB_EnableICache();
	SCB_EnableDCache();

	/* Output example information */
	printf("-- XDMA Example %s --\n\r", SOFTPACK_VERSION);
	printf("-- %s\n\r", BOARD_NAME);
	printf("-- Compiled: %s %s With %s--\n\r", __DATE__, __TIME__, COMPILER_NAME);

	/* Initialize XDMA driver instance with polling mode */
	XDMAD_Initialize(&xdmad, POLLING_MODE);

	/* Allocate a XDMA channel. */
	dmaChannel = XDMAD_AllocateChannel(&xdmad, XDMAD_TRANSFER_MEMORY,
			XDMAD_TRANSFER_MEMORY);
	if (dmaChannel == XDMAD_ALLOC_FAILED) {
		printf("-E- Can't allocate XDMA channel\n\r");
		return 0;
	}
	XDMAD_PrepareChannel(&xdmad, dmaChannel);

	/*Enable xDMA interrupt */
	NVIC_ClearPendingIRQ(XDMAC_IRQn);
	NVIC_SetPriority(XDMAC_IRQn ,1);
	NVIC_EnableIRQ(XDMAC_IRQn);

	/* Display menu */
	_displayMenu();

	for(;;) {
		key = DBG_GetChar();
		if ((key == 'a') || (key == 'b') || (key == 'c')) {
			dmaDataWidth = key - 'a';
			_displayMenu();
		}
		if ((key >= '0') && (key <= '3')) {
			dmaSourceAddrMode = key - '0';
			_displayMenu();
		}
		if ((key >= '4') && (key <= '7')) {
			dmaDestAddrMode = key - '4';
			_displayMenu();
		}
		if ((key >= '8') && (key <= '9')) {
			dmaMemSet = key - '8';
			_displayMenu();
		}
		if ((key == 'S') || (key == 's')){
			dmaProgrammingMode = 1;
			_configureTransferMode();
			ConfigFlag = 1;
		}
		if ((key == 'M') || (key == 'm')){
			dmaProgrammingMode = 2;
			_configureTransferMode();
			ConfigFlag = 1;
		}
		if ((key == 'L') || (key == 'l')){
			dmaProgrammingMode = 3;
			_configureTransferMode();
			ConfigFlag = 1;
		}
		if ((key == 'H') || (key == 'h'))
			_displayMenu();
		if ((key == 'T') || (key == 't')) {
			if (ConfigFlag) {
				printf("-I- Start XDMA transfer\n\r");
				_startDmaTransfer();
				ConfigFlag = 0;
			}
		}
	}
}
