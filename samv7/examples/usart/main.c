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
 *  \page usart USART example with DMA
 *
 *  \section Purpose
 *
 *  The USART example shows the how to transfer/receive buffer over USART.
 *
 *  \section Requirements
 *
 *  This package can be used with SAMV71 Xplained Ultra board or SAME70 Xplained board.
 *
 *  \section Description
 *
 *  The demonstration program transfer a buffer via USART in local loopback
 *  mode and receives the buffer.
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
 *  -# Start the application.
 *  -# In the terminal window, the following text should appear (values depend
 *  on the board and chip used):
 *     \code
 *      -- USART Example Example xxx --
 *      -- xxxxxx-xx
 *      -- Compiled: xxx xx xxxx xx:xx:xx --
 *
 *      Menu :
 *      ========================
 *       UART is Configured in : LoopBack
 *       D: Perform USART DMA LoopBack Normal transfer
 *       T: Perform USART DMA Normal transfer(remote loopback)
 *       H: Display menu
 *     \endcode
 *  -# The user can then choose any of the available options to perform the
 *  described action.
 *  \section References
 *  - USART/main.c
 *  - pio.h
 *  - pio_it.h
 *  - usart_dma.h
 *  - trace.h
 */

/** \file
 *
 *  This file contains all the specific code for the USART example.
 *
 */

/*----------------------------------------------------------------------------
 *        Headers
 *----------------------------------------------------------------------------*/

#include "board.h"

#include <stdbool.h>
#include <stdio.h>
#include <string.h>

/*----------------------------------------------------------------------------
 *        Local definitions
 *----------------------------------------------------------------------------*/

/** Pins for USART */
#define PINS_USART  PIN_USART0_TXD, PIN_USART0_RXD

/** Register base for USART */

#define BASE_USART              USART0
#define ID_USART                ID_USART0
#define USART_IRQ               USART0_IRQn
#define USART_TIMEOUT           115200

/*----------------------------------------------------------------------------
 *        Local functions
 *----------------------------------------------------------------------------*/

/** Global DMA driver for all transfer */
static UsartDma Usartd;
static UsartChannel UsartTx, UsartRx;
static sXdmad ChDma;
static volatile uint32_t Timeout = 0;

COMPILER_ALIGNED(32) uint8_t pTxBuffer[] = {"This is first USART TX Buffer\n\r"};
COMPILER_ALIGNED(32) uint8_t pRxBuffer[30], BuffRx, BuffTx;

/**  Pins to configure for the application.*/
const Pin pins[] = {PINS_USART};

#define PIN0 {PIO_PA6, PIOA, ID_PIOA, PIO_OUTPUT_0, PIO_DEFAULT}
#define PIN1 {PIO_PD11, PIOD, ID_PIOD, PIO_OUTPUT_0, PIO_DEFAULT}

#define PINS  PIN0, PIN1
const Pin pins_test[] = {PINS};


void XDMAC_Handler(void)
{
	XDMAD_Handler(&ChDma);
}

void USART0_Handler(void)
{
	uint32_t status = BASE_USART->US_CSR;

	if (status & US_CSR_TIMEOUT) {
		USART_AcknowledgeRxTimeOut(BASE_USART, 0);
		USART_AcknowledgeRxTimeOut(BASE_USART, 1); // generate periodic interrupt
		printf("\r %u sec without char on USART Rx", (unsigned int)Timeout++);
	}
}

/**
 * \brief Displays the user menu on the DBGU.
 */
static void DisplayMenu(void)
{
	printf("\n\rMenu :\n\r");
	printf("========================\n\r");
	printf("  L: Perform USART DMA LoopBack Normal transfer\n\r");
	printf("  T: Perform USART DMA Normal transfer(remote loopback) \n\r");
	//(need to connect USART to D14 and D15 on board)
	printf("  W: Perform USART Timeout  \n\r");
	//(need to connect USART to D14 and D15 on board)
	printf("  H: Display menu \n\r\n\r");
	printf("========================\n\r");
}

static void _UsartdConfig(void)
{
	uint32_t mode = 0
				| US_MR_USART_MODE_NORMAL
				| US_MR_CHRL_8_BIT
				| US_MR_PAR_NO
				| US_MR_NBSTOP_1_BIT
				| US_MR_CHMODE_NORMAL;

	memset(&UsartTx, 0, sizeof(UsartChannel));
	memset(&UsartRx, 0, sizeof(UsartChannel));

	UsartTx.BuffSize = 1;
	UsartTx.pBuff = &BuffTx;
	UsartRx.BuffSize = 1;
	UsartRx.pBuff = &BuffRx;
	UsartTx.dmaProgress = 1;
	UsartRx.dmaProgress = 1;

	UsartTx.dmaProgrammingMode = XDMAD_SINGLE;
	UsartRx.dmaProgrammingMode = XDMAD_SINGLE;

	Usartd.pXdmad = &ChDma;
	Usartd.pRxChannel = &UsartRx;
	Usartd.pTxChannel = &UsartTx;
	USARTD_Configure(&Usartd, ID_USART, mode, 115200, BOARD_MCK);
}

static void _UsartdConfigLB(void)
{
	uint32_t mode = 0
				| US_MR_USART_MODE_NORMAL
				| US_MR_CHRL_8_BIT
				| US_MR_PAR_NO
				| US_MR_NBSTOP_1_BIT
				| US_MR_CHMODE_LOCAL_LOOPBACK;
	memset(&UsartTx, 0, sizeof(UsartChannel));
	memset(&UsartRx, 0, sizeof(UsartChannel));

	UsartTx.BuffSize = 30;
	UsartTx.pBuff = pTxBuffer;
	UsartRx.BuffSize= 30;
	UsartRx.pBuff = pRxBuffer;
	UsartTx.dmaProgress = 1;
	UsartRx.dmaProgress = 1;

	UsartTx.dmaProgrammingMode = XDMAD_SINGLE;
	UsartRx.dmaProgrammingMode = XDMAD_SINGLE;

	UsartTx.callback = 0;
	UsartRx.callback = 0;

	Usartd.pXdmad = &ChDma;
	Usartd.pRxChannel = &UsartRx;
	Usartd.pTxChannel = &UsartTx;
	USARTD_Configure(&Usartd, ID_USART, mode, 115200, BOARD_MCK);
}

/*----------------------------------------------------------------------------
 *        Exported functions
 *----------------------------------------------------------------------------*/
/**
 *  \brief getting-started Application entry point.
 *
 *  \return Unused (ANSI-C compatibility).
 */
extern int main( void )
{

	uint8_t ucKey;

	/* Disable watchdog */
	WDT_Disable(WDT);

	SCB_EnableICache();
	SCB_EnableDCache();

	/* Output example information */
	printf("-- USART Example %s --\n\r", SOFTPACK_VERSION);
	printf("-- %s\n\r", BOARD_NAME);
	printf("-- Compiled: %s %s With %s--\n\r", __DATE__, __TIME__, COMPILER_NAME);


	/* Configure pins*/
	PIO_Configure(pins, PIO_LISTSIZE(pins));

	/*Configure pins to watch cache coherence time*/
	PIO_Configure(pins_test, PIO_LISTSIZE(pins_test));

	/* Display menu */
	DisplayMenu();

	while (1) {
		ucKey = DBG_GetChar();

		switch (ucKey) {
		case 'H' :
		case 'h' :
			DisplayMenu();
			break;

		case 'l' :
		case 'L' :
			memset(pRxBuffer,'X' ,30);
			pRxBuffer[28] = '\n';
			pRxBuffer[29] = '\r';
			printf("\n\rRx Buffer  transfer is \n\r");

			puts((const char *)pTxBuffer);

			printf("\n\rRx Buffer before transfer is \n\r");
			puts((const char *)pRxBuffer);
			_UsartdConfigLB();
			USARTD_EnableRxChannels(&Usartd, &UsartRx);
			USARTD_EnableTxChannels(&Usartd, &UsartTx);
			USARTD_RcvData(&Usartd);
			USARTD_SendData(&Usartd);
			while (UsartRx.dmaProgress != 1);
			printf("Rx Buffer after transfer is \n\r");
			puts((const char *)pRxBuffer);
			USARTD_DisableRxChannels(&Usartd, &UsartRx);
			USARTD_DisableTxChannels(&Usartd, &UsartTx);
			break;

		case 't' :
		case 'T' :
			printf("Press Q on USART terminal to quit\n\r");
			_UsartdConfig();
			while ('Q' != BuffTx) {
				USARTD_EnableRxChannels(&Usartd, &UsartRx);
				USARTD_EnableTxChannels(&Usartd, &UsartTx);
				USARTD_RcvData(&Usartd);
				while (UsartRx.dmaProgress != 1);
				BuffTx = BuffRx;
				USARTD_SendData(&Usartd);
				while (UsartTx.dmaProgress != 1);
			}
			printf("Exit \n\r");
			DisplayMenu();
			break;

		case 'w' :
		case 'W' :
			printf("\n\r It will Quit if no character received within 10 secs \
				(Need to send at least one byte to initiate timeout)\n\r");
			_UsartdConfig();
			NVIC_ClearPendingIRQ(USART_IRQ);
			NVIC_SetPriority(USART_IRQ , 1);
			USART_EnableIt(BASE_USART, US_IER_TIMEOUT);

			NVIC_EnableIRQ(USART_IRQ);
			USART_EnableRecvTimeOut( BASE_USART, USART_TIMEOUT);
			USARTD_EnableRxChannels(&Usartd, &UsartRx);
			USARTD_EnableTxChannels(&Usartd, &UsartTx);
			USARTD_RcvData(&Usartd);
			while (Timeout < 10) {
				if (UsartRx.dmaProgress) {
					BuffTx = BuffRx;
					Timeout = 0;
					USARTD_SendData(&Usartd);
					while (UsartTx.dmaProgress != 1);
					USARTD_EnableRxChannels(&Usartd, &UsartRx);
					USARTD_EnableTxChannels(&Usartd, &UsartTx);
					USARTD_RcvData(&Usartd);
				}
			}
			USART_DisableIt(BASE_USART, US_IER_TIMEOUT);
			NVIC_DisableIRQ(USART_IRQ);
			printf("Exit \n\r");
			DisplayMenu();
			break;

		default :
			break;
		}
	}
}
/** \endcond */
