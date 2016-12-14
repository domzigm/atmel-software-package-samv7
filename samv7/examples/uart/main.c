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
 *  \page uart UART example with DMA
 *
 *  \section Purpose
 *
 *  The UART example show how to use UART peripheral on
 *  SAMV7 family of microcontrollers. This basic application shows how to send
 *  and receive a buffer..
 *
 *  \section Requirements
 *
 *  This package can be used with SAM V71 Xplained Ultra board or SAME70 Xplained board.
 *
 *  \section Description
 *
 *  The demonstration program makes transfer buffer over DMA or via interrupt
 *  in local loop-back mode of UART
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
 *      -- uart Example xxx --
 *      -- xxxxxx-xx
 *      -- Compiled: xxx xx xxxx xx:xx:xx --
 *    Menu :
 *      ========================
 *       UART is Configured in : LoopBack
 *       I: Perform UART transfer with interrupt
 *       D: Perform UART DMA transfer in local loopback
 *       H: Display menu
 *     \endcode
 *  -# The user can then choose any of the available options to perform the
 *  described action.
 *
 *  \section References
 *  - uart/main.c
 *  - board.h
 */

/** \file
 *
 *  This file contains all the specific code for the UART example.
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
#define BASE_UART       UART4
#define BASE_ID         ID_UART4
#define BASE_IRQ        UART4_IRQn

#define RX_SIZE         30

/*----------------------------------------------------------------------------
 *        Local variables
 *----------------------------------------------------------------------------*/
const Pin BASE_UART_PINS[] = {PINS_UART4};
/** Global DMA driver for all transfer */
static UartDma Uartd;
static UartChannel UartTx, UartRx;
static sXdmad dmad;

COMPILER_ALIGNED(32) uint8_t pTxBuffer[] = {"This is UART Tx Buffer.........\n\r"};
COMPILER_ALIGNED(32) uint8_t pRxBuffer[RX_SIZE];

/*----------------------------------------------------------------------------
 *        Local functions
 *----------------------------------------------------------------------------*/

/**
 *  \brief Handler for UART4.
 *
 *  Process UART4 interrupts
 */
void UART4_Handler(void)
{
	uint32_t Status = UART_GetStatus(BASE_UART);

	if (Status & (UART_SR_OVRE | UART_SR_FRAME | UART_SR_PARE)) {
		BASE_UART->UART_CR = UART_CR_RSTSTA;
		printf("Error \n\r");
	}

	printf("%c", (char)BASE_UART->UART_RHR);
}

/**
 *  \brief Handler for XDMAC.
 *
 *  Process XDMA interrupts
 */
void XDMAC_Handler(void)
{
	XDMAD_Handler(&dmad);
}

/**
 * \brief Displays the user menu on the DBGU.
 */
static void DisplayMenu(void)
{
	printf("\n\rMenu :\n\r");
	printf("========================\n\r");
	printf(" UART is Configured in : LoopBack\n\r");
	printf("  I: Perform UART transfer with interrupt\n\r");
	printf("  D: Perform UART DMA transfer in local loopback\n\r");
	printf("  H: Display menu \n\r\n\r");
	printf("========================\n\r");
}

/**
 * \brief UART transfer with interrupt in UART loop back mode
 */
static void UartTransfer(void)
{
	uint8_t *pBuffer = &pTxBuffer[0];

	PMC_EnablePeripheral(BASE_ID);
	UART_Configure(BASE_UART, (UART_MR_PAR_NO | UART_MR_CHMODE_LOCAL_LOOPBACK),
			115200, BOARD_MCK);

	NVIC_ClearPendingIRQ(BASE_IRQ);
	NVIC_SetPriority(BASE_IRQ ,1);

	/* Enables the UART to transfer and receive data. */
	UART_SetTransmitterEnabled (BASE_UART , 1);
	UART_SetReceiverEnabled (BASE_UART , 1);

	UART_EnableIt(BASE_UART, (UART_IER_RXRDY | UART_IER_OVRE | UART_IER_FRAME
			| UART_IER_PARE));
	/* Enable interrupt  */
	NVIC_EnableIRQ(BASE_IRQ);

	while (*pBuffer != '\0') {
		UART_PutChar(BASE_UART, *pBuffer);
		pBuffer++;
	}
	UART_PutChar(BASE_UART, *pBuffer);
}

/**
 * \brief UART transfer with DMA in UART loop back mode
 */
static void _UartdConfigLB(void)
{
	uint32_t mode = 0
		| UART_MR_PAR_NO
		| UART_MR_BRSRCCK_PERIPH_CLK
		| UART_MR_CHMODE_LOCAL_LOOPBACK;

	dmad.pXdmacs = XDMAC;

	memset(&UartTx, 0, sizeof(UartChannel));
	memset(&UartRx, 0, sizeof(UartChannel));
	UartTx.BuffSize = 25;
	UartTx.pBuff = pTxBuffer;
	UartRx.BuffSize= 25;
	UartRx.pBuff = pRxBuffer;
	UartTx.sempaphore = 1;
	UartRx.sempaphore = 1;

	Uartd.pRxChannel = &UartRx;
	Uartd.pTxChannel = &UartTx;
	Uartd.pXdmad = &dmad;
	PMC_EnablePeripheral(BASE_ID);
	UARTD_Configure(&Uartd, BASE_ID, mode, 115200, BOARD_MCK);
}

/*----------------------------------------------------------------------------
 *        Exported functions
 *----------------------------------------------------------------------------*/
/**
 *  \brief Application entry point for UART example.
 *
 *  \return Unused (ANSI-C compatibility).
 */
extern int main(void)
{
	uint8_t ucKey;

	/* Disable watchdog */
	WDT_Disable(WDT);

	/* Output example information */
	printf("-- UART Example %s --\n\r", SOFTPACK_VERSION);
	printf("-- %s\n\r", BOARD_NAME);
	printf("-- Compiled: %s %s With %s--\n\r", __DATE__, __TIME__, COMPILER_NAME);

	/* Enable I and D cache */
	SCB_EnableICache();
	SCB_EnableDCache();

	/* Display menu */
	DisplayMenu();

	while (1) {
		ucKey = DBG_GetChar();

		switch (ucKey) {
		case 'h':
			DisplayMenu();
			break;
		case 'i':
		case 'I':
			printf("\n\rSending Tx Buffer.. \n\r");
			UartTransfer();
			break;

		case 'd':
		case 'D':
			memset(pRxBuffer,'X' ,30);
			pRxBuffer[28] = '\n';
			pRxBuffer[29] = '\r';
			printf("\n\rRx Buffer before transfer is \n\r");
			puts((char*)pRxBuffer);
			_UartdConfigLB();
			UARTD_EnableRxChannels(&Uartd, &UartRx);
			UARTD_EnableTxChannels(&Uartd, &UartTx);
			UARTD_RcvData(&Uartd);
			UARTD_SendData(&Uartd);

			printf("\n\rRx Buffer after transfer is \n\r");

			while (Uartd.pRxChannel->sempaphore == 0);
			puts((char*)pRxBuffer);
			UARTD_DisableRxChannels(&Uartd, &UartRx);
			UARTD_DisableTxChannels(&Uartd, &UartTx);
			break;

		default :
			break;
		}
	}
}
/** \endcond */
