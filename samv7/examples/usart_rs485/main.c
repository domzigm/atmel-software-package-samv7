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
 *  \page usart_rs485 USART RS485 example with DMA
 *
 *  This example demonstrates the RS485 mode provided by the USART peripherals on
 *  SAMV7 Microcontrollers.
 *
 *  \section Requirements
 *
 *  This package can be used with SAMV71 Xplained Ultra board or SAME70 Xplained board.
 *  Before running, make sure to connect two boards with RS485 lines. The rs485
 *  adapt board (ADM3485ARZ) for this purpose.
 *  Match each paired pins of two boards respectively with A to A,
 *  B to B and FGND to FGND(the central pin of J3).
 *  this part is connect with EK and ADM3485ARZ
 *   - <b>Board 1                             Board 2</b>
 *   -  TXD0(EXT1 pin14) <->DI            TXD0(EXT1 pin14) <->DI
 *   -  RXD0(EXT1 pin13) <->RO            RXD0(EXT1 pin13) <->RO
 *   -  RTS0(EXT1 pin5 ) <->DE            RTS0(EXT1 pin5 ) <->DE
 *   -  CTS0(EXT1 pin6 ) <->RE            CTS0(EXT1 pin6 ) <->RE
 *   -  3.3v                                   3.3v
 *   -  GND                                    GND
 *  this part is connect with 2 ADM3485ARZ
 *      A              <-------------------->  A
 *      B              <-------------------->  B
 *      PGND           <-------------------->  PGND
 *
 *  \section Description
 *
 *  This example connects two boards through RS485 interface. One board acts
 *  as the transmitter and the other one as the receiver. It is determined by
 *  the sequence the two applications started.
 *
 * \section Usage
 *
 *  -# Build the program and download it inside the board.
 *  Please refer to the Getting Started with SAM V71/E70 Microcontrollers.pdf
 *  -# Connect a serial cable to the DBGU port on the evaluation kit.
 *  -# On the computer, open and configure a terminal application for each board (e.g.
 *     HyperTerminal on Microsoft Windows) with these settings:
 *        - 115200 baud rate
 *        - 8 bits of data
 *        - No parity
 *        - 1 stop bit
 *        - No flow control
 *  -# Start application from two boards in sequence. Make sure the second board
 *     should NOT be started unless the first board had run to wait for the
 *     synchronizing character. The output message in later section would
 *     describe this.
 *
 *  -# In the terminal  window, the
 *     following text should appear (values depend on the board and chip used):
 *     \code
 *     -- USART RS485 Mode Example xxx --
 *     -- SAMxxxxx-xx
 *     -- Compiled: xxx xx xxxx xx:xx:xx --
 *     \endcode
 *
 *  \section References
 *  - usart_rs485/main.c
 *  - pio.h
 *  - usart.h
 */

/** \file
 *
 *  This file contains all the specific code for the usart_rs485 example.
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

/** size of the receive buffer used by the DMA, in bytes.*/
#define BUFFER_SIZE     1000

/** baud rate */
#define BAUDRATE_RS485  256000

/** Register base for USART */
#define USART           USART0
#define ID_USART        ID_USART0

/*----------------------------------------------------------------------------
 *        Local variables
 *----------------------------------------------------------------------------*/
/** Global DMA driver for all transfer */
static sXdmad dmad;

static uint32_t usartDmaRxChannel;
static uint32_t usartDmaTxChannel;

/** RE for RS485 */
#define PIN_USART0_CTS_IOR {PIO_PB2C_CTS0, PIOB, ID_PIOB, PIO_OUTPUT_0, PIO_DEFAULT}

/**  Pins to configure for the application.*/
const Pin pins[] = {PIN_USART0_RXD, PIN_USART0_TXD, PIN_USART0_CTS_IOR, PIN_USART0_RTS};

/** Transmit buffer. */
char palette[BUFFER_SIZE]=
 "**************************************************************************\n\r\
 *  This application gives an example of how to use USART in RS485 mode.\n\r\
 *  The USART features the RS485 mode to enable line driver control.\n\r\
 *  While operating in RS485 mode, the USART behaves as though in asynchronous \n\r\
 *  or synchronous mode and configuration of all the parameters is possible \n\r\
 *  \n\r\
 *  The difference is that the RTS pin is driven high when the transmitter\n\r\
 *  is operating. The behavior of the RTS pin is controlled by the TXEMPTY bit.\n\r\
 *  \n\r\
 **************************************************************************\n\r\
 ";

/** Transmit buffer. */
COMPILER_ALIGNED(32) static char Buffer[BUFFER_SIZE];

/** buffer for receiving */
COMPILER_ALIGNED(32) static char pRecvBufferUSART[BUFFER_SIZE];

/** reception done*/
static volatile uint8_t recvDone = 0;

/** sending done*/
static volatile uint8_t transDone = 0;

/*----------------------------------------------------------------------------
 *        Local functions
 *----------------------------------------------------------------------------*/

/**
 * \brief Configures USART in rs485 mode
 */
static void _ConfigureUsart(void)
{
	uint32_t mode = US_MR_USART_MODE_RS485 | US_MR_USCLKS_MCK
					| US_MR_CHRL_8_BIT | US_MR_PAR_NO | US_MR_NBSTOP_1_BIT
					| US_MR_CHMODE_NORMAL;

	/* Enable the peripheral clock in the PMC */
	PMC_EnablePeripheral(ID_USART);

	/* Configure the USART in the desired mode @USART_SPI_CLK bauds*/
	USART_Configure(USART, mode, BAUDRATE_RS485, BOARD_MCK);

	/* Enable receiver & transmitter */
	USART_SetTransmitterEnabled(USART, 1);
	USART_SetReceiverEnabled(USART, 1);
}

/**
 * ISR for DMA interrupt
 */
void XDMAC_Handler(void)
{
	XDMAD_Handler(&dmad);
}

/**
 *  \brief Callback function for DMA receiving.
 */
static void _DmaRxCallback(uint32_t channel, void* pArg)
{
	channel = channel;
	pArg = pArg;
	SCB_InvalidateDCache_by_Addr((uint32_t*)pRecvBufferUSART, BUFFER_SIZE - 1);
	recvDone = 1;
}

/**
 *  \brief Callback function for DMA transmitting.
 */
static void _DmaTxCallback(uint32_t channel, void* pArg)
{
	channel = channel;
	pArg = pArg;
	transDone = 1;
}

/**
 *  \brief Start USART sending data.
 */
static void _DmaUsartTx(void)
{
	sXdmadCfg xdmadCfg;

	xdmadCfg.mbr_ubc = BUFFER_SIZE;
	xdmadCfg.mbr_sa = (uint32_t)Buffer;
	xdmadCfg.mbr_da = (uint32_t)&USART->US_THR;
	xdmadCfg.mbr_cfg = XDMAC_CC_TYPE_PER_TRAN |
					XDMAC_CC_MBSIZE_SINGLE |
					XDMAC_CC_DSYNC_MEM2PER |
					XDMAC_CC_CSIZE_CHK_1 |
					XDMAC_CC_DWIDTH_BYTE |
					XDMAC_CC_SIF_AHB_IF1 |
					XDMAC_CC_DIF_AHB_IF1 |
					XDMAC_CC_SAM_INCREMENTED_AM |
					XDMAC_CC_DAM_FIXED_AM |
					XDMAC_CC_PERID(XDMAIF_Get_ChannelNumber(ID_USART, XDMAD_TRANSFER_TX));

	xdmadCfg.mbr_bc = 0;
	xdmadCfg.mbr_ds = 0;
	xdmadCfg.mbr_sus = 0;
	xdmadCfg.mbr_dus = 0;
	XDMAD_ConfigureTransfer(&dmad, usartDmaTxChannel, &xdmadCfg, 0, 0,
							XDMAC_CIE_BIE |
							XDMAC_CIE_DIE |
							XDMAC_CIE_FIE |
							XDMAC_CIE_RBIE |
							XDMAC_CIE_WBIE |
							XDMAC_CIE_ROIE);
	SCB_CleanDCache_by_Addr((uint32_t*)Buffer, BUFFER_SIZE);
	XDMAD_StartTransfer(&dmad, usartDmaTxChannel);
}

/**
 *  \brief Start USART waiting data.
 */
static void _DmaUsartRx(void)
{
	uint32_t status;

	/* Read USART status */
	status = USART->US_CSR;
	if ((status & US_CSR_RXRDY) == US_CSR_RXRDY)
		status = USART->US_RHR; /* clear the US_CSR_RXRDY*/

	sXdmadCfg xdmadCfg;
	xdmadCfg.mbr_ubc = BUFFER_SIZE;
	xdmadCfg.mbr_sa = (uint32_t)&USART->US_RHR;
	xdmadCfg.mbr_da = (uint32_t)pRecvBufferUSART;
	xdmadCfg.mbr_cfg = XDMAC_CC_TYPE_PER_TRAN |
					   XDMAC_CC_MBSIZE_SINGLE |
					   XDMAC_CC_DSYNC_PER2MEM |
					   XDMAC_CC_CSIZE_CHK_1 |
					   XDMAC_CC_DWIDTH_BYTE |
					   XDMAC_CC_SIF_AHB_IF1 |
					   XDMAC_CC_DIF_AHB_IF1 |
					   XDMAC_CC_SAM_FIXED_AM |
					   XDMAC_CC_DAM_INCREMENTED_AM |
					   XDMAC_CC_PERID(XDMAIF_Get_ChannelNumber( ID_USART, XDMAD_TRANSFER_RX));

	xdmadCfg.mbr_bc = 0;
	xdmadCfg.mbr_ds = 0;
	xdmadCfg.mbr_sus = 0;
	xdmadCfg.mbr_dus = 0;
	XDMAD_ConfigureTransfer(&dmad, usartDmaRxChannel, &xdmadCfg, 0, 0,
							XDMAC_CIE_BIE |
							XDMAC_CIE_DIE |
							XDMAC_CIE_FIE |
							XDMAC_CIE_RBIE |
							XDMAC_CIE_WBIE |
							XDMAC_CIE_ROIE);
	XDMAD_StartTransfer( &dmad, usartDmaRxChannel);
}


/*
 *  \brief DMA driver configuration
 */
static void _ConfigureDma(void)
{
	/* Driver initialize */
	XDMAD_Initialize(&dmad, 0);

	/* Allocate XDMA channels for USART */
	usartDmaTxChannel = XDMAD_AllocateChannel(&dmad, XDMAD_TRANSFER_MEMORY, ID_USART);
	usartDmaRxChannel = XDMAD_AllocateChannel(&dmad, ID_USART, XDMAD_TRANSFER_MEMORY);
	if (usartDmaTxChannel == XDMAD_ALLOC_FAILED ||
		usartDmaRxChannel == XDMAD_ALLOC_FAILED ) {
		printf("XDMA channel allocat failed!\n\r");
		while (1);
	}

	/* Set RX callback */
	XDMAD_SetCallback(&dmad, usartDmaRxChannel,(XdmadTransferCallback)_DmaRxCallback, 0);
	/* Set TX callback */
	XDMAD_SetCallback(&dmad, usartDmaTxChannel,(XdmadTransferCallback)_DmaTxCallback, 0);
	XDMAD_PrepareChannel(&dmad, usartDmaRxChannel );
	XDMAD_PrepareChannel(&dmad, usartDmaTxChannel);
}

/**
 * \brief Display menu.
 */
static void _DisplayMenu(void)
{
	printf("Menu :\n\r");
	printf("------\n\r");
	printf(" - t: Transmit pattern to RS485\n\r");
	printf(" - r: Receive data from RS485 \n\r");
	printf(" - m: Display menu \n\r");
}

/**
 * \brief Dump buffer to DBGU
 *
 */
static void _DumpInfo(char *buf, uint32_t size)
{
	uint32_t i = 0;

	while ((i < size) && (buf[i] != 0))
		printf("%c", buf[i++]);
}

/*----------------------------------------------------------------------------
 *        Exported functions
 *----------------------------------------------------------------------------*/
/**
 *  \brief Application entry point.
 *
 *  \return Unused (ANSI-C compatibility).
 */
extern int main(void)
{
	uint8_t ucKey;

	/* Disable watchdog */
	WDT_Disable(WDT);

	SCB_EnableICache();
	SCB_EnableDCache();

	/* Output example information */
	printf("-- USART RS485 Example %s --\n\r", SOFTPACK_VERSION);
	printf("-- %s\n\r", BOARD_NAME);
	printf("-- Compiled: %s %s  With %s--\n\r", __DATE__, __TIME__, COMPILER_NAME);

	/* Configure pins*/
	PIO_Configure(pins, PIO_LISTSIZE(pins));

	/* Display menu */
	_DisplayMenu();

	/* Configure DMA with IRQ */
	_ConfigureDma();

	memcpy(Buffer, palette, BUFFER_SIZE);

	/* configure USART in RS485 mode*/
	_ConfigureUsart();

	NVIC_EnableIRQ(XDMAC_IRQn);

	while (1) {
		ucKey = DBG_GetChar();
		switch (ucKey) {
		case 't':
		case 'T':
			printf("-I- RS485 Transmitting ... \n\r");
			_DmaUsartTx();
			while (!transDone);
			printf("-I- RS485 Transmitting completed \n\r");
			transDone = 0;
			break;

		case 'r':
		case 'R':
			printf("-I- RS485 receiving ... \n\r");
			recvDone = 0;
			_DmaUsartRx();
			while (!recvDone);
			/* successfully received */
			_DumpInfo(pRecvBufferUSART, BUFFER_SIZE);
			printf("-I- RS485 Receiving completed \n\r");
			memset(pRecvBufferUSART, 0, sizeof(pRecvBufferUSART));
			break;

		case 'm':
		case 'M':
			_DisplayMenu();
			break;
		}
	}
}

