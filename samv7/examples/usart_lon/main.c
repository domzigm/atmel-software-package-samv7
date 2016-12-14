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
 *  \page usart_lon USART LON example with DMA
 *
 *  This example demonstrates the LON mode provided by the USART peripherals on
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
 *   -  TXD1(J507 pin28, D47) <->DI       TXD1(J507 pin28, D47) <->DI
 *   -  RXD1(J507 pin27, D46) <->RO       RXD1(J507 pin27, D46) <->RO
 *   -  RTS1(J507 pin26, D45) <->DE       RTS1(J507 pin26, D45) <->DE
 *   -  CTS1(J507 pin25, D44) <->RE       CTS1(J507 pin25, D44) <->RE
 *   -  3.3v                                   3.3v
 *   -  GND                                    GND
 *  this part is connect with 2 ADM3485ARZ
 *      A              <-------------------->  A
 *      B              <-------------------->  B
 *      PGND           <-------------------->  PGND
 *
 *  \section Description
 *
 *  This example connects two boards through RS485 interface to show how to
 *  use USART in LON (local operating network) mode. One of the board acts
 *  as the transmitter and the other one as the receiver. It is determined by
 *  the sequence the two applications started.
 *
 * \section Usage
 *
 *  -# Build the program and download it inside the board.
 *  Please refer to the Getting Started with SAM V71/E70 Microcontrollers.pdf
 *  -# Connect a serial cable to the UART4 port (J505 Pin3 D18 and Pin4 D19) on the evaluation kit.
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
 *     -- USART LON Mode Example xxx --
 *     -- SAMxxxxx-xx
 *     -- Compiled: xxx xx xxxx xx:xx:xx --
 *     \endcode
 *
 *  \section References
 *  - usart_lon/main.c
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
#define BUFFER_SIZE     256

/** baud rate */
#define BAUDRATE  256000

/** Register base for USART */
#define USART           USART1
#define ID_USART        ID_USART1

/*----------------------------------------------------------------------------
 *        Local variables
 *----------------------------------------------------------------------------*/
/** Global DMA driver for all transfer */
static sXdmad dmad;

static uint32_t usartDmaRxChannel;
static uint32_t usartDmaTxChannel;

/** RE for LON */
#define PIN_USART1_CTS_IOR {PIO_PA25A_CTS1, PIOA, ID_PIOA, PIO_OUTPUT_0, PIO_DEFAULT}

/**  Pins to configure for the application.*/
const Pin pins[] = {PIN_USART1_RXD, PIN_USART1_TXD, PIN_USART1_CTS_IOR, PIN_USART1_RTS };

/** Transmit buffer. resver 2 bytes for DATAL and L2HDR*/
char palette[BUFFER_SIZE - 2]=
"#*************************************************************************\n\r\
 *  This application gives an example of how to use USART in LON mode.\n\r\
 *  The LON mode provides connectivity to the local operating network (LON).\n\r\
 *                      .\n\r\
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
 * \brief Configures USART in LON mode
 */
static void _ConfigureUsart( void )
{
	/* Initialization for Sending/Receiving A LON Frame */

	uint32_t mode;
	Usart *pUsart = USART;

	/* 1. Write TXEN and RXEN in US_CR to enable both the transmitter and the
	      receiver.                                                           */
	/* Enable receiver & transmitter */
	USART_SetTransmitterEnabled(pUsart, 1);
	USART_SetReceiverEnabled(pUsart, 1);

	/* 2. Write USART_MODE in US_MR to select the LON mode configuration.     */
	/* Enable the peripheral clock in the PMC */
	PMC_EnablePeripheral(ID_USART);

	mode = US_MR_USART_MODE_LON | US_MR_USCLKS_MCK
					| US_MR_CHRL_8_BIT | US_MR_PAR_NO | US_MR_NBSTOP_1_BIT
					| US_MR_CHMODE_NORMAL
					//| US_MR_CHMODE_LOCAL_LOOPBACK //US_MR_CHMODE_NORMAL
					| US_MR_MAN;

	pUsart->US_CR = US_CR_RSTRX | US_CR_RSTTX
					| US_CR_RXDIS | US_CR_TXDIS | US_CR_RSTSTA;
	pUsart->US_IDR = 0xFFFFFFFF;

	pUsart->US_MR = mode;

	/* 3. Write CD and FP in US_BRGR to configure the baud rate.              */
	USART_SetBaudrate(pUsart, 0, BAUDRATE, BOARD_MCK);

	/* 4. Write COMMT, COLDET, TCOL, CDTAIL, RDMNBM and DMAM in US_LONMR to
	      configure the LON operating mode.                                   */
	pUsart->US_LONMR = US_LONMR_COMMT /* LON comm_type = 2 mode */
					| US_LONMR_COLDET /* LON collision detection feature enabled */
					| US_LONMR_DMAM   /* The LON data length register US_LONDL is written by the DMA */
					| US_LONMR_LCDS   /* LON collision detection source is internal. */
					| US_LONMR_EOFS(1);

	/* 5. Write BETA2, BETA1TX, BETA1RX, PCYCLE, PSNB, NPS, IDTTX and ITDRX
	      respectively in US_FIDI, US_LONB1TX, US_LONB1RX, US_TTGR, US_LONPRIO,
	      US_LONIDTTX and US_LONIDTRX to set the LON network configuration.   */
	pUsart->US_FIDI    = 5; /* LON BETA2 length */
	pUsart->US_LONB1TX = 5; /* LON Beta1 Tx     */
	pUsart->US_LONB1RX = 5; /* LON Beta1 Rx     */
	pUsart->US_TTGR    = US_TTGR_PCYCLE(5);  /* package cycle timer value */
	pUsart->US_LONPRIO = US_LONPRIO_PSNB(5)  /* Number of priority slots  */
						| US_LONPRIO_NPS(5); /* Node priority slot        */

	/* Set LON Indeterminate Time after Transmission or Reception
	 * (comm_type = 1 mode only) */
	if (0 == (pUsart->US_LONMR & US_LONMR_COMMT)) {
		pUsart->US_IDTTX   = US_IDTTX_IDTTX(3);
		pUsart->US_IDTRX   = US_IDTRX_IDTRX(3);
	}

	/* (TX) 6. Write TX_PL in US_MAN to select the preamble pattern to use.   */
	/* (RX) 6. Write RXIDLEV and RX_PL in US_MAN to indicate the receiver line
	           value and select the preamble pattern to use.                  */
	// preamble pattern
	pUsart->US_MAN = US_MAN_TX_PL(5) | US_MAN_TX_PP_ALL_ONE
					| US_MAN_RX_PL(5) | US_MAN_RX_PP_ALL_ONE
					| US_MAN_ONE | US_MAN_DRIFT;

	pUsart->US_LONPR = US_LONPR_LONPL(5);

	/* proceed by a software reset of the transmitter and the receiver and
	followed by a transmitter/receiver enable to avoid unpredictable behavior */
	pUsart->US_CR = US_CR_RSTRX | US_CR_RSTTX;
	pUsart->US_CR = US_CR_RXEN | US_CR_TXEN;
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
static void _DmaRxCallback(uint32_t status, void* pArg)
{
	status = status;
	pArg = pArg;
	SCB_InvalidateDCache_by_Addr((uint32_t*)pRecvBufferUSART, BUFFER_SIZE - 1);
	recvDone = 1;
}

/**
 *  \brief Callback function for DMA transmitting.
 */
static void _DmaTxCallback(uint32_t status, void* pArg)
{
	status = status;
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
	xdmadCfg.mbr_cfg =
			XDMAC_CC_TYPE_PER_TRAN |
			XDMAC_CC_MBSIZE_SINGLE |
			XDMAC_CC_DSYNC_MEM2PER |
			XDMAC_CC_CSIZE_CHK_1 |
			XDMAC_CC_DWIDTH_BYTE |
			XDMAC_CC_SIF_AHB_IF1 |
			XDMAC_CC_DIF_AHB_IF1 |
			XDMAC_CC_SAM_INCREMENTED_AM |
			XDMAC_CC_DAM_FIXED_AM |
			XDMAC_CC_PERID(
				XDMAIF_Get_ChannelNumber(ID_USART, XDMAD_TRANSFER_TX)
				);

	xdmadCfg.mbr_bc  = 0;
	xdmadCfg.mbr_ds  = 0;
	xdmadCfg.mbr_sus = 0;
	xdmadCfg.mbr_dus = 0;
	XDMAD_ConfigureTransfer(
			&dmad, usartDmaTxChannel, &xdmadCfg, 0, 0,
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
	xdmadCfg.mbr_ubc = BUFFER_SIZE - 1;
	xdmadCfg.mbr_sa = (uint32_t)&USART->US_RHR;
	xdmadCfg.mbr_da = (uint32_t)pRecvBufferUSART;
	xdmadCfg.mbr_cfg =
			XDMAC_CC_TYPE_PER_TRAN |
			XDMAC_CC_MBSIZE_SINGLE |
			XDMAC_CC_DSYNC_PER2MEM |
			XDMAC_CC_CSIZE_CHK_1 |
			XDMAC_CC_DWIDTH_BYTE |
			XDMAC_CC_SIF_AHB_IF1 |
			XDMAC_CC_DIF_AHB_IF1 |
			XDMAC_CC_SAM_FIXED_AM |
			XDMAC_CC_DAM_INCREMENTED_AM |
			XDMAC_CC_PERID(
				XDMAIF_Get_ChannelNumber(ID_USART, XDMAD_TRANSFER_RX)
				);

	xdmadCfg.mbr_bc  = 0;
	xdmadCfg.mbr_ds  = 0;
	xdmadCfg.mbr_sus = 0;
	xdmadCfg.mbr_dus = 0;
	XDMAD_ConfigureTransfer(
			&dmad, usartDmaRxChannel, &xdmadCfg, 0, 0,
			XDMAC_CIE_BIE |
			XDMAC_CIE_DIE |
			XDMAC_CIE_FIE |
			XDMAC_CIE_RBIE |
			XDMAC_CIE_WBIE |
			XDMAC_CIE_ROIE);
	XDMAD_StartTransfer(&dmad, usartDmaRxChannel);
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
		usartDmaRxChannel == XDMAD_ALLOC_FAILED) {
		printf("XDMA channel allocat failed!\n\r");
		while (1);
	}

	/* Set RX callback */
	XDMAD_SetCallback(&dmad, usartDmaRxChannel,(XdmadTransferCallback)_DmaRxCallback, 0);
	/* Set TX callback */
	XDMAD_SetCallback(&dmad, usartDmaTxChannel,(XdmadTransferCallback)_DmaTxCallback, 0);
	XDMAD_PrepareChannel(&dmad, usartDmaRxChannel);
	XDMAD_PrepareChannel(&dmad, usartDmaTxChannel);

}

/**
 * \brief Display menu.
 */
static void _DisplayMenu(void)
{
	printf("Menu :\n\r");
	printf("------\n\r");
	printf(" - t: Transmit pattern to LON\n\r");
	printf(" - r: Receive data from LON \n\r");
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
	printf("-- USART LON Example %s --\n\r", SOFTPACK_VERSION);
	printf("-- %s\n\r", BOARD_NAME);
	printf("-- Compiled: %s %s  With %s--\n\r", __DATE__, __TIME__, COMPILER_NAME);

	/* Configure pins */
	PIO_Configure(pins, PIO_LISTSIZE(pins));
	/* PB4 function selected */
	MATRIX->MATRIX_WPMR = MATRIX_WPMR_WPKEY_PASSWD;
	MATRIX->CCFG_SYSIO |= CCFG_SYSIO_SYSIO4;

	/* Display menu */
	_DisplayMenu();

	/* Configure DMA with IRQ */
	_ConfigureDma();

	Buffer[0] = sizeof(palette) - 1; /* LON Data Length:  */
	Buffer[1] = US_LONL2HDR_BLI(2);
	memcpy(&Buffer[2], palette, sizeof(palette));

	/* configure USART in LON mode*/
	_ConfigureUsart();

	NVIC_EnableIRQ(XDMAC_IRQn);

	while (1) {
		ucKey = DBG_GetChar();
		switch (ucKey) {
		case 't':
		case 'T':
			printf("-I- LON Transmitting ... \n\r");
			USART->US_CR = US_CR_RSTSTA;    /* Reset Status Bits */
			_DmaUsartTx();
			while (!transDone);
			printf("-I- LON Transmitting completed \n\r");
			transDone = 0;
			break;

		case 'r':
		case 'R':
			printf("-I- LON receiving ... \n\r");
			USART->US_CR = US_CR_RSTSTA;    /* Reset Status Bits */
			recvDone = 0;

			_DmaUsartRx();
			while (!recvDone);
			/* successfully received */
			_DumpInfo(pRecvBufferUSART, BUFFER_SIZE - 1);
			printf("\n\r-I- LON Receiving completed \n\r");
			memset(pRecvBufferUSART, 0, sizeof(pRecvBufferUSART));
			break;

		case 'm':
		case 'M':
			_DisplayMenu();
			break;
		}
	}
}


