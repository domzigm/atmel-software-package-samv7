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
 * \page afe12_dma AFE12 DMA Example
 *
 * \section Purpose
 *
 * The AFE12 example demonstrates how to use AFE peripheral with several modes.
 *
 * \section Requirements
 *
 * This package can be used with SAMV71 Xplained Ultra board or SAME70 Xplained board.
 * To enable full scale measurement connect DAC to AFE.
 *
 * \section Connection
 * PB13 -- PB2
 *
 * \section Description
 *
 * This application shows how to use the AFE in 12-bit free run mode with DMA
 * enabled.
 *
 *  \section Usage
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
 * -# In the terminal window, the
 *    following text should appear (values depend on the board and chip used):
 *    \code
 *     -- AFE12 Example xxx --
 *     -- xxxxxx-xx
 *     -- Compiled: xxx xx xxxx xx:xx:xx --
 *    \endcode
 * -# The application will output converted value to hyperterminal and display
 *    them.
 *
 * \section References
 * - afe12_dma/main.c
 * - afe_dma.h
 * - afec.h
 */

/** \file
 *
 *  This file contains all the specific code for the AFE12 example.
 */

/*----------------------------------------------------------------------------
 *        Headers
 *----------------------------------------------------------------------------*/

#include "board.h"
#include <string.h>

/** SAMPLES per cycle*/
#define SAMPLES         100
#define TEST_CHANNEL    5
#define AFE_CLK         2200000

/* sine wave data */
const uint16_t sine_data[SAMPLES] = {
	1,    2,      5,     10,     17,     26,     37,     50,     65,     81,
	99,  119,    140,    163,    187,    212,    239,    266,    295,    325,
	355,  385,    417,    449,    481,    512,    543,    575,    607,    639,
	669,  699,    729,    758,    785,    812,    837,    861,    884,    905,
	925,  943,    959,    974,    987,    998,   1007,   1014,   1019,   1022,
	1023, 1022,   1019,   1014,   1007,    998,    987,    974,    959,    943,
	925,  905,    884,    861,    837,    812,    785,    758,    729,    699,
	669,  639,    607,    575,    543,    512,    481,    449,    417,    385,
	355,  325,    295,    266,    239,    212,    187,    163,    140,    119,
	99,   81,     65,     50,     37,     26,     17,     10,      5,      2 };

uint32_t afeBuffer[SAMPLES];
uint32_t dacBuffer[SAMPLES];

/** Global DMA driver for all transfer */
static sXdmad dmad;

/** Global AFE DMA instance */
static AfeDma Afed;
/** AFE command instance */
static AfeCmd AfeCommand;

/** Global DAC DMA instance */
static DacDma Dacd;
/** DAC command instance */
static DacCmd DacCommand;

/*----------------------------------------------------------------------------
 *        Local definitions
 *----------------------------------------------------------------------------*/

/**
 *  \brief xDMA interrupt handler.
 *
 */
void XDMAC_Handler(void)
{
	XDMAD_Handler(&dmad);
}

/**
 *  \brief Callback function for AFE interrupt
 *
 */
static void _afe_Callback(int dummy, void* pArg)
{
	uint32_t i;
	uint32_t ch;
	uint32_t voltage;
	dummy = dummy;
	pArg = pArg;
	printf("\n\rCH  AFE   Voltage(mV) \n\r");
	for (i = 0; i < SAMPLES; i++) {
		ch = (afeBuffer[i] & AFEC_LCDR_CHNB_Msk) >> AFEC_LCDR_CHNB_Pos;
		voltage = ((afeBuffer[i] & 0xFFFF ) - 0x800) * 1650 / 2047;
		printf("%02u  %04x  %04u\n\r" ,(unsigned int)ch,
			(unsigned int)(afeBuffer[i] & 0xFFFF) ,(unsigned int)voltage);
	}
}

/**
 *  \brief Initialize DAC.
 *
 */
static void _dac_initialization(void) {
	uint32_t i;
	PMC_EnablePeripheral(ID_DACC);
	DACC_SoftReset(DACC);
	DACC_EnableChannel(DACC, 0);
	for (i = 0; i < SAMPLES; i++)
		dacBuffer[i] = sine_data[i] << 1;
}

/**
 *  \brief Configure DAC DMA and start DMA transfer.
 *
 */
static void _dac_dmaTransfer(void)
{
	DacCommand.dacChannel = DACC_CHANNEL_0;
	DacCommand.TxSize = SAMPLES;
	DacCommand.pTxBuff = (uint8_t *)dacBuffer;
	DacCommand.loopback = 1;
	Dac_ConfigureDma(&Dacd, DACC, ID_DACC, &dmad);
	Dac_SendData(&Dacd, &DacCommand);
}

/**
 *  \brief Initialize AFE.
 *
 */
static void _afe_initialization(void) {
	AFEC_Initialize(AFEC0, ID_AFEC0);
	AFEC_SetModeReg(AFEC0,
			AFEC_MR_FREERUN_ON
			| AFEC_EMR_RES_NO_AVERAGE
			| (1 << AFEC_MR_TRANSFER_Pos)
			| (2 << AFEC_MR_TRACKTIM_Pos)
			| AFEC_MR_ONE
			| AFEC_MR_SETTLING_AST3
			| AFEC_MR_STARTUP_SUT64);

	AFEC_SetClock( AFEC0, AFE_CLK, BOARD_MCK);
	AFEC_SetExtModeReg(AFEC0,
			0
			| AFEC_EMR_RES_NO_AVERAGE
			| AFEC_EMR_TAG
			| AFEC_EMR_STM);
	AFEC_SetAnalogOffset(AFEC0, TEST_CHANNEL, 0x800);
	AFEC_SetAnalogControl(AFEC0, AFEC_ACR_IBCTL(1) | AFEC_ACR_PGA0_ON |
			AFEC_ACR_PGA1_ON);
	AFEC_EnableChannel(AFEC0, TEST_CHANNEL);
}

/**
 *  \brief Configure AFE DMA and start DMA transfer.
 *
 */
static void _afe_dmaTransfer(void)
{
	AfeCommand.RxSize= SAMPLES;
	AfeCommand.pRxBuff = afeBuffer;
	AfeCommand.callback = (AfeCallback)_afe_Callback;
	Afe_ConfigureDma(&Afed, AFEC0, ID_AFEC0, &dmad);
	Afe_SendData(&Afed, &AfeCommand);
}

/**
 *  \brief AFE12 Application entry point.
 *
 *  \return Unused (ANSI-C compatibility).
 */
int main( void )
{
	/* Disable watchdog */
	WDT_Disable(WDT);

	/* Enable I and D cache */
	SCB_EnableICache();
	SCB_EnableDCache();

	/* Output example information */
	printf("\n\r-- AFE12_dma Example %s --\n\r", SOFTPACK_VERSION);
	printf("-- %s\n\r", BOARD_NAME );
	printf("-- Compiled: %s %s With %s--\n\r", __DATE__, __TIME__, COMPILER_NAME);

	/* Initialize DAC */
	_dac_initialization();
	_dac_dmaTransfer();

	/* Initialize AFE */
	_afe_initialization();
	_afe_dmaTransfer();

	while (1);
}

