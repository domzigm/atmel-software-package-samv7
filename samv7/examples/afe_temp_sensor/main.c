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
 * \page afe_temp_sensor AFE temp sensor Example
 *
 * \section Purpose
 *
 * The example is aimed to demonstrate the temperature sensor feature
 * inside the device. The channel 11 is connected to the sensor by default.
 *
 * \section Requirements
 *
 * This package can be used with SAMV71 Xplained Ultra board or SAME70 Xplained board.
 *
 * \section Description
 *
 * The temperature sensor provides an output voltage (VT) that is proportional
 * to absolute temperature (PTAT). The relationship between measured voltage and
 * actual temperature could be found in Electrical Characteristics part of the
 * datasheet.
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
 *     following text should appear (values depend on the board and chip used):
 *    \code
 *     -- AFE12 Example xxx --
 *     -- xxxxxx-xx
 *     -- Compiled: xxx xx xxxx xx:xx:xx --
 *    \endcode
 * -# The application will output converted value to hyper-terminal and display
 *    a menu for user to show current temperature.
 *
 * \section References
 * - afe_temp_sensor /main.c
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

/*----------------------------------------------------------------------------
 *        Definition
 *----------------------------------------------------------------------------*/

/** SAMPLES per cycle*/
#define SAMPLES (100)

/** Reference voltage for AFEC in mv. */
#define VOLT_REF        (3300)

/** The maximal digital value */
#define MAX_DIGITAL_12_BIT     (4095UL)
#define AFEC_TEMPERATURE_SENSOR  11

#define AFEC_ACR_PGA0_ON     (0x1u << 2)
#define AFEC_ACR_PGA1_ON     (0x1u << 3)

/*----------------------------------------------------------------------------
 *        Local variables
 *----------------------------------------------------------------------------*/
/** Global DMA driver for all transfer */
static sXdmad dmad;

/** Global AFE DMA instance */
static AfeDma Afed;

/** AFE command instance */
static AfeCmd AfeCommand;

/** AFE output value */
COMPILER_ALIGNED(32) uint32_t afeOutput;

/*----------------------------------------------------------------------------
 *        Local Functions
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
 *  \brief Initialize AFE.
 *
 */
static void _afe_initialization(void) {
	AFEC_Initialize( AFEC0, ID_AFEC0 );

	AFEC_SetModeReg(AFEC0, 0
			| AFEC_EMR_RES_NO_AVERAGE
			| (1 << AFEC_MR_TRANSFER_Pos)
			| (2 << AFEC_MR_TRACKTIM_Pos)
			| AFEC_MR_ONE
			| AFEC_MR_SETTLING_AST3
			| AFEC_MR_STARTUP_SUT64);

	AFEC_SetClock(AFEC0, 6000000, BOARD_MCK);

	AFEC_SetExtModeReg(AFEC0, 0
			| AFEC_EMR_RES_NO_AVERAGE
			| AFEC_EMR_TAG
			| AFEC_EMR_STM );

	AFEC_EnableChannel(AFEC0, AFEC_TEMPERATURE_SENSOR);
	AFEC0->AFEC_ACR = AFEC_ACR_IBCTL(2)
		| (1 << 4)
		| AFEC_ACR_PGA0_ON
		| AFEC_ACR_PGA1_ON;

	AFEC_SetChannelGain(AFEC0, AFEC_CGR_GAIN11(0));
	AFEC_SetAnalogOffset(AFEC0, AFEC_TEMPERATURE_SENSOR, 0x200);
}

/**
 *  \brief Callback function for AFE interrupt
 *
 */
static void _afe_Callback(int dummy, void* pArg)
{
	uint32_t afeValue;
	dummy = dummy;
	pArg = pArg;
	afeValue = (afeOutput & AFEC_LCDR_LDATA_Msk) * VOLT_REF / MAX_DIGITAL_12_BIT;
	/* According to datasheet, The output voltage VT = 0.72V at 27C and the
		temperature slope dVT/dT = 2.33 mV/C */
	printf("\n\r Temperature is: %4d ", (int) (afeValue - 720) * 100 / 233 + 27);
}

/**
 *  \brief Configure AFE DMA and start DMA transfer.
 *
 */
static void _afe_dmaTransfer(void)
{
	AfeCommand.RxSize= 1;
	AfeCommand.pRxBuff = &afeOutput;
	AfeCommand.callback = (AfeCallback)_afe_Callback;
	Afe_ConfigureDma(&Afed, AFEC0, ID_AFEC0, &dmad);
	Afe_SendData(&Afed, &AfeCommand);
}

/**
 *  \brief afe_temp_sensor Application entry point.
 *
 *  \return Unused (ANSI-C compatibility).
 */
int main( void )
{
	uint8_t key;
	/* Disable watchdog */
	WDT_Disable(WDT);

	/* Enable I and D cache */
	SCB_EnableICache();
	SCB_EnableDCache();

	/* Output example information */
	printf("\n\r-- AFE Temperature Sensor Example %s --\n\r", SOFTPACK_VERSION);
	printf("-- %s\n\r", BOARD_NAME );
	printf("-- Compiled: %s %s With %s--\n\r", __DATE__, __TIME__ , COMPILER_NAME);

	_afe_initialization();
	printf("\n\r Press 't' to get current temperature");
	for(;;) {
		key = DBG_GetChar();
		if ((key == 't')) {
			_afe_dmaTransfer();
		}
	}
}
