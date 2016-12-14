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
 * \page pwm PWM with DMA Example
 *
 * \section Purpose
 *
 * This example demonstrates a simple configuration of three PWM channels to
 * generate variable duty cycle signals. The update of the duty cycle values
 * is made automatically by the Peripheral DMA Controller .
 * This will cause LED on the evaluation kit to glow repeatedly.
 *
 * \section Requirements
 *
 * This package can be used with SAMV71 Xplained Ultra board or SAME70 Xplained board.
 *
 * \section Description
 *
 * Three PWM channels (channel #0) are configured to generate
 * a 50Hz PWM signal. The update of the duty cycle values is made
 * automatically by the DMA.
 *
 * \section Usage
 *
 *  -# Build the program and download it inside the board.
 *     Please refer to the Getting Started with SAM V71/E70 Microcontrollers.pdf
 * -# Optionally, on the computer, open and configure a terminal application
 *    (e.g. HyperTerminal on Microsoft Windows) with these settings:
 *   - 115200 baud rate
 *   - 8 bits of data
 *   - No parity
 *   - 1 stop bit
 *   - No flow control
 * -# Start the application.
 * \if document_SAMV71_XULT
 * -# Depending on the board being used, the LED will start glowing repeatedly.
 * \elseif document_SAME70_XPRO
 * -# due to the LED is not connected to the PWM output pin, the PWM signal can
 * be measured on J507 PIN24, if the pin is connected to the LED, the LED will
 * start glowing repeatedly.
 * \endif
 * -# Select one or more options to set the configuration of PWM channel.
 *
 * \section References
 * - pwm_pdc/main.c
 * - pwm.c
 * - pwm.h
 */

/**
 * \file
 *
 * This file contains all the specific code for the pwm_pdc example.
 */

/*----------------------------------------------------------------------------
 *        Headers
 *----------------------------------------------------------------------------*/

#include "board.h"

#include <stdint.h>
#include <stdio.h>

/*----------------------------------------------------------------------------
 *        Local definitions
 *----------------------------------------------------------------------------*/

/** PWM frequency in Hz. */
#define PWM_FREQUENCY               50

/** Maximum duty cycle value. */
#define MAX_DUTY_CYCLE              100
/** Minimum duty cycle value. */
#define MIN_DUTY_CYCLE              0

/** Duty cycle buffer length for three channels */
#define DUTY_BUFFER_LENGTH          (MAX_DUTY_CYCLE - MIN_DUTY_CYCLE )

/*----------------------------------------------------------------------------
 *        Local variables
 *----------------------------------------------------------------------------*/

/** PIO pins to configure. */

#define PIN_PWM_LED   { PIO_PA23B_PWMC0_PWMH0, PIOA, ID_PIOA, PIO_PERIPH_B, PIO_DEFAULT}
#define CHANNEL_PWM_LED0  0
//
static const Pin pinPwm[] = {PIN_PWM_LED};

/** duty cycle buffer*/
COMPILER_ALIGNED(32) static uint16_t dwDutys[DUTY_BUFFER_LENGTH];

/** Global DMA driver for all transfer */
static sXdmad dmad;

/** DMA channel for TX */
static uint32_t pwmDmaTxChannel;
COMPILER_ALIGNED(32) static LinkedListDescriporView1 dmaWriteLinkList[DUTY_BUFFER_LENGTH];

/*----------------------------------------------------------------------------
 *        Local functions
 *----------------------------------------------------------------------------*/

/**
 * Interrupt handler for the XDMAC.
 */
void XDMAC_Handler(void)
{
	XDMAD_Handler(&dmad );
}

/**
 * Interrupt handler for the PWMC.
 */
void PWM0_Handler(void)
{
	volatile uint32_t dummySt;
	/* WRDY: this flag is set to'1' when the PWM Controller is ready to receive
	new duty-cycle values and a new update period value. It is reset to '0'
	when the PWM_ISR2 is read.*/
	dummySt = PWMC_GetStatus2(PWM0);
}

/**
 * \brief xDMA driver configuration
 */
static void _ConfigureDma(void)
{
	sXdmad *pDmad = &dmad;
	/* Driver initialize */
	XDMAD_Initialize( pDmad, 0 );
	/* Allocate DMA channels for PWM */
	pwmDmaTxChannel = XDMAD_AllocateChannel(pDmad, XDMAD_TRANSFER_MEMORY, ID_PWM0);
	if (pwmDmaTxChannel == XDMAD_ALLOC_FAILED) {
		printf("xDMA channel allocation error\n\r");
		while (1);
	}
	XDMAD_PrepareChannel(pDmad, pwmDmaTxChannel);
	/* Configure interrupt for DMA transfer */
	NVIC_ClearPendingIRQ(XDMAC_IRQn);
	NVIC_SetPriority( XDMAC_IRQn ,1);
	NVIC_EnableIRQ(XDMAC_IRQn);
}

/**
 * \brief xDMA transfer PWM duty
 */
static void _PwmDmaTransfer(void)
{
	sXdmadCfg xdmadCfg;
	uint32_t xdmaCndc;
	uint32_t i;

	for (i = 0; i < DUTY_BUFFER_LENGTH; i++) {
		dmaWriteLinkList[i].mbr_ubc = XDMA_UBC_NVIEW_NDV1
			| XDMA_UBC_NDE_FETCH_EN
			| XDMA_UBC_NSEN_UPDATED
			| XDMAC_CUBC_UBLEN(1);
		dmaWriteLinkList[i].mbr_sa = (uint32_t)(&dwDutys[i]);
		dmaWriteLinkList[i].mbr_da = (uint32_t)(&(PWM0->PWM_DMAR));
		if (i == (DUTY_BUFFER_LENGTH - 1))
			dmaWriteLinkList[i].mbr_nda = (uint32_t)&dmaWriteLinkList[0];
		else
			dmaWriteLinkList[i].mbr_nda = (uint32_t)&dmaWriteLinkList[i+1];
	}

	xdmadCfg.mbr_cfg = XDMAC_CC_TYPE_PER_TRAN
		| XDMAC_CC_MBSIZE_SINGLE
		| XDMAC_CC_DSYNC_MEM2PER
		| XDMAC_CC_CSIZE_CHK_1
		| XDMAC_CC_DWIDTH_HALFWORD
		| XDMAC_CC_SIF_AHB_IF1
		| XDMAC_CC_DIF_AHB_IF1
		| XDMAC_CC_SAM_FIXED_AM
		| XDMAC_CC_DAM_FIXED_AM
		| XDMAC_CC_PERID(XDMAIF_Get_ChannelNumber(ID_PWM0, XDMAD_TRANSFER_TX ));
	xdmaCndc = XDMAC_CNDC_NDVIEW_NDV1
		| XDMAC_CNDC_NDE_DSCR_FETCH_EN
		| XDMAC_CNDC_NDSUP_SRC_PARAMS_UPDATED
		| XDMAC_CNDC_NDDUP_DST_PARAMS_UPDATED;
	XDMAD_ConfigureTransfer( &dmad, pwmDmaTxChannel, &xdmadCfg, xdmaCndc,
			(uint32_t)&dmaWriteLinkList[0], XDMAC_CIE_LIE);
	SCB_CleanDCache_by_Addr((uint32_t *)dwDutys, sizeof(dwDutys));
	SCB_CleanDCache_by_Addr((uint32_t *)dmaWriteLinkList, sizeof(dmaWriteLinkList));
	XDMAD_StartTransfer(&dmad, pwmDmaTxChannel);
}

/*----------------------------------------------------------------------------
 *         Global functions
 *----------------------------------------------------------------------------*/

/**
 * \brief Application entry point for PWM with PDC example.
 *
 * Outputs a PWM on LED1.
 * Channel #0 is configured as synchronous channels.
 * The update of the duty cycle values is made automatically by the Peripheral
 * DMA Controller.
 *
 * \return Unused (ANSI-C compatibility).
 */
int main(void)
{
	uint32_t i;

	/* Disable watchdog */
	WDT_Disable(WDT);

	/* Enable I and D cache */
	SCB_EnableICache();
	SCB_EnableDCache();

	/* Output example information */
	printf("-- PWM with DMA Example %s --\n\r", SOFTPACK_VERSION);
	printf("-- %s\n\r", BOARD_NAME);
	printf("-- Compiled: %s %s With %s--\n\r", __DATE__, __TIME__, COMPILER_NAME);

	/* PIO configuration */
	PIO_Configure(pinPwm, PIO_LISTSIZE(pinPwm));
	for (i = 0; i < DUTY_BUFFER_LENGTH; i++)
		dwDutys[i] = i / 2;

	/* Enable PWMC peripheral clock */
	PMC_EnablePeripheral(ID_PWM0);

	/* Configure interrupt for PWM transfer */
	NVIC_DisableIRQ(PWM0_IRQn);
	NVIC_ClearPendingIRQ(PWM0_IRQn);
	NVIC_SetPriority(PWM0_IRQn, 0);

	/* Configure DMA channel for PWM transfer */
	_ConfigureDma();

	/* Set clock A to run at PWM_FREQUENCY * MAX_DUTY_CYCLE (clock B is not
		used) */
	PWMC_ConfigureClocks(PWM0, PWM_FREQUENCY * MAX_DUTY_CYCLE , 0, BOARD_MCK);

	/* Configure PWMC channel for LED0 (left-aligned, enable dead time
		generator) */
	PWMC_ConfigureChannel( PWM0,
			0,  /* channel */
			PWM_CMR_CPRE_CLKA,  /* prescaler, CLKA  */
			0,                  /* alignment */
			0                   /* polarity */
			);

	PWMC_ConfigureSyncChannel(PWM0,
			/* Define the synchronous channels by the bits SYNCx */
			(1 << CHANNEL_PWM_LED0),
			/* Select the manual write of duty-cycle values and the automatic
				update by setting the field UPDM  */
			PWM_SCM_UPDM_MODE2,
			0,
			0);

	/* Configure channel 0 period */
	PWMC_SetPeriod(PWM0, 0, DUTY_BUFFER_LENGTH);
	/* Configure channel 0 duty cycle */
	PWMC_SetDutyCycle(PWM0, 0, MIN_DUTY_CYCLE);
	/* Define the update period by the field UPR in the PWM_SCUP register*/
	PWMC_SetSyncChannelUpdatePeriod(PWM0, 8);
	/* Enable the synchronous channels by writing CHID0 in the PWM_ENA register */
	PWMC_EnableChannel(PWM0, 0);
	/* Enable PWM interrupt */
	PWMC_EnableIt(PWM0, 0, PWM_IER2_WRDY);
	NVIC_EnableIRQ(PWM0_IRQn);
	_PwmDmaTransfer();
	while (1);
}
