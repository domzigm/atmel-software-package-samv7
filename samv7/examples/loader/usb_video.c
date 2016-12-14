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

/*----------------------------------------------------------------------------
 *        Headers
 *----------------------------------------------------------------------------*/

#include "board.h"

/*----------------------------------------------------------------------------
 *        Local definitions
 *----------------------------------------------------------------------------*/

/** TWI clock frequency in Hz. */
#define TWCK            400000

/** TWI Pins definition */
#define BOARD_PINS_TWI_ISI PINS_TWI0
/** TWI peripheral ID for Sensor configuration */
#define BOARD_ID_TWI_ISI        ID_TWIHS0
/** TWI base address for Sensor configuration */
#define BOARD_BASE_TWI_ISI      TWIHS0


/*----------------------------------------------------------------------------
 *          Exported functions
 *----------------------------------------------------------------------------*/
uint32_t UVC_preCondition(void);


/*----------------------------------------------------------------------------
 *        External variables
 *----------------------------------------------------------------------------*/
extern const sensorProfile_t ov2643Profile;
extern const sensorProfile_t ov5640Profile;
extern const sensorProfile_t ov7740Profile;
extern const sensorProfile_t ov9740Profile;

static const sensorProfile_t * sensorProfiles[] = {
	&ov2643Profile, &ov5640Profile, &ov7740Profile, &ov9740Profile};

static const char sensorName[][8] = {
	"OV2643", "OV5640", "OV7740", "OV9740"
};


/*----------------------------------------------------------------------------
 *        Local variables
 *----------------------------------------------------------------------------*/

/** ISI pins to configure. */
const Pin pinsTWI[] = BOARD_PINS_TWI_ISI;
const Pin pin_ISI_RST = BOARD_ISI_RST;
const Pin pin_ISI_PWD = BOARD_ISI_PWD;
const Pin pPinsISI[]= {BOARD_ISI_PINS};

/** TWI driver instance.*/
static Twid twid;


/*----------------------------------------------------------------------------
 *        Local functions
 *----------------------------------------------------------------------------*/
/**
 * \brief TWI interrupt handler. Forwards the interrupt to the TWI driver handler.
 */
void TWIHS0_Handler(void)
{
	TWID_Handler( &twid );
}

/**
 * \brief TWI initialization.
 */
static void _twiInit(void)
{
	/* Configure TWI pins. */
	PIO_Configure(pinsTWI, PIO_LISTSIZE(pinsTWI));
	/* Enable TWI peripheral clock */
	PMC_EnablePeripheral(BOARD_ID_TWI_ISI);
	/* Configure TWI */
	TWI_ConfigureMaster(BOARD_BASE_TWI_ISI, TWCK, BOARD_MCK);
	TWID_Initialize(&twid, BOARD_BASE_TWI_ISI);

	/* Configure TWI interrupts */
	NVIC_ClearPendingIRQ(TWIHS0_IRQn);
	NVIC_EnableIRQ(TWIHS0_IRQn);
}

/**
 * \brief ISI PCK initialization.
 */
static void _isiPckInit(void)
{
	/* Configure ISI pins. */
	PIO_Configure(pPinsISI, PIO_LISTSIZE(pPinsISI));

	/* Disable programmable clock 1 output */
	REG_PMC_SCDR = PMC_SCER_PCK0;
	/* Enable the DAC master clock */
	PMC->PMC_PCK[0] = PMC_PCK_CSS_PLLA_CLK | (9 << 4);
	/* Enable programmable clock 0 output */
	REG_PMC_SCER = PMC_SCER_PCK0;
	/* Wait for the PCKRDY0 bit to be set in the PMC_SR register*/
	while ((REG_PMC_SR & PMC_SR_PCKRDY0) == 0);
	/* ISI PWD OFF*/
	PIO_Clear(&pin_ISI_PWD);
	PIO_Clear(&pin_ISI_RST);
}

/*----------------------------------------------------------------------------
 *        Exported functions
 *----------------------------------------------------------------------------*/
uint32_t UVC_preCondition(void)
{
	uint32_t rc = 1;
	uint32_t i;

	TimeTick_Configure();

	/* TWI Initialize */
	_twiInit();

	/* ISI PCK clock Initialize */
	_isiPckInit();
	PIO_Set(&pin_ISI_RST);

	for (i = 0; i < (sizeof(sensorProfiles)/sizeof(sensorProfiles[0])); i++) {
		if (sensor_setup(&twid, sensorProfiles[i], QVGA) == SENSOR_OK){
			printf("\n\r-I- Sensor %s setup succeed.", sensorName[i]);
			rc = 0;
			break;
		}
	}
	if (rc) {
		printf("\n\r-I- No image sensor detected.");
	}

	NVIC_DisableIRQ(TWIHS0_IRQn);
	return rc;
}
