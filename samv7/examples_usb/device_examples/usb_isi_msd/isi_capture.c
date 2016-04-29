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

/*------------------------------------------------------------------------------
 *         Headers
 *----------------------------------------------------------------------------*/

#include "board.h"

/*------------------------------------------------------------------------------
 *         Internal definitions
 *----------------------------------------------------------------------------*/


/*------------------------------------------------------------------------------
 *         Macros
 *----------------------------------------------------------------------------*/

/** TWI clock frequency in Hz. */
#define TWCK                    400000
/** TWI peripheral ID for Sensor configuration */
#define BOARD_ID_TWI_ISI        ID_TWIHS0
/** TWI base address for Sensor configuration */
#define BOARD_BASE_TWI_ISI      TWIHS0

/** Frame Buffer Descriptors */
#define ISI_MAX_PREV_BUFFER     1

/** TWI Pins definition */
#define BOARD_PINS_TWI_ISI      PINS_TWI0

/*------------------------------------------------------------------------------
 *         Global variables
 *----------------------------------------------------------------------------*/

extern const sensorProfile_t ov2640Profile;
extern const sensorProfile_t ov2643Profile;
extern const sensorProfile_t ov5640Profile;
extern const sensorProfile_t ov7740Profile;
extern const sensorProfile_t ov9740Profile;
extern const sensorProfile_t ov7670Profile;

/** Supported sensor profiles */
static const sensorProfile_t *sensorsProfile[6] = {&ov2640Profile,
												   &ov2643Profile,
												   &ov5640Profile,
												   &ov7740Profile,
												   &ov9740Profile,
												   &ov7670Profile
												  };

/** Buffer for ISI capture */
extern uint16_t gIsiBuffer[];

/** Image size */
extern uint32_t wImageWidth, wImageHeight;

/*------------------------------------------------------------------------------
 *         Local variables
 *----------------------------------------------------------------------------*/

/** ISI pins to configure. */
static const Pin pinsTWI[] = BOARD_PINS_TWI_ISI;
static const Pin pin_ISI_RST = BOARD_ISI_RST;
static const Pin pin_ISI_PWD = BOARD_ISI_PWD;
static const Pin pPinsISI[] = {BOARD_ISI_PINS};

/** TWI driver instance.*/
static Twid twid;

/* ISI DMA descriptor for preview path */
static COMPILER_WORD_ALIGNED ISI_FrameBufferDescriptors
preBufDescList[ISI_MAX_PREV_BUFFER];

/* ISI DMA descriptor for codec path */
static COMPILER_WORD_ALIGNED ISI_FrameBufferDescriptors
codecBufDescList[ISI_MAX_PREV_BUFFER];

/*------------------------------------------------------------------------------
 *         Global functions
 *----------------------------------------------------------------------------*/
int32_t isi_init(uint32_t index);
void isi_capture_preview(void *pBuffer);
void isi_capture_codec(void *pBuffer);

/*----------------------------------------------------------------------------
 *        Local functions
 *----------------------------------------------------------------------------*/

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
	PMC->PMC_PCK[0] = PMC_PCK_CSS_MCK | (2 << 4);
	/* Enable programmable clock 0 output */
	REG_PMC_SCER = PMC_SCER_PCK0;

	/* Wait for the PCKRDY0 bit to be set in the PMC_SR register*/
	while ((REG_PMC_SR & PMC_SR_PCKRDY0) == 0);

	/* ISI PWD OFF*/

	PIO_Clear(&pin_ISI_PWD);
	PIO_Clear(&pin_ISI_RST);
}


/**
 * \brief ISI  initialization.
 */
static void _isiInit(void)
{
	ISI_Y2R y2r;

	/* Enable ISI peripheral clock */
	PMC_EnablePeripheral(ID_ISI);
	/* Reset ISI peripheral */
	ISI_Reset();
	/* Set the windows blank */
	ISI_SetBlank(0, 0);
	/* Set vertical and horizontal Size of the Image Sensor for preview path*/
	ISI_SetSensorSize(wImageWidth, wImageHeight);
	/*  Set data stream in YUV format.*/
	ISI_setInputStream(YUV_INPUT);
	/* Defines YCrCb swap format.*/
	ISI_YCrCbFormat(ISI_CFG2_YCC_SWAP_MODE1);

	ISI_setPreviewSize(wImageWidth, wImageHeight);

	/* calculate scaler factor automatically. */
	ISI_calcScalerFactor();

	/* Set Matrix color space for YUV to RBG convert */
	y2r.C0 = 0x95;
	y2r.C1 = 0xFF;
	y2r.C2 = 0x68;
	y2r.C3 = 0x32;
	y2r.C4 = 0xCC;
	y2r.Yoff = 1;
	y2r.Croff = 1;
	y2r.Cboff = 1;
	ISI_SetMatrix4Yuv2Rgb(&y2r);
}

/*----------------------------------------------------------------------------
 *        Exported functions
 *----------------------------------------------------------------------------*/

/**
 * \brief initialize ISI sensor for capturing images
 * \param index          index of sensor profiles
 * \param pwImageWidth   pointer to the image width
 * \param pwImageHeight  pointer to the image heigth
 */
int32_t isi_init(uint32_t index)
{
	/* Image output format */
	sensorOutputFormat_t wImageFormat;

	/* TWI Initialize */
	_twiInit();

	/* ISI PCK clock Initialize */
	_isiPckInit();

	PIO_Set(&pin_ISI_RST);

	if (sensor_setup(&twid, sensorsProfile[index], VGA) != SENSOR_OK) {
		printf("-E- Sensor setup failed.");
		return -1;
	}

	/* Retrieve sensor output format and size */
	sensor_get_output(&wImageFormat, &wImageWidth, &wImageHeight, VGA);

	/* ISI Initialize */
	_isiInit();

	ISI_Enable();

	return 0;
}

/**
 * \brief Capture images using ISI sensor
 * \param pBuffer   pointer to the buffer image to store in
 */
void isi_capture_preview(void *pBuffer)
{
	volatile uint32_t delay;

	ISI_YCrCbFormat(ISI_CFG2_YCC_SWAP_MODE2);

	for (delay = 0; delay < 0xffff; delay++);

	/* parpare buffer descriptor */
	preBufDescList[0].Current = (uint32_t)pBuffer;
	preBufDescList[0].Control = ISI_DMA_P_CTRL_P_WB;
	preBufDescList[0].Next    = NULL;
	ISI_setDmaInPreviewPath((uint32_t)&preBufDescList[0], ISI_DMA_P_CTRL_P_FETCH,
							(uint32_t)pBuffer);

	SCB_CleanInvalidateDCache();

	/* capture image */
	ISI_DmaChannelEnable(ISI_DMA_CHER_P_CH_EN);

	for (delay = 0; delay < 0xffff; delay++);

	//  ISI->ISI_CR |= ISI_CR_ISI_CDC;

	/* wait capture finished */
	//  while (ISI_SR_PXFR_DONE == (ISI->ISI_SR & ISI_SR_PXFR_DONE));
	while (ISI_DMA_P_CTRL_P_DONE != (preBufDescList[0].Control &
									 ISI_DMA_P_CTRL_P_DONE))
		SCB_InvalidateDCache();

	for (delay = 0; delay < 0x5ffff; delay++);

	ISI_DmaChannelDisable(ISI_DMA_CHDR_P_CH_DIS);
}


/**
 * \brief Capture images using ISI sensor
 * \param pBuffer   pointer to the buffer image to store in
 */
void isi_capture_codec(void *pBuffer)
{
	volatile uint32_t delay;

	ISI_YCrCbFormat(ISI_CFG2_YCC_SWAP_MODE1);

	for (delay = 0; delay < 0xffff; delay++);

	/* parpare buffer descriptor */
	codecBufDescList[0].Current = (uint32_t)pBuffer;
	codecBufDescList[0].Control = ISI_DMA_C_CTRL_C_WB;
	codecBufDescList[0].Next    = NULL;
	ISI_setDmaInCodecPath((uint32_t)&codecBufDescList[0], ISI_DMA_C_CTRL_C_FETCH,
						  (uint32_t)pBuffer);

	SCB_CleanInvalidateDCache();

	/* capture image */
	ISI_DmaChannelEnable(ISI_DMA_CHER_C_CH_EN);

	for (delay = 0; delay < 0xffff; delay++);

	ISI->ISI_CR |= ISI_CR_ISI_CDC;

	/* wait capture finished */
	while (ISI_DMA_C_CTRL_C_DONE != (codecBufDescList[0].Control &
									 ISI_DMA_C_CTRL_C_DONE))
		SCB_InvalidateDCache();

	for (delay = 0; delay < 0x5ffff; delay++);

	ISI_DmaChannelDisable(ISI_DMA_CHDR_C_CH_DIS);
}

