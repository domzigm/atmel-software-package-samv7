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
 * \page isi ISI Example
 *
 * \section Purpose
 *
 * This example demonstrates the ISI (Image Sensor Interface) of an SAM V71
 * Xplained Ultra.
 *
 * \section Requirements
 *
 * This package can be used with SAM V71 Xplained Ultra  with
 * On-board ISI interface and a external CMOS-type image sensor board.
 *
 * \section Description
 * The provided program uses the Image Sensor Interface to connects a CMOS-type
 * image sensor to the processor and displays in VGA format.
 *
 *  \section Note
 *
 *  Some pins conflict between LCD pins and JTAG pins, this example can not run
 * in debug mode.
 *
 * \section Usage
 *  -# Build the program and download it inside the SAM V71 Xplained Ultra board.
 *  Please refer to the Getting Started with SAM V71 Microcontrollers.pdf
 * -# On the computer, open and configure a terminal application
 *    (e.g. HyperTerminal on Microsoft Windows) with these settings:
 *   - 115200 bauds
 *   - 8 bits of data
 *   - No parity
 *   - 1 stop bit
 *   - No flow control
 * -# Start the application.
 * -# In the terminal window, the following text should appear:
 *    \code
 *     -- ISI Example xxx --
 *      -- SAMxxxxx-xx
 *     -- Compiled: xxx xx xxxx xx:xx:xx --
 *    \endcode
 * The user can then choose any of the available options to perform the
 * described action.
 *
 * \section References
 * - lcdc.c
 * - twi.c
 * - twid.c
 * - isi.c
 */

/**
 * \file
 *
 * This file contains all the specific code for the ISI example.
 */

/*----------------------------------------------------------------------------
 *        Headers
 *----------------------------------------------------------------------------*/

#include "board.h"
#include "string.h"

/*----------------------------------------------------------------------------
 *        Local definitions
 *----------------------------------------------------------------------------*/

COMPILER_SECTION("sdram_region")
static uint16_t gIsiBuffer[640 * 480 * 2];

#define ISI_BASE                gIsiBuffer

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

#define LCD_NORMAL_MODE   0
#define LCD_REVERSE_MODE  1    // Row/column exchange
#define LCD_MODE            ILI9488_EBIMODE

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

/* ISI DMA descriptor for preview path */
COMPILER_ALIGNED(32) ISI_FrameBufferDescriptors preBufDescList[ISI_MAX_PREV_BUFFER];

/* Image size in preview mode */
static uint32_t wImageWidth, wImageHeight;

/* Image output format */
static sensorOutputFormat_t wImageFormat;

/* Display size in preview mode */
static uint32_t wDisplayWidth, wDisplayHeight;

/* LCD display mode Normal or reverse mode */
static uint8_t lcdDisplayMode = LCD_REVERSE_MODE;

/** Global DMA driver for all transfer */
static sXdmad lcdEbiDma;

extern const sensorProfile_t ov2640Profile;
extern const sensorProfile_t ov2643Profile;
extern const sensorProfile_t ov5640Profile;
extern const sensorProfile_t ov7740Profile;
extern const sensorProfile_t ov9740Profile;

/** Supported sensor profiles */
static const sensorProfile_t *sensorsProfile[5] = {&ov2640Profile,
                                                  &ov2643Profile,
                                                  &ov5640Profile,
                                                  &ov7740Profile,
                                                  &ov9740Profile
};

/*----------------------------------------------------------------------------
 *        Local functions
 *----------------------------------------------------------------------------*/
/**
 * \brief TWI interrupt handler. Forwards the interrupt to the TWI driver handler.
 */
void TWIHS0_Handler(void)
{
	TWID_Handler(&twid);
}

/**
 * ISR for XDMA interrupt
 */
void XDMAC_Handler(void)
{
	XDMAD_Handler(&lcdEbiDma);
}

/**
 * \brief ISI interrupt handler.
 */
void ISI_Handler(void)
{
	uint32_t status,imr;
	status = ISI->ISI_SR;
	imr = ISI->ISI_IMR;
	if ((status & ISI_SR_PXFR_DONE) && (imr & ISI_IMR_PXFR_DONE)) {
		ISI_DmaChannelDisable(ISI_DMA_CHDR_P_CH_DIS);
		ISI_DisableInterrupt(ISI_IDR_PXFR_DONE);
		LCDD_UpdateWindow();
		ISI_DmaChannelEnable(ISI_DMA_CHER_P_CH_EN);
		ISI_EnableInterrupt(ISI_IER_PXFR_DONE);
	}
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
	PMC->PMC_PCK[0] = PMC_PCK_CSS_MCK | (4 << 4);
	/* Enable programmable clock 0 output */
	REG_PMC_SCER = PMC_SCER_PCK0;
	/* Wait for the PCKRDY0 bit to be set in the PMC_SR register*/
	while ((REG_PMC_SR & PMC_SR_PCKRDY0) == 0);
	/* ISI PWD OFF*/

	PIO_Clear(&pin_ISI_PWD);
	PIO_Clear(&pin_ISI_RST);
}

/**
 * \brief Set up Frame Buffer Descriptors(FBD) for preview path.
 */
static void _isi_AllocateFBD(void)
{
	uint32_t i;
	for(i = 0; i < ISI_MAX_PREV_BUFFER; i++) {
		preBufDescList[i].Current = (uint32_t)ISI_BASE;
		preBufDescList[i].Control = ISI_DMA_P_CTRL_P_FETCH;
		preBufDescList[i].Next    = (uint32_t)&preBufDescList[0];
	}
}

/**
 * \brief Configure LCD
 */
static void  _lcd_Configure(void)
{
	rect rc;
	LCDD_Initialize(LCD_MODE, &lcdEbiDma, lcdDisplayMode);
	LCDD_SetCavasBuffer((void*)gIsiBuffer, BOARD_LCD_HEIGHT * BOARD_LCD_WIDTH * 2);
	//LCDD_SetCavasBuffer((void*)gIsiBuffer, 320 * 240 * 2);
	rc.x = 0;
	rc.y = 0;
	if (lcdDisplayMode == LCD_REVERSE_MODE) {
		rc.width = BOARD_LCD_HEIGHT - 1;
		rc.height = BOARD_LCD_WIDTH - 1;
		//rc.width = 320 - 1;
		//rc.height = 240 - 1;
	} else {
		rc.width = BOARD_LCD_WIDTH - 1;
		rc.height = BOARD_LCD_HEIGHT - 1;
	}
	LCDD_SetUpdateWindowSize(rc);
	LCDD_DrawRectangleWithFill(0, 0, 0, rc.width - 1, rc.height - 1,
			RGB_24_TO_RGB565(COLOR_BLACK));
	LCDD_UpdateWindow();
	Wait(500);
}

/**
 * \brief ISI  initialization.
 */
static void _isiInit(void)
{
	ISI_Y2R y2r;

	/* Enable ISI peripheral clock */
	PMC_EnablePeripheral(ID_ISI);
	/* Set up Frame Buffer Descriptors(FBD) for preview path. */
	_isi_AllocateFBD();
	/* Reset ISI peripheral */
	ISI_Reset();
	/* Set the windows blank */
	ISI_SetBlank(0, 0);
	/* Set vertical and horizontal Size of the Image Sensor for preview path*/
	ISI_SetSensorSize(wImageWidth, wImageHeight);

	/* Set preview size to fit LCD size*/
	if (lcdDisplayMode == LCD_REVERSE_MODE) {
		/* LCD Height = BOARD_LCD_WIDTH  LCD Width = BOARD_LCD_HEIGHT */
		wDisplayHeight = wImageHeight * 1000 / (wImageWidth * 1000 / BOARD_LCD_HEIGHT);
		wDisplayWidth = BOARD_LCD_HEIGHT;
		ISI_setPreviewSize(wDisplayWidth, wDisplayHeight);

	} else {
		/* LCD Height = BOARD_LCD_HEIGHT  LCD Width = BOARD_LCD_WIDTH */
		wDisplayHeight = wImageHeight * 1000 / (wImageWidth * 1000 / BOARD_LCD_WIDTH);
		wDisplayWidth = BOARD_LCD_WIDTH;
		ISI_setPreviewSize(BOARD_LCD_WIDTH, wDisplayHeight );
	}
	/*  Set data stream in YUV format.*/
	ISI_setInputStream(YUV_INPUT);
	/* Defines YCrCb swap format.*/
	ISI_YCrCbFormat(ISI_CFG2_YCC_SWAP_MODE2);
	/* calculate scaler factor automatically. */
	ISI_calcScalerFactor();
	/* Configure DMA for preview path. */
	ISI_setDmaInPreviewPath((uint32_t)&preBufDescList[0], ISI_DMA_P_CTRL_P_FETCH,
			(uint32_t)ISI_BASE);
	/* */
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
	NVIC_ClearPendingIRQ(ISI_IRQn);
	NVIC_SetPriority(ISI_IRQn, 7);
	NVIC_EnableIRQ(ISI_IRQn);
}

/*----------------------------------------------------------------------------
 *        Global functions
 *----------------------------------------------------------------------------*/
/**
 * \brief Application entry point for ISI example.
 *
 * \return Unused (ANSI-C compatibility).
 */
extern int main( void )
{
	uint8_t key;
	WDT_Disable(WDT);

	/* Enable I and D cache */
	SCB_EnableICache();
	SCB_EnableDCache();

	/* Output example information */
	printf("-- ISI Example %s --\n\r", SOFTPACK_VERSION);
	printf("-- %s\n\r", BOARD_NAME );
	printf("-- Compiled: %s %s With %s--\n\r", __DATE__, __TIME__, COMPILER_NAME);

	/* Enable SDRAM for Frame buffer */
	BOARD_ConfigureSdram();

	/* Configure Time Tick */
	TimeTick_Configure();

	/* Configure LCD */
	_lcd_Configure();

	/* TWI Initialize */
	_twiInit();

	/* ISI PCK clock Initialize */
	_isiPckInit();

	PIO_Set(&pin_ISI_RST);

	printf("Press [0|1|2|3|4][5] to select supported sensor \n\r");
	printf("- '0' omnivision 2640 \n\r");
	printf("- '1' omnivision 2643 \n\r");
	printf("- '2' omnivision 5640 \n\r");
	printf("- '3' omnivision 7740 \n\r");
	printf("- '4' omnivision 9740 \n\r");
	for(;;) {
		key = DBG_GetChar();
		if ((key >= '0') && (key <='5')) {
			if (sensor_setup(&twid, sensorsProfile[key- '0'], VGA) != SENSOR_OK) {
				printf("-E- Sensor setup failed.");
				while (1);
			} else {
				break;
			}
		}
	}
	/* Retrieve sensor output format and size */
	sensor_get_output(&wImageFormat, &wImageWidth, &wImageHeight, VGA);
	printf("Sensor <%d, %d> \n\r", (int)wImageWidth, (int)wImageHeight );
	if (wImageFormat == MONO_12_BIT) {
		printf("-I- Monochrome sensor do not support in this example!");
		while (1);
	}
	if (wImageFormat == RAW_BAYER_12_BIT || wImageFormat == RAW_BAYER_10_BIT)
		wImageFormat = (sensorOutputFormat_t)RGB_INPUT;
	else
		wImageFormat = (sensorOutputFormat_t)YUV_INPUT;

	/* ISI Initialize */
	_isiInit();

	printf("preview in RGB 565 mode\n\r");
	ISI_Enable();
	ISI_DisableInterrupt(0xFFFFFFFF);
	ISI_EnableInterrupt(ISI_IER_PXFR_DONE);
	ISI_DmaChannelDisable(ISI_DMA_CHDR_C_CH_DIS);
    SCB_CleanDCache_by_Addr((uint32_t *)preBufDescList, sizeof(preBufDescList));
	ISI_DmaChannelEnable(ISI_DMA_CHER_P_CH_EN);
	while (1);
}
