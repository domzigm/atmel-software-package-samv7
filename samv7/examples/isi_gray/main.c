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
 * \page isi_gray ISI Gray Scale Example
 *
 * \section Purpose
 *
 * This example demonstrates the ISI (Image Sensor Interface) of an SAM V71
 * Xplained Ultra.
 *
 * \section Requirements
 *
 * This package can be used with SAM V71 Xplained Ultra with
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
 *     -- SAMxxxxx-xx
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
static uint8_t gIsiBuffer[640 * 480];
COMPILER_SECTION("sdram_region")
static uint16_t gDisplayBuffer[320 * 240];
static uint16_t lut[256];

#define ISI_BASE                gIsiBuffer
#define DISPLAY_BASE            gDisplayBuffer

/** TWI clock frequency in Hz. */
#define TWCK                    400000
/** TWI peripheral ID for Sensor configuration */
#define BOARD_ID_TWI_ISI        ID_TWIHS0
/** TWI base address for Sensor configuration */
#define BOARD_BASE_TWI_ISI      TWIHS0
#define CAMX_MT9V022_SLAVE_ADDR  (0x30>>1)

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
extern const sensorProfile_t mt9v022Profile;

/** ISI pins to configure. */
const Pin pinsTWI[] = BOARD_PINS_TWI_ISI;
const Pin pin_ISI_RST = BOARD_ISI_RST;
const Pin pin_ISI_PWD = BOARD_ISI_PWD;
const Pin pPinsISI[]= {BOARD_ISI_PINS};

/** TWI driver instance.*/
static Twid twid;

/* ISI DMA descriptor for preview path */
ISI_FrameBufferDescriptors preBufDescList;

/* Image size in preview mode */
static uint32_t wImageWidth, wImageHeight;

/* Image output format */
static sensorOutputFormat_t wImageFormat;

/* LCD display mode Normal or reverse mode */
static uint8_t lcdDisplayMode = LCD_REVERSE_MODE;

/** Global DMA driver for all transfer */
static sXdmad lcdEbiDma;

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

static void y2rgb565(void)
{
	uint16_t i, j;
	uint8_t *s;
	uint16_t *p;

	s = (uint8_t*)ISI_BASE;
	p = (uint16_t*)DISPLAY_BASE;

	for (i = 0; i < wImageWidth; i++) {
		for (j = 0; j < wImageHeight; j++) {
			*p++ = lut[*s];
			s++;
		}
	}
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
		if (ISI->ISI_DMA_P_ADDR != (uint32_t)ISI_BASE + (wImageWidth * wImageHeight ))
			return;
		ISI_DisableInterrupt(ISI_IDR_PXFR_DONE);
		ISI_DmaChannelDisable(ISI_DMA_CHDR_P_CH_DIS);
		y2rgb565();
		LCDD_UpdateWindow();
		ISI_setDmaInPreviewPath((uint32_t)&preBufDescList,
							ISI_DMA_P_CTRL_P_FETCH, (uint32_t)ISI_BASE);
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
	PMC->PMC_PCK[0] = PMC_PCK_CSS_PLLA_CLK | (9 << 4);
	/* Enable programmable clock 0 output */
	REG_PMC_SCER = PMC_SCER_PCK0;
	/* Wait for the PCKRDY0 bit to be set in the PMC_SR register*/
	while ((REG_PMC_SR & PMC_SR_PCKRDY0) == 0);
	/* ISI PWD OFF*/
	PIO_Clear(&pin_ISI_PWD);
	PIO_Clear(&pin_ISI_RST);
}


/**
 * \brief Configure LCD
 */
static void  _lcd_Configure(void)
{
	rect rc;

	LCDD_Initialize(LCD_MODE, &lcdEbiDma, lcdDisplayMode);
	LCDD_SetCavasBuffer((void*)gDisplayBuffer, wImageWidth * wImageHeight * 2);
	rc.x = 0;
	rc.y = 0;
	rc.width = wImageWidth - 1;
	rc.height = wImageHeight - 1;
	LCDD_SetUpdateWindowSize(rc);
	LCDD_DrawRectangleWithFill(0, 0, 0, rc.width -1, rc.height -1,
			RGB_24_TO_RGB565(COLOR_BLACK));
	LCDD_UpdateWindow();
	Wait(500);
}

/**
 * \brief ISI  initialization.
 */
static void _isiInit(void)
{
	/* Enable ISI peripheral clock */
	PMC_EnablePeripheral(ID_ISI);
	/* Reset ISI peripheral */
	ISI_Reset();
	/* Set the windows blank */
	ISI_SetBlank(0, 0);
	/* Set vertical and horizontal Size of the Image Sensor for preview path*/
	ISI_SetSensorSize(wImageWidth/2, wImageHeight);
	/* Set preview size to fit LCD size*/
	ISI_setPreviewSize(wImageWidth/2, wImageHeight);
	/* calculate scaler factor automatically. */
	ISI_calcScalerFactor();
	/*  Set data stream in YUV format.*/
	ISI_setInputStream(RGB_INPUT);
	ISI_RgbPixelMapping(1);
	/* Configure DMA for preview path. */
	preBufDescList.Current = (uint32_t)ISI_BASE;
	preBufDescList.Control = ISI_DMA_P_CTRL_P_FETCH;
	preBufDescList.Next    = (uint32_t)&preBufDescList;
	ISI_setDmaInPreviewPath((uint32_t)&preBufDescList, ISI_DMA_P_CTRL_P_FETCH,(uint32_t)ISI_BASE);
	NVIC_ClearPendingIRQ(ISI_IRQn);
	NVIC_SetPriority(ISI_IRQn, 5);
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
extern int main(void)
{
	uint8_t reg = 0x0;
	uint8_t val = 0xFD;
	uint16_t i;

	WDT_Disable(WDT);

	/* Enable I and D cache */
	SCB_EnableICache();
	SCB_EnableDCache();

	/* Output example information */
	printf("-- ISI Gray Example %s --\n\r", SOFTPACK_VERSION);
	printf("-- %s\n\r", BOARD_NAME);
	printf("-- Compiled: %s %s With %s--\n\r", __DATE__, __TIME__, COMPILER_NAME);

	/* Enable SDRAM for Frame buffer */
	BOARD_ConfigureSdram();

	/* Configure Time Tick */
	TimeTick_Configure();

	/* TWI Initialize */
	_twiInit();

	/* ISI PCK clock Initialize */
	_isiPckInit();

	PIO_Set(&pin_ISI_RST);

	TWID_Write(&twid, CAMX_MT9V022_SLAVE_ADDR, 0x03, 1, &reg, 1, 0);
	TWID_Write(&twid, CAMX_MT9V022_SLAVE_ADDR, 1, 1, &val, 1, 0);
	if (sensor_setup(&twid, &mt9v022Profile, QVGA) != SENSOR_OK) {
		printf("-E- Sensor setup failed.");
		while (1);
	}

	/* Retrieve sensor output format and size */
	sensor_get_output(&wImageFormat, &wImageWidth, &wImageHeight, QVGA);

	/* Configure LCD */
	_lcd_Configure();

	/* Init RBG565 lookup table */
	for(i = 0; i < 256; i++)
		lut[i] = ((i >> 3) << 11) | ((i >> 2) << 5) | (i >> 3);

	/* ISI Initialize */
	_isiInit();

	ISI_Enable();
	ISI_DisableInterrupt(0xFFFFFFFF);
	ISI_EnableInterrupt(ISI_IER_PXFR_DONE);
	ISI_DmaChannelDisable(ISI_DMA_CHDR_C_CH_DIS);
	SCB_CleanInvalidateDCache();

	ISI_DmaChannelEnable(ISI_DMA_CHER_P_CH_EN);

	printf("preview in RGB 565 mode\n\r");
	while (1);
}

