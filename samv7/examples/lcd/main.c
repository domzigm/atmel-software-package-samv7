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
 *  \page lcd LCD example
 *
 *  \section Purpose
 *
 *  This example demonstrates how to configure the LCD with SPI interface
 *
 *  \section Requirements
 *
 *  This package can be used with SAM V71 Xplained Ultra board with maXtouch
 *  xplained LCD board. LCD board must be set to 4-wire SPI configuration with
 *  the help of switch behind LCD. It should be in IM0, IM1 and IM2 should be in
 *  On position. Connect the LCD pad to EXT2 connectors.
 *  \section Description
 *
 *  \section Usage
 *
 *  -# Build the program and download it inside the SAM V71 Xplained Ultra board.
 * Please refer to the Getting Started with SAM V71 Microcontrollers.pdf
 *  -# On the computer, open and configure a terminal application
 *     (e.g. HyperTerminal on Microsoft Windows) with these settings:
 *    - 115200 baud rate
 *    - 8 bits of data
 *    - No parity
 *    - 1 stop bit
 *    - No flow control
 *  -# Start the application.
 *  -# In the terminal window, the
 *     following text should appear (values depend on the board and chip used):
 *     \code
 *      -- LCD Example xxx --
 *      -- xxxxxx-xx
 *      -- Compiled: xxx xx xxxx xx:xx:xx --
 *     \endcode
 *
 *  \section References
 *  - lcd/main.c
 *  - trace.h
 */

/** \file
 *
 *  This file contains all the specific code for the xplained LCD example.
 *
 */

/*----------------------------------------------------------------------------
 *        Headers
 *----------------------------------------------------------------------------*/

#include "board.h"
#include "image.h"
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/*----------------------------------------------------------------------------
 *        Local Definition
 *----------------------------------------------------------------------------*/
 /* #define USE_SDRAM */
#define COLOR_CONVERT       RGB_24_TO_18BIT

#define HEADLINE_OFFSET     25

#define LCD_MODE           ILI9488_SPIMODE

#if defined USE_SDRAM
#define CANVAS_LCD_WIDTH    BOARD_LCD_WIDTH
#define CANVAS_LCD_HEIGHT   BOARD_LCD_HEIGHT

#else
#define CANVAS_LCD_WIDTH    240
#define CANVAS_LCD_HEIGHT   360

#endif
/*----------------------------------------------------------------------------
 *        Local variables
 *----------------------------------------------------------------------------*/
/** Global DMA driver for all transfer */
static sXdmad lcdSpiDma;

/** Image buffer (16-bits color). */
const uint32_t gImageBuffer[DEMO_IMAGE_HEIGHT * DEMO_IMAGE_WIDTH] = DEMO_IMAGE;
#if defined USE_SDRAM
COMPILER_SECTION("sdram_region")
#endif
static sBGR gLcdCavas[CANVAS_LCD_WIDTH * CANVAS_LCD_HEIGHT];


/**
 * ISR for XDMA interrupt
 */
void XDMAC_Handler(void)
{
	XDMAD_Handler(&lcdSpiDma);
}

/**
 *  \brief LCD Application entry point.
 *
 *  \return Unused (ANSI-C compatibility).
 */
extern int main(void)
{
	uint32_t i, j;
	int32_t dX, dY;
	rect rc;
	const uint8_t String[] = "LCD Example";

	/* Disable watchdog */
	WDT_Disable(WDT);

	SCB_EnableICache();
	SCB_EnableDCache();

	/* Output example information */
	printf("-- LCD Example %s --\n\r", SOFTPACK_VERSION);
	printf("-- %s\n\r", BOARD_NAME);
	printf("-- Compiled: %s %s With %s--\n\r", __DATE__, __TIME__, COMPILER_NAME);

#if defined USE_SDRAM
	BOARD_ConfigureSdram();
#endif

	/* Configure systick for 1 ms. */
	TimeTick_Configure();

	/* Initialize LCD and its interface */
	LCDD_Initialize(LCD_MODE, &lcdSpiDma, 0);
	LCDD_SetCavasBuffer(gLcdCavas, sizeof(gLcdCavas));

	rc.x = 0;
	rc.y = 0;
	rc.width = CANVAS_LCD_WIDTH - 1;
	rc.height = CANVAS_LCD_HEIGHT - 1;
	LCDD_SetUpdateWindowSize(rc);

	LCDD_DrawRectangleWithFill(0, 0, 0, CANVAS_LCD_WIDTH - 1, CANVAS_LCD_HEIGHT - 1,
			COLOR_CONVERT(COLOR_WHITE));
	LCDD_UpdateWindow();
	Wait(1000);

	LCDD_DrawRectangleWithFill(0, 0, 0, CANVAS_LCD_WIDTH - 1, CANVAS_LCD_HEIGHT - 1,
			COLOR_CONVERT(COLOR_BLUE));
	LCDD_UpdateWindow();
	Wait(1000);

	LCD_DrawString(0, 50, 5, String,  RGB_24_TO_18BIT(COLOR_BLACK));
	LCDD_UpdateWindow();
	Wait(500);

	/* Test basic color space translation and LCD_DrawFilledRectangle */
	LCDD_DrawRectangleWithFill(0,
							0,
							HEADLINE_OFFSET,
							CANVAS_LCD_WIDTH - 1,
							CANVAS_LCD_HEIGHT - 1 - HEADLINE_OFFSET,
							COLOR_CONVERT(COLOR_WHITE));
	LCDD_UpdateWindow();
	Wait(500);

	LCDD_DrawRectangleWithFill(0,
							4,
							4 + HEADLINE_OFFSET,
							CANVAS_LCD_WIDTH - 5 - 4,
							CANVAS_LCD_HEIGHT - 5 - HEADLINE_OFFSET,
							COLOR_CONVERT(COLOR_BLACK));
	LCDD_UpdateWindow();
	Wait(500);

	LCDD_DrawRectangleWithFill(0,
							8,
							8 + HEADLINE_OFFSET,
							CANVAS_LCD_WIDTH - 9 - 8,
							CANVAS_LCD_HEIGHT- 9 - 8 - HEADLINE_OFFSET,
							COLOR_CONVERT(COLOR_BLUE));
	LCDD_UpdateWindow();
	Wait(500);

	LCDD_DrawRectangleWithFill(0,
							12,
							12 + HEADLINE_OFFSET,
							CANVAS_LCD_WIDTH - 13 - 12,
							CANVAS_LCD_HEIGHT - 13 - 12 - HEADLINE_OFFSET,
							COLOR_CONVERT(COLOR_RED));
	LCDD_UpdateWindow();
	Wait(500);

	LCDD_DrawRectangleWithFill(0,
							16,
							14 + HEADLINE_OFFSET,
							CANVAS_LCD_WIDTH - 17 - 16,
							CANVAS_LCD_HEIGHT - 17 - 14 - HEADLINE_OFFSET,
							COLOR_CONVERT(COLOR_GREEN));
	LCDD_UpdateWindow();
	Wait(500);
	/* Test horizontal/vertical LCD_drawLine  */
	LCDD_DrawLine(0,
				0,
				CANVAS_LCD_HEIGHT / 2,
				CANVAS_LCD_WIDTH - 1,
				CANVAS_LCD_HEIGHT / 2,
			COLOR_CONVERT(COLOR_RED));
	LCDD_UpdateWindow();
	Wait(500);

	LCDD_DrawLine(0,
				CANVAS_LCD_WIDTH / 2,
				HEADLINE_OFFSET ,
				CANVAS_LCD_WIDTH / 2,
				CANVAS_LCD_HEIGHT - 1, COLOR_CONVERT(COLOR_RED));
	LCDD_UpdateWindow();
	Wait(500);

	/* Test LCD_drawLine  */
	LCDD_DrawLine(0,
				0,
				0 ,
				CANVAS_LCD_WIDTH -1,
				CANVAS_LCD_HEIGHT - 1,
			RGB_24_TO_RGB565(COLOR_RED));
	LCDD_UpdateWindow();
	Wait(500);

	LCDD_DrawLine(0,
				0,
				CANVAS_LCD_HEIGHT - 1,
				CANVAS_LCD_WIDTH - 1, 0,
				RGB_24_TO_RGB565(COLOR_RED));
	LCDD_UpdateWindow();
	Wait(500);

	/* Test LCD_DrawRectangle */
	LCDD_DrawRectangle(0,
					CANVAS_LCD_WIDTH / 4,
					CANVAS_LCD_HEIGHT / 4,
					CANVAS_LCD_WIDTH * 3 / 4 - CANVAS_LCD_WIDTH / 4,
					CANVAS_LCD_HEIGHT * 3 / 4 - CANVAS_LCD_HEIGHT / 4,
					COLOR_CONVERT(COLOR_RED));
	LCDD_UpdateWindow();
	Wait(500);

	LCDD_DrawRectangle(0,
					CANVAS_LCD_WIDTH / 3,
					CANVAS_LCD_HEIGHT / 3,
					CANVAS_LCD_WIDTH * 2 / 3 - CANVAS_LCD_WIDTH / 3,
					CANVAS_LCD_HEIGHT * 2 / 3 - CANVAS_LCD_HEIGHT / 3,
					COLOR_CONVERT(COLOR_RED));
	LCDD_UpdateWindow();
	Wait(500);

	/* Test LCD_DrawFilledCircle */
	LCD_DrawFilledCircle(0,
						CANVAS_LCD_WIDTH * 3 / 4,
						CANVAS_LCD_HEIGHT * 3 / 4,
						CANVAS_LCD_WIDTH / 4,
						COLOR_CONVERT(COLOR_BLUE));
	LCDD_UpdateWindow();
	Wait(500);

	LCD_DrawFilledCircle(0,
						CANVAS_LCD_WIDTH / 2,
						CANVAS_LCD_HEIGHT / 2,
						CANVAS_LCD_HEIGHT / 4,
						COLOR_CONVERT(COLOR_WHITE));
	LCDD_UpdateWindow();
	Wait(500);

	LCD_DrawFilledCircle(0,
						CANVAS_LCD_WIDTH / 4,
						CANVAS_LCD_HEIGHT * 3 / 4,
						CANVAS_LCD_HEIGHT / 4,
						COLOR_CONVERT(COLOR_RED));
	LCDD_UpdateWindow();
	Wait(500);

	LCD_DrawFilledCircle(0,
						CANVAS_LCD_WIDTH * 3 / 4,
						CANVAS_LCD_HEIGHT / 4,
						CANVAS_LCD_WIDTH / 4,
						COLOR_CONVERT(COLOR_YELLOW));
	LCDD_UpdateWindow();
	Wait(500);
	/* Test LCD_DrawPicture */
	LCDD_DrawImage(0, 50, 50, (LcdColor_t *)gImageBuffer ,  (50 + DEMO_IMAGE_WIDTH),
			(50 + DEMO_IMAGE_HEIGHT));
	LCDD_UpdateWindow();
	Wait(3000);

	LCDD_DrawRectangleWithFill(0,
							0,
							0,
							CANVAS_LCD_WIDTH - 1,
							CANVAS_LCD_HEIGHT - 1,
							COLOR_CONVERT(COLOR_BLACK));

	/** Move picture across the screen */
	dX = 2;
	dY = 2;
	j = 0;
	i = 0;

	for (; ;) {
		for (; i < (CANVAS_LCD_WIDTH-DEMO_IMAGE_WIDTH - 1);) {
			LCDD_DrawRectangleWithFill(0,
									0,
									0,
									CANVAS_LCD_WIDTH - 1,
									CANVAS_LCD_HEIGHT - 1,
									COLOR_CONVERT(COLOR_BLACK));
			LCDD_DrawImage(0, i, j, (LcdColor_t *)gImageBuffer, (i + DEMO_IMAGE_WIDTH),
					(j + DEMO_IMAGE_HEIGHT));
			LCDD_UpdateWindow();

			j+=dY;
			i+=dX;

			if ((i >= (CANVAS_LCD_WIDTH - DEMO_IMAGE_WIDTH - 1 )) &&
					(j < (CANVAS_LCD_HEIGHT - DEMO_IMAGE_HEIGHT - 1 ))) {
				i-=dX;
				dX=-dX;
			}
			else if ((i < (CANVAS_LCD_WIDTH-DEMO_IMAGE_WIDTH - 1)) &&
					(j >= (CANVAS_LCD_HEIGHT-DEMO_IMAGE_HEIGHT - 1))) {
				j-=dY;
				dY=-dY;
			}
		}
		i = 0;
		j = 0;
	}
}
