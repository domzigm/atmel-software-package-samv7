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
 *  \page lcd_ebi LCD example in SMC(EBI) mode
 *
 *  \section Purpose
 *
 *  This example demonstrates how to configure the LCD with EBI interface
 *
 *  \section Requirements
 *
 *  This package can be used with SAM V71 Xplained Ultra board with maXtouch
 *  xplained LCD board. LCD board must be set to 16-bit MCU configuration with
 *  the help of switch behind LCD. It should be in IM0 and IM2 should be in Off
 *  position and IM1 should be in On.
 *
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
 *        Definition
 *----------------------------------------------------------------------------*/
//#define USE_SDRAM
#define COLOR_CONVERT       RGB_24_TO_RGB565
#define LCD_MODE            ILI9488_EBIMODE

/*----------------------------------------------------------------------------
 *        Local variables
 *----------------------------------------------------------------------------*/
/** Global DMA driver for all transfer */
static sXdmad lcdEbiDma;

/** Image buffer (16-bits color). */
const uint32_t gImageBuffer[DEMO_IMAGE_HEIGHT * DEMO_IMAGE_WIDTH] = DEMO_IMAGE;
rect canvas_region[4];
static rect lcd_rc;

#if defined USE_SDRAM
#pragma location = "sdram_region"
#endif
COMPILER_ALIGNED(32) static uint16_t
			cavas_region_buf[BOARD_LCD_WIDTH * BOARD_LCD_HEIGHT / 4];
static TimeEvent RegionTimeEvent[4];

static uint8_t sin_xy[236] =
{
	78, 80, 82, 84, 86, 88, 90, 92, 94, 96, 98, 100, 102, 104, 106, 107,
	109, 111, 113, 115, 116, 118, 120, 121, 123, 125, 126, 128, 129, 130, 132, 133,
	135, 136, 137, 138, 139, 140, 141, 142, 143, 144, 145, 146, 147, 147, 148, 149,
	149, 150, 150, 151, 151, 151, 152, 152, 152, 152, 152, 152, 152, 152, 152, 151,
	151, 151, 150, 150, 149, 149, 148, 147, 147, 146, 145, 144, 143, 142, 141, 140,
	139, 138, 137, 136, 135, 133, 132, 130, 129, 128, 126, 125, 123, 121, 120, 118,
	116, 115, 113, 111, 109, 107, 106, 104, 102, 100, 98 , 96 , 94 , 92 , 90 , 88 ,
	86 , 84 , 82 , 80 , 78 , 76 , 74 , 72 , 70 , 68 , 66 , 64 , 62 , 60 , 58 , 56 ,
	54 , 52 , 50 , 48 , 46 , 45 , 43 , 41 , 39 , 37 , 36 , 34 , 32 , 31 , 29 , 27 ,
	26 , 24 , 23 , 22 , 20 , 19 , 17 , 16 , 15 , 14 , 13 , 12 , 11 , 10 , 9  , 8  ,
	7  , 6  , 5  , 5  , 4  , 3  , 3  , 2  , 2  , 1  , 1  , 1  , 0  , 0  , 0  , 0  ,
	0  , 0  , 0  , 0  , 0  , 1  , 1  , 1  , 2  , 2  , 3  , 3  , 4  , 5  , 5  , 6  ,
	7  , 8  , 9  , 10 , 11 , 12 , 13 , 14 , 15 , 16 , 17 , 19 , 20 , 22 , 23 , 24 ,
	26 , 27 , 29 , 31 , 32 , 34 , 36 , 37 , 39 , 41 , 43 , 45 , 46 , 48 , 50 , 52 ,
	54 , 56 , 58 , 60 , 62 , 64 , 66 , 68 , 70 , 72 , 74 , 76
};

uint32_t gColorArray[] =
{
	0x000000, 0xFFFFFF, 0x0000FF, 0xFF0000,
	0x000080, 0x00008B, 0x006400, 0x008B8B,
	0x00FFFF, 0x40E0D0, 0x4B0082, 0x800000,
	0x808000, 0x808080, 0x87CEEB, 0x8A2BE2,
	0x90EE90, 0x9400D3, 0x9ACD32, 0xA52A2A,
	0xA9A9A9, 0xA0522D, 0xADD8E6, 0xADFF2F,
	0xC0C0C0, 0xD3D3D3, 0xE0FFFF, 0xEE82EE,
	0xF0FFFF, 0xF5F5DC, 0xFF00FF, 0xFF6347,
};


/**
 * ISR for XDMA interrupt
 */
void XDMAC_Handler(void)
{
	XDMAD_Handler(&lcdEbiDma);
}

/**
 * canvas region initialization.
 */
static void init_canvas_region(void)
{
	uint32_t w, h;

	w = 480;
	h = 320;

	canvas_region[0].x = 0;
	canvas_region[0].y = 0;
	canvas_region[0].width = w / 2 - 1;
	canvas_region[0].height = h / 2 - 1;

	canvas_region[1].x = w / 2;
	canvas_region[1].y = 0;
	canvas_region[1].width = w / 2 - 1;
	canvas_region[1].height = h / 2 - 1;

	canvas_region[2].x = 0;
	canvas_region[2].y = h / 2;
	canvas_region[2].width = w / 2 - 1;
	canvas_region[2].height = h / 2 - 1;

	canvas_region[3].x = w / 2;
	canvas_region[3].y = h / 2;
	canvas_region[3].width = w / 2 - 1;
	canvas_region[3].height = h / 2 - 1;

	lcd_rc.x = 0;
	lcd_rc.y = 0;
	lcd_rc.width = w - 1;
	lcd_rc.height = h - 1;

}

/**
 * Draw coordinate axis with with giving buffer.
 */
static void draw_coordinate_axis(uint16_t* pCanvasBuffer,
								uint32_t x,
								uint32_t y,
								uint32_t ex,
								uint32_t ey,
								uint32_t pixel_width,
								uint32_t color)
{
	LCDD_DrawLine(pCanvasBuffer, x, y, ex, y + pixel_width - 1, color);
	LCDD_DrawLine(pCanvasBuffer, x, y, x + pixel_width - 1 , ey, color);
	LCDD_DrawLine(pCanvasBuffer, ex - 12, y - 4 ,ex, y ,  color);
	LCDD_DrawLine(pCanvasBuffer, ex - 12, y + 4 ,ex, y ,  color);
	LCDD_DrawLine(pCanvasBuffer, x - 4,  ey + 12 ,x, ey , color);
	LCDD_DrawLine(pCanvasBuffer, x + 4,  ey + 12 ,x, ey , color);
}

/**
 * Draw sin wave with with giving buffer.
 */
static void draw_sin_wave(uint16_t* pCanvasBuffer,
						uint32_t offset_x,
						uint32_t offset_y,
						uint32_t color)
{
	uint32_t i = 0;

	for (i = 0; i < sizeof(sin_xy); i++) {
		LCDD_DrawPixel(pCanvasBuffer,i + offset_x, offset_y - sin_xy[i], color);
	}
}

/**
 * Draw histogram.
 */
static void draw_random_histogram(uint32_t height, uint32_t color)
{
	uint8_t rand_val[32];
	uint32_t i;
	uint32_t offset_x = 24;
	uint32_t offset_y = height - 4;
	uint32_t w = 6;
	uint32_t * p_val = (uint32_t *)rand_val;

	for (i = 0; i < 8; i++) {
		TRNG->TRNG_CR = TRNG_CR_KEY_PASSWD | TRNG_CR_ENABLE;
		while (!(TRNG->TRNG_ISR & TRNG_ISR_DATRDY));
		p_val[i] = TRNG->TRNG_ODATA;
	}
	for (i = 0; i < 32; i++) {
		rand_val[i] = offset_y - rand_val[i] * offset_y / 255;
	}
	for (i = 0; i < 32; i++) {
		LCDD_DrawRectangleWithFill(cavas_region_buf,
								offset_x+i*w,
								rand_val[i],
								6,
								offset_y-rand_val[i],
								color);
	}
}

/**
 * Update canvas region 1.
 */
static void update_region1(uint32_t pos)
{
	LCDD_SetUpdateWindowSize(canvas_region[0]);
	LCDD_DrawRectangleWithFill(cavas_region_buf,canvas_region[0].x,
							canvas_region[0].y,
							canvas_region[0].width,canvas_region[0].height,
							COLOR_CONVERT(COLOR_BLUE));
	draw_coordinate_axis(cavas_region_buf,
						4,
						canvas_region[0].height - 3,
						canvas_region[0].width,
						0,
						1,
						COLOR_CONVERT(COLOR_WHITE));

	draw_sin_wave(cavas_region_buf,
				4,
				155,
				COLOR_CONVERT(COLOR_WHITE));

	LCDD_DrawRectangleWithFill(cavas_region_buf,
							4 + pos - 2,
							155 - sin_xy[pos] - 2,
							3,
							3,
							COLOR_CONVERT(COLOR_YELLOW));
	LCDD_UpdatePartialWindow((uint8_t*)cavas_region_buf,sizeof(cavas_region_buf));
}

/**
 * Update canvas region 2.
 */
static void update_region2(uint32_t x,uint32_t y,uint32_t color)
{
	LCDD_SetUpdateWindowSize(canvas_region[1]);
	LCDD_DrawRectangleWithFill(cavas_region_buf,
							0,
							0,
							canvas_region[1].width,
							canvas_region[1].height,
							COLOR_CONVERT(COLOR_GREEN));
	LCD_DrawString(cavas_region_buf,x, y, (const uint8_t *)"SAMV71",  COLOR_CONVERT(color));
	LCDD_UpdatePartialWindow((uint8_t*)cavas_region_buf,sizeof(cavas_region_buf));
}

/**
 * Update canvas region 3.
 */
static void update_region3(uint32_t alpha)
{
	uint32_t i,j;

	LCDD_SetUpdateWindowSize(canvas_region[2]);
	LCDD_DrawRectangleWithFill(cavas_region_buf,
							0,
							0,
							canvas_region[2].width,
							canvas_region[2].height,
							COLOR_CONVERT(COLOR_MAGENTA));

	i = ((canvas_region[2].width + 1) - DEMO_IMAGE_WIDTH) / 2;
	j = ((canvas_region[2].height + 1) - DEMO_IMAGE_HEIGHT) / 2;
	LCDD_BitBltAlphaBlend(cavas_region_buf,
						i,
						j,
						canvas_region[2].width + 1,
						canvas_region[2].height + 1,
						(LcdColor_t *)gImageBuffer,
						0,
						0,
						DEMO_IMAGE_WIDTH,DEMO_IMAGE_HEIGHT,alpha);
	LCDD_UpdatePartialWindow((uint8_t*)cavas_region_buf,sizeof(cavas_region_buf));
}

/**
 * Update canvas region 4.
 */
static void update_region4(void)
{
	LCDD_SetUpdateWindowSize(canvas_region[3]);
	LCDD_DrawRectangleWithFill(cavas_region_buf,
							0,
							0,
							canvas_region[3].width,canvas_region[3].height,
							COLOR_CONVERT(COLOR_WHITE));

	draw_random_histogram( canvas_region[3].height + 1, COLOR_CONVERT(COLOR_RED));

	LCDD_UpdatePartialWindow((uint8_t*)cavas_region_buf,sizeof(cavas_region_buf));
}

/**
 *  \brief LCD Application entry point.
 *
 *  \return Unused (ANSI-C compatibility).
 */
extern int main( void )
{
	uint32_t i, j;

	int32_t dX, dY;
	uint32_t region1_pos = 0;
	uint32_t region2_x,region2_y;
	TimeEvent *pEvent;
	uint32_t alpha = 0;
	int8_t alpha_direction = 4;

	/* Disable watchdog */
	WDT_Disable(WDT);

	SCB_EnableICache();
	SCB_EnableDCache();

	/* Output example information */
	printf("-- LCD EBI Example %s --\n\r", SOFTPACK_VERSION);
	printf("-- %s\n\r", BOARD_NAME);
	printf("-- Compiled: %s %s With %s--\n\r", __DATE__, __TIME__, COMPILER_NAME);

#if defined USE_SDRAM
    BOARD_ConfigureSdram();
#endif

	PMC_EnablePeripheral(ID_TRNG);

	/* Configure systick for 1 ms. */
	TimeTick_Configure();

	RegionTimeEvent[0].event = 1;
	RegionTimeEvent[0].time_tick = 1;
	RegionTimeEvent[0].time_start = 0;
	RegionTimeEvent[0].pPreEvent = 0;
	RegionTimeEvent[0].pNextEvent = &RegionTimeEvent[1];

	RegionTimeEvent[1].event = 2;
	RegionTimeEvent[1].time_tick = 500;
	RegionTimeEvent[1].time_start = 0;
	RegionTimeEvent[1].pPreEvent = &RegionTimeEvent[0];
	RegionTimeEvent[1].pNextEvent = &RegionTimeEvent[2];

	RegionTimeEvent[2].event = 3;
	RegionTimeEvent[2].time_tick = 20;
	RegionTimeEvent[2].time_start = 0;
	RegionTimeEvent[2].pPreEvent = &RegionTimeEvent[2];
	RegionTimeEvent[2].pNextEvent = &RegionTimeEvent[3];

	RegionTimeEvent[3].event = 4;
	RegionTimeEvent[3].time_tick = 300;
	RegionTimeEvent[3].time_start = 0;
	RegionTimeEvent[3].pPreEvent = &RegionTimeEvent[2];
	RegionTimeEvent[3].pNextEvent = 0;

	SetTimeEvent(RegionTimeEvent);

	pEvent = RegionTimeEvent;

	/* Initialize LCD and its interface */
	LCDD_Initialize(LCD_MODE, &lcdEbiDma, 1);

	init_canvas_region();
	LCDD_SetUpdateWindowSize(lcd_rc);

	LCDD_SetUpdateWindowSize(canvas_region[0]);
	LCDD_DrawRectangleWithFill(cavas_region_buf,
							canvas_region[0].x,
							canvas_region[0].y,
							canvas_region[0].width,canvas_region[0].height,
							COLOR_CONVERT(COLOR_BLUE));

	draw_coordinate_axis(cavas_region_buf,
						4,
						canvas_region[0].height - 3,
						canvas_region[0].width,
						0,
						1,
						COLOR_CONVERT(COLOR_WHITE));

	draw_sin_wave(cavas_region_buf,
				4,
				155,
				COLOR_CONVERT(COLOR_WHITE));

	LCDD_UpdatePartialWindow((uint8_t *)cavas_region_buf, sizeof(cavas_region_buf));

	LCDD_SetUpdateWindowSize(canvas_region[1]);
	LCDD_DrawRectangleWithFill(cavas_region_buf,
							0,
							0,
							canvas_region[1].width,
							canvas_region[1].height,
							COLOR_CONVERT(COLOR_GREEN));
	LCD_DrawString(cavas_region_buf,10, 5, (const uint8_t *)"SAMV71",  COLOR_CONVERT(COLOR_BLACK));
	LCDD_UpdatePartialWindow((uint8_t *)cavas_region_buf, sizeof(cavas_region_buf));

	LCDD_SetUpdateWindowSize(canvas_region[2]);
	LCDD_DrawRectangleWithFill(cavas_region_buf,
							0,
							0,
							canvas_region[2].width,
							canvas_region[2].height,
							COLOR_CONVERT(COLOR_MAGENTA));
	i = ((canvas_region[2].width + 1) - DEMO_IMAGE_WIDTH) / 2;
	j = ((canvas_region[2].height + 1) - DEMO_IMAGE_HEIGHT) / 2;
	LCDD_BitBlt(cavas_region_buf, i, j,
				DEMO_IMAGE_WIDTH, DEMO_IMAGE_HEIGHT,
				(LcdColor_t *)gImageBuffer, 0, 0,
				DEMO_IMAGE_WIDTH, DEMO_IMAGE_HEIGHT);
	LCDD_UpdatePartialWindow((uint8_t*)cavas_region_buf, sizeof(cavas_region_buf));

	LCDD_SetUpdateWindowSize(canvas_region[3]);
	LCDD_DrawRectangleWithFill(cavas_region_buf,
							0,
							0,
							canvas_region[3].width,
							canvas_region[3].height,
							COLOR_CONVERT(COLOR_WHITE));

	draw_random_histogram(canvas_region[3].height + 1,
						COLOR_CONVERT(COLOR_RED));

	LCDD_UpdatePartialWindow((uint8_t*)cavas_region_buf, sizeof(cavas_region_buf));
	region2_x = 0;
	region2_y = 0;
	dX = 2;
	dY = 2;
	RegionTimeEvent[0].time_start = 1;
	RegionTimeEvent[1].time_start = 1;
	RegionTimeEvent[2].time_start = 1;
	RegionTimeEvent[3].time_start = 1;
	while (1) {
		while (pEvent) {
			if (pEvent->occur) {
				pEvent->occur = 0;
				switch (pEvent->event) {
				case 1:
					update_region1(region1_pos);
					pEvent->time_tick = 1;
					pEvent->time_start = 1;
					if (region1_pos == 236)
						region1_pos = 0;
					else
						region1_pos++;
					pEvent = pEvent->pNextEvent;
					break;

				case 2:
					TRNG->TRNG_CR = TRNG_CR_KEY_PASSWD | TRNG_CR_ENABLE;
					while (!(TRNG->TRNG_ISR & TRNG_ISR_DATRDY));
					i = TRNG->TRNG_ODATA;
					j = (i >> 16) & 0x1F;
					region2_x =i & 0xFF;
					region2_y =(i >> 8)&0xFF;
					region2_x = region2_x * (canvas_region[1].width - 80) / 0xFF;
					region2_y = region2_y * (canvas_region[1].height - 20) / 0xFF;
					update_region2(region2_x,region2_y,COLOR_CONVERT(gColorArray[j]));
					if ((region2_x >= (canvas_region[1].width - 80))
							&& (region2_y < (canvas_region[1].height - 20))) {
						region2_x -=dX;
						dX=-dX;
					} else if ((region2_x < (canvas_region[1].width - 80))
							&& (region2_y >= (canvas_region[1].height - 20))) {
						region2_y -=dY;
						dY=-dY;
					}
					pEvent->time_tick = 500;
					pEvent->time_start = 1;
					pEvent = pEvent->pNextEvent;
					break;

				case 3:
					update_region3(alpha);
					alpha += alpha_direction;
					if (alpha >= 255)
						alpha_direction = -4;
					else if (alpha <= 0)
						alpha_direction = 4;
					pEvent->time_tick = 20;
					pEvent->time_start = 1;
					pEvent = pEvent->pNextEvent;
					break;

				case 4:
					update_region4();
					pEvent->time_tick = 300;
					pEvent->time_start = 1;
					break;
				}
			} else {
				pEvent = RegionTimeEvent;
			}
		}
	}
}

