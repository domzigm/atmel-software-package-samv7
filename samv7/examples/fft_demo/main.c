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
 * \page fft_demo FFT Demo
 *
 * \section Purpose
 *
 * The example demonstrates complex FFT of 22.05K signals.
 * To achieve the best performance, ITCM and DTCM (128K bytes each) are used and
 * variables/code in file "fft.c" are located in TCM areas.
 *
 *
 * \section Requirements
 *
 * This package can be used with SAMV71 Xplained Ultra board or SAME70 Xplained board.
 * maXtouch xplained LCD module (controlled by "DEMO_WITH_LCD") could be used.
 * While using LCD module, the module must be set to 4-wire SPI configuration
 * with the help of switch behind LCD. It should be in IM0, IM1 and IM2 should
 * be in "ON" position. Connect the LCD pad to EXT2 connectors.
 *
 * Note: This demo can be executed from flash, execute from SRAM is not supported.
 *
 * \section Usage
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
 * -# Start the application.
 * -# In the terminal window, the
 *    following text should appear (values depend on the board and chip used):
 *    \code
 *     -- FFT Demo Example xxx --
 *     -- xxxxxx-xx
 *     -- Compiled: xxx xx xxxx xx:xx:xx --
 *    \endcode
 * -# The application will output FFT process information to hyperterminal and
 * display signal wave form, frequency domain magnitudes and other information
 * on the LCD(if configured).
 *
 * \section References
 * - fft_demo/main.c
 * - fft_demo/fft.c
 * - fft_demo/fft.h
 * - fft_demo/signal.c
 * - fft_demo/signal.h
 */

/** \file
 *
 *  This file contains all the specific code for the FFT_demo example.
 */

/*----------------------------------------------------------------------------
 *        Headers
 *----------------------------------------------------------------------------*/

#include "board.h"

#include <string.h>

#include "fft.h"
#include "signal.h"

extern float32_t wav_in_buffer[];
extern float32_t mag_in_buffer[];
extern uint32_t time_total_us;
extern int32_t time_fft_us;
extern uint32_t CycleCounter[];


/*----------------------------------------------------------------------------
 *        Local definitions
 *----------------------------------------------------------------------------*/

/** definition for demo with (1) or without (0) LCD module */
#define DEMO_WITH_LCD    0

#if DEMO_WITH_LCD
#if !defined(BOARD_LCD_SMC)
/** color converter, from RGB 24 bits to 18 bits */
#define COLOR_CONVERT       RGB_24_TO_18BIT
#else
/** color converter, from RGB 24 bits to 16 bits (5,6,5) */
#define COLOR_CONVERT       RGB_24_TO_RGB565
#endif
#endif

/** height of chart to display wave form and frequency domain magnitudes */
#define CHART_HEIGHT    170
/** width of chart to display wave form and frequency domain magnitudes */
#define CHART_WIDTH     (256+1)
/** size of the boarder */
#define CHART_BORDER    ((BOARD_LCD_WIDTH - CHART_WIDTH)/2)


/** demonstrate parameters */
typedef struct DEMO_PARA {
	EN_SIGINAL_TYPE signal_type;    /** signal type */
	uint32_t        fft_size;       /** size of FFT: 256/1024/4096 */
	EN_DATA_TYPE    data_type;      /** data type: Q15/Q31/F32 */
	uint32_t        fft_factor;     /** factors for average the magnitudes in
										frequency domain */
	uint32_t        frequency;      /** frequency of sine/square/triangle wave,
										meaningless for others */
	uint32_t        duty_ratio;     /** duty ratio of square/triangle wave,
										meaningless for others */
}DEMO_PARA;


/*----------------------------------------------------------------------------
 *        Local variables
 *----------------------------------------------------------------------------*/
/** pin for pause the demonstration */
const Pin pinPB1 = {PIO_PA9, PIOA, ID_PIOA, PIO_INPUT, PIO_PULLUP|PIO_IT_RISE_EDGE};
/** pin for pause the demonstration */
const Pin pinPB2 = {PIO_PB12, PIOB, ID_PIOB, PIO_INPUT, PIO_PULLUP|PIO_IT_RISE_EDGE};

#if DEMO_WITH_LCD
/** Magnitude buffer converted to uint32_t for drawing */
static uint32_t mag_in_buffer_int[MAX_FFT_SIZE];
/** backup buffer for draw wave form */
static float32_t wav_in_backup[MAX_FFT_SIZE * 2];
/** backup buffer for draw magnitudes in frequency domain */
static uint32_t mag_in_backup[MAX_FFT_SIZE];

const uint32_t wav_center_y = 60 + CHART_HEIGHT/2;
const uint32_t mag_botton_y = 60 + CHART_HEIGHT * 2 + 15;

/** string for display signal's type */
static const uint8_t str_signal_type[][10] = {
		{"Sine"},
		{"Audio"},
		{"Square"},
		{"TriAngle"},
		{"White"},
		{"Pink"}
};
#endif

/** signal definition */
static const char SIGNAL_TYPES[][20] = {
		{"Sine_wave"},
		{"Real_time_audio"},
		{"Square_wave"},
		{"Triangle_wave"},
		{"White_noise"},
		{"Pink_noise"},
};
/** string for display data type */
static const uint8_t str_data_type[][10] = {
		{"Q15"}, {"Q31"}, {"F32"}
};

/** default demonstration parameter */
static DEMO_PARA demo_para[] = {
		{EN_SIGINAL_SINUS, 4096, EN_DATA_TYPE_F32,   4096/256,  10,  50 },
};

/** flag to control the demonstration */
static uint32_t pause_flag = 0;

/** Pointer pointed to demonstration parameter structure */
static DEMO_PARA * pDemoPara = &demo_para[0];

#if DEMO_WITH_LCD
/** Global DMA driver for all transfer */
static sXdmad lcdSpiDma;
#endif

/*----------------------------------------------------------------------------
 *        Local Functions
 *----------------------------------------------------------------------------*/

#if DEMO_WITH_LCD
/**
 * ISR for XDMA interrupt
 */
void XDMAC_Handler(void)
{
	XDMAD_Handler(&lcdSpiDma);
}
#endif

/**
 *  \brief handler for button press event
 *
 *  param pPin    Pointer to an Pin instance.
 */
static void button_handler( const Pin* pPin )
{
	/*dummy*/
	pPin = pPin;

	pause_flag = (0==pause_flag) ? 1 : 0;
	pause_flag = 1;
}

/**
 *  \brief Configure the Push buttons
 *
 *  Configure the PIO as inputs and generate corresponding interrupt when
 *  pressed or released.
 */
static void configure_buttons( void )
{
	/* Configure PIO as inputs. */
	PIO_Configure( &pinPB1, 1 );
	PIO_Configure( &pinPB2, 1 );

	/* Adjust PIO denounce filter parameters, uses 10 Hz filter. */
	PIO_SetDebounceFilter( &pinPB1, 10 );
	PIO_SetDebounceFilter( &pinPB2, 10 );

	/* Initialize PIO interrupt handlers, see PIO definition in board.h. */
	PIO_ConfigureIt( &pinPB1, button_handler ); /* Interrupt on rising edge  */
	PIO_ConfigureIt( &pinPB2, button_handler ); /* Interrupt on rising edge */

	/* Enable PIO controller IRQs. */
	NVIC_EnableIRQ( (IRQn_Type)pinPB1.id );
	NVIC_EnableIRQ( (IRQn_Type)pinPB2.id );

	/* Enable PIO line interrupts. */
	PIO_EnableIt( &pinPB1 );
	PIO_EnableIt( &pinPB2 );
}

#if DEMO_WITH_LCD
/**
 *  \brief Initialize LCD and its interface
 */
static void lcd_init(void)
{
	rect rc;
	/* Enable SDRAM for Frame buffer */
	BOARD_ConfigureSdram();

	/* Initialize LCD and its interface */
	LCDD_Initialize(ILI9488_SPIMODE, &lcdSpiDma, 0);
	LCDD_SetCavasBuffer(NULL,0);

	rc.x = 0;
	rc.y = 0;
	rc.width = BOARD_LCD_WIDTH;
	rc.height = BOARD_LCD_HEIGHT;
	LCDD_SetUpdateWindowSize(rc);
}
#endif

/**
 *  \brief Initialize LED, buttons, timers and LCD
 */
static void initialize(void)
{
	LED_Configure(0);

	configure_buttons();

	PMC->PMC_PCER0 = (1<<ID_TC1);
	TC_Configure(TC0, 1, TC_CMR_TCCLKS_TIMER_CLOCK3);
	PMC_EnablePeripheral(ID_TRNG);
	TRNG_Enable();

	/* Configure systick for 1 ms. */
	TimeTick_Configure( );

#if DEMO_WITH_LCD
	lcd_init();
#endif
}

#if DEMO_WITH_LCD
/**
 *  \brief draw frame for the demonstration
 */
static void gfx_draw_frame(void)
{
	/* Get rid of this compiler warning. */
	LCDD_DrawRectangleWithFill(NULL, 0, 0, BOARD_LCD_WIDTH-1, BOARD_LCD_HEIGHT-1,
			COLOR_CONVERT(COLOR_WHITE));
	LCD_DrawString(NULL, 10, 30, (const uint8_t *)"SAMV7 DSP Demo",
			COLOR_CONVERT(COLOR_BLACK));

	// gfx_refresh_wav()
	LCDD_DrawRectangle(NULL, CHART_BORDER, 60, CHART_WIDTH - 1,
			CHART_HEIGHT - 1, COLOR_CONVERT(COLOR_BLACK));
	LCDD_DrawRectangleWithFill(NULL, CHART_BORDER + 1,
							   60 + 1,
							   CHART_WIDTH - 2,
							   CHART_HEIGHT - 2,
							   COLOR_CONVERT(COLOR_WHITE));

	// gfx_refresh_mag()
	LCDD_DrawRectangle(NULL, CHART_BORDER,
					60 + CHART_HEIGHT + 15,
					CHART_WIDTH - 1,
					mag_botton_y - (60 + CHART_HEIGHT + 15) - 1,
					COLOR_CONVERT(COLOR_BLACK));
	LCDD_DrawRectangleWithFill(NULL, CHART_BORDER + 1,
							60 + CHART_HEIGHT + 15 + 1,
							CHART_WIDTH - 2,
							mag_botton_y - 1 - (60 + CHART_HEIGHT + 15 + 1),
							COLOR_CONVERT(COLOR_WHITE));
	LCD_DrawString(NULL, CHART_BORDER, mag_botton_y + 15, (const uint8_t *)"0Hz",
			COLOR_CONVERT(COLOR_BLACK));
	LCD_DrawString(NULL, 230, mag_botton_y + 15, (const uint8_t *)"11kHz",
			COLOR_CONVERT(COLOR_BLACK));
}

/**
 *  \brief refresh the wave form on the LCD
 */
static void gfx_refresh_wav(void)
{
	uint32_t i = 0;
	uint32_t col = CHART_BORDER + 1;
	float32_t display_factor = 64;
	float32_t pixel;

	static uint32_t first_draw = 1;
	if (first_draw) {
		first_draw = 0;
		while (i < ( 2 * ( CHART_WIDTH-2 ) ) )
		{
			pixel = (wav_in_buffer[i] * display_factor);
			if ((pixel < (CHART_HEIGHT/2)) && (pixel > -(CHART_HEIGHT/2)))
				LCDD_DrawLine(NULL, col, (uint32_t)(wav_center_y - pixel), col,
						wav_center_y, COLOR_CONVERT(COLOR_BLUE));
			i = i + 2;
			col = col + 1;
		}
		return;
	}

	while (i < ( 2 * ( CHART_WIDTH-2 ) ) )
	{
		if ( wav_in_backup[i] * wav_in_buffer[i] > 0 )
		{
			float32_t pixel_bk = wav_in_backup[i] * display_factor;
			pixel = wav_in_buffer[i] * display_factor;
			LCDD_DrawLine( NULL, col, (uint32_t)(wav_center_y - pixel),
						col,(uint32_t)(wav_center_y - pixel_bk),
						((( pixel >= 0) && (wav_in_buffer[i] > wav_in_backup[i]))||
							 (( pixel < 0) && (wav_in_buffer[i] < wav_in_backup[i]))
						) ? COLOR_CONVERT(COLOR_BLUE) : COLOR_CONVERT(COLOR_WHITE) );
		}
		else
		{
			pixel = (wav_in_backup[i] * display_factor);
			if ((pixel < (CHART_HEIGHT/2)) && (pixel > -(CHART_HEIGHT/2)))
				LCDD_DrawLine(NULL, col, (uint32_t)(wav_center_y - pixel), col,
					wav_center_y, COLOR_CONVERT(COLOR_WHITE));
			pixel = (wav_in_buffer[i] * display_factor);
			if ((pixel < (CHART_HEIGHT/2)) && (pixel > -(CHART_HEIGHT/2)))
				LCDD_DrawLine(NULL, col, (uint32_t)(wav_center_y - pixel), col,
					wav_center_y, COLOR_CONVERT(COLOR_BLUE));
		}
		i = i + 2;
		col = col + 1;
	}
}

/**
 *  \brief refresh the magnitudes in frequency domain on the LCD
 */
static void gfx_refresh_mag(void)
{
	uint32_t i = 0;
	uint32_t col = CHART_BORDER + 2;

	static uint32_t first_draw = 1;
	if (first_draw){
		first_draw = 0;
		for (i = 0; i < 256/2; i++, col += 2)
			LCDD_DrawLine(NULL, col, mag_botton_y - 1, col,
				mag_botton_y - 1 - mag_in_buffer_int[i], COLOR_CONVERT(COLOR_RED));
		return;
	}
	for (i = 0; i < 256/2; i++, col += 2){
		LCDD_DrawLine( NULL, col, mag_botton_y - 1 - mag_in_buffer_int[i],
					col, mag_botton_y - 1 - mag_in_backup[i],
					(mag_in_buffer_int[i] > mag_in_backup[i]) ?
							COLOR_CONVERT(COLOR_RED) : COLOR_CONVERT(COLOR_WHITE));
	}
}

/**
 *  \brief refresh the signal type on the LCD
 *
 *  param x             horizontal position
 *  param y             vertical position
 *  param signal_type   signal type
 */
static void gfx_refresh_signal_type(uint32_t x, uint32_t y, uint32_t signal_type)
{
	static uint32_t type_bk = EN_SIGINAL_BUTT;
	if (type_bk != signal_type){
		type_bk = signal_type;
		uint32_t w, h;
		LCDD_GetStringSize(str_signal_type[3], &w, &h);
		LCDD_DrawRectangleWithFill(NULL, x, y, w, h, COLOR_CONVERT(COLOR_WHITE));
		LCD_DrawString(NULL, x, y, str_signal_type[signal_type], COLOR_CONVERT(COLOR_RED));
	}
}

/**
 *  \brief refresh the data type on the LCD
 *
 *  param x             horizontal position
 *  param y             vertical position
 *  param data_type     data type
 */
static void gfx_refresh_data_type(uint32_t x, uint32_t y, uint32_t data_type)
{
	static uint32_t data_type_bk = EN_DATA_TYPE_BUTT;
	if (data_type_bk != data_type){
		data_type_bk = data_type;
		uint32_t w, h;
		LCDD_GetStringSize(str_data_type[0], &w, &h);
		LCDD_DrawRectangleWithFill(NULL, x, y, w, h, COLOR_CONVERT(COLOR_WHITE));
		LCD_DrawString(NULL, x, y, str_data_type[data_type], COLOR_CONVERT(COLOR_RED));
	}
}
#endif

/**
 *  \brief refresh other information on the LCD
 *
 *  param x             horizontal position
 *  param y             vertical position
 *  param fft_size      size of FFT
 *  param time_us       uSeconds FFT process taken
 */
static void gfx_refresh_info(uint32_t x, uint32_t y, uint32_t fft_size, uint32_t time_us)
{
	static uint8_t fftSpeed[] = "FFT1024:xxxxus, CPU:1.6%";
	static uint8_t strCPU[] = "1.6%";
	static uint32_t time_bk = 0;
	static uint32_t fftSize_bk;
	uint32_t cpu_usage;
#if DEMO_WITH_LCD
	uint32_t w, h;
#endif

	if (fftSize_bk != fft_size){
		fftSize_bk = fft_size;
		switch (fft_size){
			case 256:
				fftSpeed[0] = ' ';  fftSpeed[1] = 'F';  fftSpeed[2] = 'F';
				fftSpeed[3] = 'T';  fftSpeed[4] = '2';  fftSpeed[5] = '5';  fftSpeed[6] = '6';
				break;
			case 1024:
				fftSpeed[0] = 'F';  fftSpeed[1] = 'F';  fftSpeed[2] = 'T';
				fftSpeed[3] = '1';  fftSpeed[4] = '0';  fftSpeed[5] = '2';  fftSpeed[6] = '4';
				break;
			case 4096:
				fftSpeed[0] = 'F';  fftSpeed[1] = 'F';  fftSpeed[2] = 'T';
				fftSpeed[3] = '4';  fftSpeed[4] = '0';  fftSpeed[5] = '9';  fftSpeed[6] = '6';
				break;
			default:
				break;
		}
	}

	if (time_bk != time_us)
	{
		time_bk = time_us;

		/* cpu_usage = fft_time(us/time) * (samples_rate/fft_size) / \
				(1s * 1000ms/s * 1000us/s) * 100 % */
		/* multiply 2 is for channel L&R */
		cpu_usage = 2 * time_us * SAMPLING_FREQUENCY / fft_size / 1000;

		char * pChar = (char *)&fftSpeed[8 + 4];
		*--pChar = '0' + (time_us % 10);
		time_us /= 10;
		*--pChar = '0' + (time_us % 10);
		time_us /= 10;
		*--pChar = '0' + (time_us % 10);
		time_us /= 10;
		*--pChar = '0' + (time_us % 10);
		if (time_bk < 1000)  fftSpeed[8 + 0] = ' ';
		if (time_bk < 100)   fftSpeed[8 + 1] = ' ';
		if (time_bk < 10)    fftSpeed[8 + 2] = ' ';

		strCPU[2] = '0' + (cpu_usage % 10);
		cpu_usage /= 10;
		strCPU[0] = '0' + (cpu_usage % 10);
		fftSpeed[20] = strCPU[0];
		fftSpeed[22] = strCPU[2];

#if DEMO_WITH_LCD
		LCDD_GetStringSize(fftSpeed, &w, &h);
		LCDD_DrawRectangleWithFill(NULL, x, y, CHART_BORDER + w - x, h,
				COLOR_CONVERT(COLOR_WHITE));
		LCD_DrawString(NULL, x, y, fftSpeed, COLOR_CONVERT(COLOR_BLUE));
#endif
	}

	strCPU[3] = ' ';
	printf("%16s  %5u %2u   %s    %4u   %4u   %4u  %s %8u %8u\r\n",
		   SIGNAL_TYPES[pDemoPara->signal_type],
		   (unsigned)pDemoPara->frequency, (unsigned)pDemoPara->duty_ratio,
		   str_data_type[pDemoPara->data_type], (unsigned)fft_size,
		   (unsigned int)time_fft_us,
		   (unsigned)time_total_us,
		   strCPU,
		   (unsigned int)CycleCounter[0], (unsigned int)CycleCounter[1]);
}

#if DEMO_WITH_LCD
/**
 *  \brief draw the demonstration on the LCD
 *
 */
static void gfx_task(void)
{
	static uint32_t first_run = 1;
	if (first_run) {
		first_run = 0;
		gfx_draw_frame();
	}
	LCDD_DrawLine(NULL, CHART_BORDER - 5,
				  wav_center_y,
				  CHART_BORDER + CHART_WIDTH + 5,
				  wav_center_y,
				  COLOR_CONVERT(COLOR_BLACK));

	/* average magnitudes of FFT1024 or 4096 by 4 or 16 */
	if (pDemoPara->fft_size != 256)
	{
		uint32_t index;
		uint32_t cnt;
		uint32_t mag_tmp;
		uint32_t * pMag = mag_in_buffer_int;
		for(index = 0; index < 256; index ++){
			mag_tmp = 0;
			for(cnt = 0; cnt < pDemoPara->fft_factor; cnt ++) {
				mag_tmp += *pMag ++;
			}
			mag_in_buffer_int[index] = mag_tmp/pDemoPara->fft_factor;
		}
	}

	gfx_refresh_wav();
	gfx_refresh_mag();

	{
		uint32_t i;
		uint32_t idx = 0;
		for(i=0; i<CHART_WIDTH; i++){
			wav_in_backup[idx] = wav_in_buffer[idx];
			idx++;
			wav_in_backup[idx] = wav_in_buffer[idx];
			idx++;
			mag_in_backup[i] = mag_in_buffer_int[i];
		}
	}

	gfx_refresh_signal_type(200, 30, pDemoPara->signal_type);

	gfx_refresh_data_type(135, mag_botton_y + 15, pDemoPara->data_type);

	/* display the computation time of FFT  */
	gfx_refresh_info(10, 455, pDemoPara->fft_size, time_total_us);
}
#endif

/**
 *  \brief get signal parameter
 *
 */
static void get_signal_parameter(void)
{
	static uint32_t adjust = 1;
	static uint32_t frequency = 10;
	if (adjust) {
		if (frequency > 1000)    frequency += 200;
		else                    frequency += 20;
		if ( frequency >= (SAMPLING_FREQUENCY/2) ) {
			adjust = 0;
		}
	} else {
		if (frequency > 1000)    frequency -= 200;
		else                    frequency -= 20;
		if (frequency<=10) {
			adjust = 1;
			pDemoPara->duty_ratio = TRNG_GetRandData() & 0x7F;
			if (90 < pDemoPara->duty_ratio)  pDemoPara->duty_ratio = 90;
			if (10 > pDemoPara->duty_ratio)  pDemoPara->duty_ratio = 10;
		}
	}
	pDemoPara->frequency = frequency;
}

#if DEMO_WITH_LCD
/**
 * \brief adjust magnitude before display on the LCD.
 *
 * \param fft_size   size of FFT.
 * \param data_type  data type.
 */
static void adjust_magnitude(uint32_t fft_size, EN_DATA_TYPE data_type)
{
	uint32_t i;
	float32_t bin;
	float32_t display_factor = (EN_DATA_TYPE_F32 == data_type) ? 1 : 1024;

	/* Prepare bins rendering for display. */
	/* Bins are printed using col, incremented by mean of 1 */
	for (i = 0; i < fft_size; ++i){
		bin = (mag_in_buffer[i] * display_factor * pDemoPara->fft_factor);
		if (bin > 0)	{
			if (bin > (CHART_HEIGHT - 2))    bin = (CHART_HEIGHT - 2);
			mag_in_buffer_int[i] = (uint32_t)bin;
		}
		else
			mag_in_buffer_int[i] = 0;
	}
}
#endif

/**
 * \brief update the demonstration parameters.
 *
 */
static void update_demo_parameter(void)
{
	pDemoPara->data_type += 1;
	if (EN_DATA_TYPE_BUTT <= pDemoPara->data_type)
	{
		pDemoPara->data_type = (EN_DATA_TYPE)0;
		if (256 == pDemoPara->fft_size){
			pDemoPara->fft_size = 1024;
		}
		else if (1024 == pDemoPara->fft_size){
			pDemoPara->fft_size = 4096;
		}
		else{
			pDemoPara->fft_size = 256;
			pDemoPara->signal_type += 1;
			if (EN_SIGINAL_BUTT <= pDemoPara->signal_type)
				pDemoPara->signal_type = (EN_SIGINAL_TYPE)0;
		}
	}
	pDemoPara->fft_factor = pDemoPara->fft_size/256;
}

/**
 * \brief main loop
 *
 */
static void main_loop(void)
{
	uint32_t sample_number;
	static uint32_t loop_cnt = 0;
	static const uint32_t loop_num[] = {
			200, 100, 200, 200, 100, 100
	};

	if (pause_flag) {
		pause_flag = 0;
		Wait(5000);
	}
	if (loop_cnt++ >= loop_num[pDemoPara->signal_type]){
		loop_cnt = 0;
		update_demo_parameter();
	}
	sample_number = (pDemoPara->fft_size > CHART_WIDTH) ? pDemoPara->fft_size :
			CHART_WIDTH;

	switch (pDemoPara->signal_type){
		case EN_SIGINAL_SOUND:
			get_real_time_audio(sample_number, pDemoPara->fft_size);
			break;
		case EN_SIGINAL_SINUS:
			get_signal_parameter();
			generate_sinus_wave(sample_number, pDemoPara->frequency);
			break;
		case EN_SIGINAL_SQUARE:
			get_signal_parameter();
			generate_square_wave(sample_number, pDemoPara->frequency,
					pDemoPara->duty_ratio/100.0);
			break;
		case EN_SIGINAL_TRIANGLE:
			get_signal_parameter();
			generate_triangle_wave(sample_number, pDemoPara->frequency,
					pDemoPara->duty_ratio/100.0);
			break;
		case EN_SIGINAL_WHITE_NOISE:
			generate_white_noise(sample_number);
			break;
		case EN_SIGINAL_PINK_NOISE:
			generate_pink_noise(sample_number);
			break;
		default:
			break;
	}

	__disable_irq();
	fft_process(pDemoPara->fft_size, pDemoPara->data_type);
	__enable_irq();

#if DEMO_WITH_LCD
	adjust_magnitude(pDemoPara->fft_size, pDemoPara->data_type);

	/* start refreshing screen */
	gfx_task();
	LCDD_UpdateWindow();
#else
	/* display the computation time of FFT  */
	gfx_refresh_info(10, 455, pDemoPara->fft_size, time_total_us);
#endif
}

/*----------------------------------------------------------------------------
 *        Exported functions
 *----------------------------------------------------------------------------*/
/**
 *  \brief FFT_demo Application entry point.
 *
 *  \return Unused (ANSI-C compatibility).
 */
int main( void )
{
	/* Disable watchdog */
	WDT_Disable(WDT);

	SCB_EnableICache();
	SCB_EnableDCache();

	/* Output example information */
	printf( "\n\r-- FFT Demo %s --\n\r", SOFTPACK_VERSION );
	printf( "-- %s\n\r", BOARD_NAME );
	printf( "-- Compiled: %s %s With %s--\n\r", __DATE__, __TIME__ , COMPILER_NAME);

	initialize();

	pDemoPara->signal_type = EN_SIGINAL_SINUS;
	pDemoPara->fft_size = 256;
	pDemoPara->data_type = EN_DATA_TYPE_Q15;

	printf( "    Signal type | Freq |  | DType | size | time | total | CPU | Cycle | Cycle\r\n");
	while (1) {
		main_loop();
	}
}
