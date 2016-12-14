/* ----------------------------------------------------------------------------
 *         SAM Software Package License
 * ----------------------------------------------------------------------------
 * Copyright (c) 2014, Atmel Corporation
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * - Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the disclaimer below.
 *
 * Atmel's name may not be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * DISCLAIMER: THIS SOFTWARE IS PROVIDED BY ATMEL "AS IS" AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT ARE
 * DISCLAIMED. IN NO EVENT SHALL ATMEL BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
 * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * ----------------------------------------------------------------------------
 */


/*----------------------------------------------------------------------------
 *        Headers
 *----------------------------------------------------------------------------*/
#include "board.h"
#include "fft.h"

/** macro to control whether to use deprecated functions in CMSIS-DSP v1.4.5  */
#if defined(__GNUC__)
	#define USING_DEPRECATED 1
#else
	#define USING_DEPRECATED 0
#endif

#if 0 == USING_DEPRECATED
	#include "arm_const_structs.h"
	extern const arm_cfft_instance_q15 arm_cfft_sR_q15_len256;
	extern const arm_cfft_instance_q15 arm_cfft_sR_q15_len1024;
	extern const arm_cfft_instance_q15 arm_cfft_sR_q15_len4096;
	extern const arm_cfft_instance_q31 arm_cfft_sR_q31_len256;
	extern const arm_cfft_instance_q31 arm_cfft_sR_q31_len1024;
	extern const arm_cfft_instance_q31 arm_cfft_sR_q31_len4096;
#endif

/*----------------------------------------------------------------------------
 *        Imported variables
 *----------------------------------------------------------------------------*/

/*----------------------------------------------------------------------------
 *        Exported variables
 *----------------------------------------------------------------------------*/
volatile uint32_t time_total_us;
int32_t time_fft_us;
uint32_t CycleCounter[3];

/** wave buffer in float */
float32_t wav_in_buffer[MAX_FFT_SIZE * 2];

/** Magnitude buffer converted in float */
float32_t mag_in_buffer[MAX_FFT_SIZE];


#if defined(__GNUC__)

//__attribute__( ( always_inline ) )
float32_t sqrtf(float32_t in)
{
	float32_t out;
	__asm__ ("VSQRT.F32 %0, %1" : "=t" (out) : "t" (in));
	return(out);
}

/* copied from libraries\libchip_samv7\include\cmsis\CMSIS\DSP_Lib\Source\CommonTables\arm_common_tables.c
   and masked out some tables not used in this demo
*/
#include "arm_common_tables.c"

#include "cmsis\CMSIS\DSP_Lib\Source\SupportFunctions\arm_float_to_q15.c"
#include "cmsis\CMSIS\DSP_Lib\Source\SupportFunctions\arm_float_to_q31.c"
#include "cmsis\CMSIS\DSP_Lib\Source\SupportFunctions\arm_q15_to_float.c"
#include "cmsis\CMSIS\DSP_Lib\Source\SupportFunctions\arm_q31_to_float.c"
#include "cmsis\CMSIS\DSP_Lib\Source\ComplexMathFunctions\arm_cmplx_mag_f32.c"
#include "cmsis\CMSIS\DSP_Lib\Source\ComplexMathFunctions\arm_cmplx_mag_q15.c"
#include "cmsis\CMSIS\DSP_Lib\Source\ComplexMathFunctions\arm_cmplx_mag_q31.c"
#include "cmsis\CMSIS\DSP_Lib\Source\FastMathFunctions\arm_sin_f32.c"
#include "cmsis\CMSIS\DSP_Lib\Source\FastMathFunctions\arm_sqrt_q15.c"
#include "cmsis\CMSIS\DSP_Lib\Source\FastMathFunctions\arm_sqrt_q31.c"
#include "cmsis\CMSIS\DSP_Lib\Source\TransformFunctions\arm_bitreversal.c"
#include "cmsis\CMSIS\DSP_Lib\Source\TransformFunctions\arm_cfft_radix4_init_f32.c"
#include "cmsis\CMSIS\DSP_Lib\Source\TransformFunctions\arm_cfft_radix4_init_q15.c"
#include "cmsis\CMSIS\DSP_Lib\Source\TransformFunctions\arm_cfft_radix4_init_q31.c"
#include "cmsis\CMSIS\DSP_Lib\Source\TransformFunctions\arm_cfft_radix4_f32.c"
#include "cmsis\CMSIS\DSP_Lib\Source\TransformFunctions\arm_cfft_radix4_q15.c"
#include "cmsis\CMSIS\DSP_Lib\Source\TransformFunctions\arm_cfft_radix4_q31.c"
#endif

/*----------------------------------------------------------------------------
 *        Local variables
 *----------------------------------------------------------------------------*/

/** cFFT buffer declaration */
static union {
	/** q15 cFFT buffer declaration */
	q15_t q15[MAX_FFT_SIZE * 2];
	/** q31 cFFT buffer declaration, F31 cFFT use the same buffer */
	q31_t q31[MAX_FFT_SIZE * 2];
	/** q31 cFFT buffer declaration, F31 cFFT use the same buffer */
	q31_t f32[MAX_FFT_SIZE * 2];
}cfft_buffer;

/** cFFT instance declaration */
#if USING_DEPRECATED
static union {
	/** cFFT configuration instance declaration Q15 */
	arm_cfft_radix4_instance_q15 q15;
	/** cFFT configuration instance declaration Q31 */
	arm_cfft_radix4_instance_q31 q31;
	/** cFFT configuration instance declaration F32 */
	arm_cfft_radix4_instance_f32 f32;
#else
static struct {
	/** cFFT configuration instance declaration Q15 */
	const arm_cfft_instance_q15 * q15;
	/** cFFT configuration instance declaration Q31 */
	const arm_cfft_instance_q31 * q31;
	/** cFFT configuration instance declaration F32 */
	const arm_cfft_instance_f32 * f32;
#endif
}cfft_instance;

/** Magnitude buffer declaration */
static union {
	/** q15 Magnitude buffer declaration */
	q15_t q15[MAX_FFT_SIZE];
	/** q31 Magnitude buffer declaration */
	q31_t q31[MAX_FFT_SIZE];
	/* f32 not needed */
}magnitude_buffer;


/*----------------------------------------------------------------------------
 *        Exported functions
 *----------------------------------------------------------------------------*/
/**
 *  \brief FFT main process routine
 *
 *  \param fft_size   size of FFT.
 *  \param data_type  data type.
 *  \return void
 */
void fft_process(uint32_t fft_size, EN_DATA_TYPE data_type)
{
	uint32_t i;
	float32_t timer_factor;
	uint32_t CycleCounterOffset;

	RESET_CYCLE_COUNTER();
	__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
	CycleCounterOffset = DWT->CYCCNT;
	CycleCounterOffset -=10;

	if (EN_DATA_TYPE_F32 != data_type){
		TC_Configure(TC0, 1, TC_CMR_TCCLKS_TIMER_CLOCK3);
		timer_factor = BOARD_MCK/1000/1000/32.0;
	} else {
		TC_Configure(TC0, 1, TC_CMR_TCCLKS_TIMER_CLOCK2);
		timer_factor = BOARD_MCK/1000/1000/8.0;
	}

#if 0 == USING_DEPRECATED
	switch (fft_size){
		case 256:
			cfft_instance.q15 = &arm_cfft_sR_q15_len256;
			cfft_instance.q31 = &arm_cfft_sR_q31_len256;
			cfft_instance.f32 = &arm_cfft_sR_f32_len256;
			break;
		case 1024:
			cfft_instance.q15 = &arm_cfft_sR_q15_len1024;
			cfft_instance.q31 = &arm_cfft_sR_q31_len1024;
			cfft_instance.f32 = &arm_cfft_sR_f32_len1024;
			break;
		case 4096:
			cfft_instance.q15 = &arm_cfft_sR_q15_len4096;
			cfft_instance.q31 = &arm_cfft_sR_q31_len4096;
			cfft_instance.f32 = &arm_cfft_sR_f32_len4096;
			break;
		default:
			break;
	}
#endif

	/* Perform FFT and bin Magnitude calculation */
	LED_Clear(0);
	if (EN_DATA_TYPE_Q15 == data_type){
			TC_Start(TC0, 1);
			RESET_CYCLE_COUNTER();
		arm_float_to_q15(wav_in_buffer, cfft_buffer.q15, fft_size * 2);
#if USING_DEPRECATED
		arm_cfft_radix4_init_q15(&cfft_instance.q15, fft_size, 0, 1);
			GET_CYCLE_COUNTER(CycleCounter[0]);
			time_fft_us = TC0->TC_CHANNEL[1].TC_CV;
		arm_cfft_radix4_q15(&cfft_instance.q15, cfft_buffer.q15);
#else
			GET_CYCLE_COUNTER(CycleCounter[0]);
			time_fft_us = TC0->TC_CHANNEL[1].TC_CV;
		arm_cfft_q15(cfft_instance.q15, cfft_buffer.q15, 0, 1);
#endif
			GET_CYCLE_COUNTER(CycleCounter[2]);
			time_fft_us = TC0->TC_CHANNEL[1].TC_CV - time_fft_us;
		arm_cmplx_mag_q15(cfft_buffer.q15, magnitude_buffer.q15, fft_size);
		arm_q15_to_float(magnitude_buffer.q15, mag_in_buffer, fft_size);
			GET_CYCLE_COUNTER(CycleCounter[1]);
			TC_Stop(TC0, 1);
	}
	else if (EN_DATA_TYPE_Q31 == data_type){
			TC_Start(TC0, 1);
			RESET_CYCLE_COUNTER();
		arm_float_to_q31(wav_in_buffer, cfft_buffer.q31, fft_size * 2);
#if USING_DEPRECATED
		arm_cfft_radix4_init_q31(&cfft_instance.q31, fft_size, 0, 1);
			GET_CYCLE_COUNTER(CycleCounter[0]);
			time_fft_us = TC0->TC_CHANNEL[1].TC_CV;
		arm_cfft_radix4_q31(&cfft_instance.q31, cfft_buffer.q31);
#else
			GET_CYCLE_COUNTER(CycleCounter[0]);
			time_fft_us = TC0->TC_CHANNEL[1].TC_CV;
		arm_cfft_q31(cfft_instance.q31, cfft_buffer.q31, 0, 1);
#endif
			GET_CYCLE_COUNTER(CycleCounter[2]);
			time_fft_us = TC0->TC_CHANNEL[1].TC_CV - time_fft_us;
		arm_cmplx_mag_q31(cfft_buffer.q31, magnitude_buffer.q31, fft_size);
		arm_q31_to_float(magnitude_buffer.q31, mag_in_buffer, fft_size);
			GET_CYCLE_COUNTER(CycleCounter[1]);
			TC_Stop(TC0, 1);
	}
	else if (EN_DATA_TYPE_F32 == data_type){
		float32_t *pTmp = (float32_t *)cfft_buffer.q31;
		for (i = 0; i < fft_size * 2; i++)
			pTmp[i] = wav_in_buffer[i];
			TC_Start(TC0, 1);
			RESET_CYCLE_COUNTER();
#if USING_DEPRECATED
		arm_cfft_radix4_init_f32(&cfft_instance.f32, fft_size, 0, 1);
			GET_CYCLE_COUNTER(CycleCounter[0]);
			time_fft_us = TC0->TC_CHANNEL[1].TC_CV;
		arm_cfft_radix4_f32(&cfft_instance.f32, pTmp);
#else
			GET_CYCLE_COUNTER(CycleCounter[0]);
			time_fft_us = TC0->TC_CHANNEL[1].TC_CV;
		arm_cfft_f32(cfft_instance.f32, pTmp, 0, 1);
#endif
			GET_CYCLE_COUNTER(CycleCounter[2]);
			time_fft_us = TC0->TC_CHANNEL[1].TC_CV - time_fft_us;
		arm_cmplx_mag_f32(pTmp, mag_in_buffer, fft_size);
			GET_CYCLE_COUNTER(CycleCounter[1]);
			TC_Stop(TC0, 1);
	}
	time_total_us = (uint32_t)(TC0->TC_CHANNEL[1].TC_CV / timer_factor);
	LED_Set(0);
	time_fft_us /= timer_factor;
	CycleCounter[0] = CycleCounter[2] - CycleCounter[0];
	CycleCounter[1] -= CycleCounterOffset;
}
