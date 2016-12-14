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
 *  \page tcm TCM (Tightly Coupled Memory) Example
 *
 *  \section Purpose
 *
 *  The tcm example will help new users get familiar with Atmel's
 *  SAMV7/E7 family of Microcontroller's tcm memory and there are benefits to
 *  run code with low latency.
 *
 *  \section Requirements
 *
 *  This package can be used with SAMV71 Xplained Ultra board or SAME70 Xplained board.
 *
 *  \section Description
 *
 *  The demonstration program compares the performance between normal 'memcpy'
 * and 'TCM memcpy'.
 *
 *  \section Usage
 *
 *  -# Build the program and download it inside the board.
 *     Please refer to the Getting Started with SAM V71/E70 Microcontrollers.pdf
 *  -# On the computer, open and configure a terminal application
 *     (e.g. HyperTerminal on Microsoft Windows) with these settings:
 *    - 115200 baud rates
 *    - 8 bits of data
 *    - No parity
 *    - 1 stop bit
 *    - No flow control
 *  -# Start the application.
 *  -# In the terminal window, the following text should appear
 *  (values depend on the board and chip used):
 *     \code
 *      -- TCM memory Example xxx --
 *      -- xxxxxx-xx
 *      -- Compiled: xxx xx xxxx xx:xx:xx --
 *     \endcode
 *
 *  \section References
 *  - tcm/main.c
 *  - board.h
 */

/** \file
 *
 *  This file contains all the specific code for the TCM example.
 *
 */

/*----------------------------------------------------------------------------
 *        Headers
 *----------------------------------------------------------------------------*/

#include "board.h"

#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "..\..\..\..\utils\utility.h"
/*----------------------------------------------------------------------------
 *        Global definitions
 *----------------------------------------------------------------------------*/
extern void TCM_StackInit(void);

/*----------------------------------------------------------------------------
 *        Local definitions
 *----------------------------------------------------------------------------*/
#define BUFF_SIZE       0x1000

/*----------------------------------------------------------------------------
 *        Local variables
 *----------------------------------------------------------------------------*/

#if defined ( __ICCARM__ ) /* IAR Ewarm */
#pragma location = ".data_TCM"
#elif defined (  __GNUC__  ) || defined (__CC_ARM)  /* GCC || MDK */
__attribute__((__section__(".data_TCM")))
#endif
uint8_t TCM_DestBuff[3*BUFF_SIZE];

#if defined ( __ICCARM__ ) /* IAR Ewarm */
#pragma location = ".data_TCM"
#elif defined (  __GNUC__  ) || defined (__CC_ARM)  /* GCC || MDK */
__attribute__((__section__(".data_TCM")))
#endif
uint8_t TCM_SrcBuff[BUFF_SIZE];

const char *pBuffer = "The ARM architecture is defined in a hierarchical manner,\
where the features are described in Chapter A2\
Application Level Programmers?Model at the application level,\
with underlying system support. What features are available and how they are \
supported is defined in the architecture profiles, making the system \
level support profile specific. Deprecated features can be found in an \
appendix to this manual. See page. As stated in Privileged execution on page \
A2-13, programs can execute in a privileged or unprivileged manner. System \
level support requires privileged access, allowing it the access permissions \
to configure and control the resources. This is typically supported by an \
operating system, which provides system services to the applications, either \
transparently, or through application initiated service calls. The operating \
system is also responsible for servicing interrupts and other system events, \
making exceptions a key component of the system level programmers?model.";

uint8_t DestBuff[3*BUFF_SIZE], SrcBuff[BUFF_SIZE];

static uint32_t CycleCounter;

/*----------------------------------------------------------------------------
 *        Local functions
 *----------------------------------------------------------------------------*/

#if defined ( __ICCARM__ ) /* IAR Ewarm */
#pragma default_function_attributes = @ ".code_TCM"
#elif defined (  __GNUC__  ) || defined (__CC_ARM)  /* GCC || MDK */
__attribute__((__section__(".code_TCM")))
#endif
static uint32_t TCM_memcpy(uint8_t *pDest, uint8_t *pSrc, uint16_t len)
{
	// clean destination and source buffer
	memset(TCM_SrcBuff, 0, len);
	__DSB();
	__ISB();
	memset(TCM_DestBuff, 0, len);
	__DSB();
	__ISB();

	// copy buffer to TCM source buffer
	memcpy(pSrc, pBuffer,len);
	__DSB();
	__ISB();

	// Disable and reset DWT cycle counter
	RESET_CYCLE_COUNTER();
	// Copy from DTCM source buffer to DTCM destination buffer
	{
		uint32_t i;
		uint8_t * pTmpD = pDest;
		uint8_t * pTmpS;

		for (i = 0, pTmpS = pSrc; i < len; i++)
			*pTmpD++ = *pTmpS++;
		for (i = 0, pTmpS = pSrc; i < len; i++)
			*pTmpD++ = *pTmpS++;
		for (i = 0, pTmpS = pSrc; i < len; i++)
			*pTmpD++ = *pTmpS++;
	}
	GET_CYCLE_COUNTER(CycleCounter);
	return CycleCounter;
}


static uint32_t Normal_memcpy(uint8_t *pDest, uint8_t *pSrc, uint16_t len)
{
	memset(SrcBuff, 0, len);
	__DSB();
	__ISB();
	memset(DestBuff, 0, len);
	__DSB();
	__ISB();
	memcpy(SrcBuff, pBuffer,len);
	__DSB();
	__ISB();

	// Disable and reset DWT cycle counter
	RESET_CYCLE_COUNTER();
	// Copy from DTCM source buffer to DTCM destination buffer
	{
		uint32_t i;
		uint8_t * pTmpD = pDest;
		uint8_t * pTmpS;
		for (i = 0, pTmpS = pSrc; i < len; i++)
			*pTmpD++ = *pTmpS++;
		for (i = 0, pTmpS = pSrc; i < len; i++)
			*pTmpD++ = *pTmpS++;
		for (i = 0, pTmpS = pSrc; i < len; i++)
			*pTmpD++ = *pTmpS++;
	}
	GET_CYCLE_COUNTER(CycleCounter);
	return CycleCounter;
}

static uint32_t Recursive(uint32_t n)
{
	volatile uint32_t tmp = n;

	if (0 == tmp)
		return 0;
	else
		return n + Recursive(tmp - 1);
}

static uint32_t Stack_Test(uint32_t n)
{
	uint32_t sum;
	RESET_CYCLE_COUNTER();
	sum = Recursive(n);
	GET_CYCLE_COUNTER(CycleCounter);
	printf("\r\n Sum of 1 to %u is %u, ", (unsigned)n, (unsigned)sum);
	return CycleCounter;
}

/*----------------------------------------------------------------------------
 *        Exported functions
 *----------------------------------------------------------------------------*/
/**
 *  \brief Application entry point for TCM example.
 *
 *  \return Unused (ANSI-C compatibility).
 */
extern int main( void )
{
	uint32_t CycleCounterOffset = 0;
	uint32_t Cycles=0;

	/* Disable watchdog */
	WDT_Disable(WDT);

	/* Output example information */
	printf("\n\r-- TCM memory Example %s --\n\r", SOFTPACK_VERSION);
	printf("-- %s\n\r", BOARD_NAME);
	printf("-- Compiled: %s %s With %s--\n\r", __DATE__, __TIME__, COMPILER_NAME);

	RESET_CYCLE_COUNTER();
	__NOP();
	__NOP();
	__NOP();
	__NOP();
	__NOP();
	__NOP();
	__NOP();
	__NOP();
	__NOP();
	__NOP();

	CycleCounterOffset = DWT->CYCCNT;
	__DMB();
	CycleCounterOffset -=10;

	printf("\n\r");
	TRACE_INFO(" ------ ICache & DCache Disabled ------\n\r");
	TRACE_INFO(" ------ Stack In SRAM ------\n\r");

	/* stack test: in SRAM vs in DTCM */
	Cycles = Stack_Test(100);
	printf("Inst Cycles passed is %u\r\n", (unsigned)(CycleCounter - CycleCounterOffset));

	Cycles = Stack_Test(100);
	printf("Inst Cycles passed is %u\r\n", (unsigned)(CycleCounter - CycleCounterOffset));

	printf("\n\r");
	TRACE_INFO(" ------ Stack In DTCM ------\n\r");
	TCM_StackInit();

	Cycles = Stack_Test(100);
	printf("Inst Cycles passed is %u\r\n", (unsigned)(CycleCounter - CycleCounterOffset));

	Cycles = Stack_Test(100);
	printf("Inst Cycles passed is %u\r\n", (unsigned)(CycleCounter - CycleCounterOffset));

	/* memory copy test */
	Cycles = Normal_memcpy(DestBuff, SrcBuff, BUFF_SIZE);
	printf("\n\r Number of Inst Cycles passed without TCM is %u  \n\r",
			(unsigned)(Cycles - CycleCounterOffset));

	Cycles = TCM_memcpy(TCM_DestBuff, TCM_SrcBuff, BUFF_SIZE);
	printf("\n\r Number of Inst Cycles passed with TCM is %u \n\r",
			(unsigned)(Cycles - CycleCounterOffset));

	SCB_EnableICache();
	SCB_EnableDCache();

	printf("\n\r");
	TRACE_INFO(" ------ ICache & DCache Enabled ------\n\r");

	Cycles = Normal_memcpy(DestBuff, SrcBuff, BUFF_SIZE);
	printf("\n\r Number of Inst Cycles passed without TCM is %u  \n\r",
			(unsigned)(Cycles - CycleCounterOffset));

	Cycles = TCM_memcpy(TCM_DestBuff, TCM_SrcBuff, BUFF_SIZE);
	printf("\n\r Number of Inst Cycles passed with TCM is %u  \n\r",
			(unsigned)(Cycles - CycleCounterOffset));

	while (1);
}
