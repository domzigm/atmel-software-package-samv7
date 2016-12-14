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
 *  \page mpu MPU with SAMV7/E7 Microcontrollers
 *
 *  \section Purpose
 *
 *  The MPU example will help new users get familiar with the MPU of cortex-m7
 *  on Atmel's SAMV7/E7 family of microcontrollers. This basic application shows
 *  the setup of the basic MPU memory regions and how to use the MPU module.
 *
 *  \section Requirements
 *
 *  This package can be used with SAMV71 Xplained Ultra board or SAME70 Xplained board.
 *
 *  \section Description
 *
 *  At startup, the program will setup basic memory regions, then user can
 *  update and observe the memory management fault exception through the serial
 *  port.
 *
 *  \section Usage
 *
 *  -# Build the program and download it inside the board.
 *  Please refer to the Getting Started with SAM V71/E70 Microcontrollers.pdf
 *  -# On the computer, open and configure a terminal application
 *     (e.g. HyperTerminal on Microsoft Windows) with these settings:
 *    - 115200 baud rate
 *    - 8 bits of data
 *    - No parity
 *    - 1 stop bit
 *    - No flow control
 *  -# Start the application.
 *  -# In the terminal window, the following text should appear (values depend
 *  on the board and chip used):
 *     \code
 *      -- MPU Example xxx --
 *      -- xxxxxx-xx
 *      -- Compiled: xxx xx xxxx xx:xx:xx --
 *     \endcode
 *
 * \section References
 * - mpu\main.c
 *
 */

/** \file
 *
 *  This file contains all the specific code for the MPU example.
 *
 */

/*----------------------------------------------------------------------------
 *        Headers
 *----------------------------------------------------------------------------*/
#include "board.h"
#include <stdbool.h>
#include <stdio.h>

/*----------------------------------------------------------------------------
 *        Local definitions
 *----------------------------------------------------------------------------*/

#define MPU_UNPRIVILEGED_RAM_REGION       ( 0 )
#define MPU_PRIVILEGE_RAM_REGION          ( 1 )
#define MPU_PRIVILEGE_RAM_REGION_2        ( 2 )
#define MPU_UNPRIVILEGED_FLASH_REGION     ( 3 )
#define MPU_PRIVILEGED_FLASH_REGION       ( 4 )
#define MPU_PERIPHERALS_REGION_0          ( 5 )
#define MPU_PERIPHERALS_REGION_1          ( 6 )
#define MPU_PIOC_REGION_REGION            ( 7 )

/**************  Peripherals memory region macros for MPU example ********/
#define MPU_PERIPHERALS_START_ADDRESS_0     0x40000000UL
#define MPU_PERIPHERALS_END_ADDRESS_0       (0x400E0E00UL - 1)

/**************  Peripherals memory region macros for MPU example ********/
#define MPU_PERIPHERALS_START_ADDRESS_1     0x400E0E00UL
#define MPU_PERIPHERALS_END_ADDRESS_1       0x400E2000UL

/**************  Peripherals memory sub region macros for MPU example ********/
#define MPU_PIOC_PERIPHERALS_REGION_START_ADDRESS    0x400E1200UL
#define MPU_PIOC_PERIPHERALS_REGION_END_ADDRESS      0x400E1400UL

/**************  sram memory  unprivileged for MPU example ********/
#define MPU_SRAM_UNPRIVILEGE_START_ADDRESS      (SRAM_START_ADDRESS+0)
#define MPU_SRAM_UNPRIVILEGE_END_ADDRESS        (SRAM_START_ADDRESS+0x2FFFF)

/**************  sram memory  privileged for MPU example ********/
#define MPU_SRAM_PRIVILEGE_START_ADDRESS        (SRAM_START_ADDRESS+0x30000)
#define MPU_SRAM_PRIVILEGE_END_ADDRESS          (SRAM_START_ADDRESS+0x4FFFF)

/**************  Second part of sram memory  privileged for MPU example *******/
#define MPU_SRAM_PRIVILEGE2_START_ADDRESS        (SRAM_START_ADDRESS+0x50000)
#define MPU_SRAM_PRIVILEGE2_END_ADDRESS          (SRAM_START_ADDRESS+0x5FFFF)

/*----------------------------------------------------------------------------
 *        Local Variables
 *----------------------------------------------------------------------------*/
/** Flag to indicate whether the SVC is done */
uint32_t dwRaisePriDone = 0;

/*----------------------------------------------------------------------------
 *        Local functions
 *----------------------------------------------------------------------------*/

/**
 * \brief Function to raise the privilege, used by the assemble code
 */
static void _RaisePrivilege(void)
{
	/* As this function is called in privilege mode, we don't need to set the
			privilege mode*/
	printf("\n\r-I- Raise to Privilege mode\n\r");
	__set_CONTROL(PRIVILEGE_MODE);
}

/**
 * \brief Default SVC interrupt handler.
 */
void SVC_Handler(void)
{
	/* Set to privilege mode */
	__set_CONTROL(PRIVILEGE_MODE);

	/* Set flag */
	dwRaisePriDone = 1;
}

/**
 * \brief Default MemManage interrupt handler.
 * There is a weak handler by default and if the MPU feature is enabled,
 * it must be implemented with the actual handler
 */
void MemManage_Handler(void)
{
	uint32_t nCfsr, nMmfar;
	printf("\n\r-I- Memory Exceptions!!! \n\r");
	/* adjust the code to exit the MMF */
	nCfsr = SCB->CFSR;
	nMmfar = SCB->MMFAR;
	if (!(nCfsr & ( 1 << 1)))
		_RaisePrivilege();
	if ((nMmfar & 0x20400000) || (nMmfar & 0x400E0E00) )
		_RaisePrivilege();
	printf("\n\r-I- Exit Memory Exceptions!\n\r");
}

/**
 * \brief Setup a memory region.
 */
static void _SetupMPU(void)
{

	uint32_t dwRegionBaseAddr;
	uint32_t dwRegionAttr;

	/* Internal flash privilege memory region */
	dwRegionBaseAddr = IFLASH_PRIVILEGE_START_ADDRESS |
		MPU_REGION_VALID |
		MPU_PRIVILEGED_FLASH_REGION;

	dwRegionAttr = MPU_AP_FULL_ACCESS |
		MPU_CalMPURegionSize(IFLASH_PRIVILEGE_END_ADDRESS -
				IFLASH_PRIVILEGE_START_ADDRESS) |
		MPU_REGION_ENABLE;

	MPU_SetRegion(dwRegionBaseAddr, dwRegionAttr);


	/* Internal flash unprivileged memory region */
	dwRegionBaseAddr = IFLASH_START_ADDRESS |
		MPU_REGION_VALID |
		MPU_UNPRIVILEGED_FLASH_REGION;

	dwRegionAttr = MPU_AP_READONLY |
		MPU_CalMPURegionSize(IFLASH_END_ADDRESS - IFLASH_START_ADDRESS) |
		MPU_REGION_ENABLE;

	MPU_SetRegion(dwRegionBaseAddr, dwRegionAttr);

	/* SRAM memory unprivileged region */
	dwRegionBaseAddr = MPU_SRAM_UNPRIVILEGE_START_ADDRESS |
		MPU_REGION_VALID |
		MPU_UNPRIVILEGED_RAM_REGION;

	dwRegionAttr = MPU_AP_FULL_ACCESS |
		INNER_NORMAL_WB_NWA_TYPE(NON_SHAREABLE) |
		MPU_CalMPURegionSize(MPU_SRAM_UNPRIVILEGE_END_ADDRESS -
				MPU_SRAM_UNPRIVILEGE_START_ADDRESS) |
		MPU_REGION_ENABLE;

	MPU_SetRegion(dwRegionBaseAddr, dwRegionAttr);

	/* SRAM memory privilege region */
	dwRegionBaseAddr = MPU_SRAM_PRIVILEGE_START_ADDRESS |
		MPU_REGION_VALID |
		MPU_PRIVILEGE_RAM_REGION;

	dwRegionAttr = MPU_AP_FULL_ACCESS |
		INNER_NORMAL_WB_NWA_TYPE(NON_SHAREABLE ) |
		MPU_CalMPURegionSize(MPU_SRAM_PRIVILEGE_END_ADDRESS -
				MPU_SRAM_PRIVILEGE_START_ADDRESS) |
		MPU_REGION_ENABLE;

	MPU_SetRegion(dwRegionBaseAddr, dwRegionAttr);

	/* SRAM memory privilege region */
	dwRegionBaseAddr = MPU_SRAM_PRIVILEGE2_START_ADDRESS |
		MPU_REGION_VALID |
		MPU_PRIVILEGE_RAM_REGION_2;

	dwRegionAttr = MPU_AP_FULL_ACCESS |
		INNER_NORMAL_WB_NWA_TYPE(NON_SHAREABLE ) |
		MPU_CalMPURegionSize(MPU_SRAM_PRIVILEGE2_END_ADDRESS -
				MPU_SRAM_PRIVILEGE2_START_ADDRESS) |
		MPU_REGION_ENABLE;

	MPU_SetRegion(dwRegionBaseAddr, dwRegionAttr);

	/* Peripheral memory region */
	dwRegionBaseAddr = MPU_PERIPHERALS_START_ADDRESS_0 |
		MPU_REGION_VALID |
		MPU_PERIPHERALS_REGION_0;

	dwRegionAttr = MPU_AP_FULL_ACCESS |
		MPU_REGION_EXECUTE_NEVER |
		MPU_CalMPURegionSize(MPU_PERIPHERALS_END_ADDRESS_0 -
				MPU_PERIPHERALS_START_ADDRESS_0) |
		MPU_REGION_ENABLE;

	MPU_SetRegion(dwRegionBaseAddr, dwRegionAttr);

	/* Peripheral memory region */
	dwRegionBaseAddr = MPU_PERIPHERALS_START_ADDRESS_1 |
		MPU_REGION_VALID |
		MPU_PERIPHERALS_REGION_1;

	dwRegionAttr = MPU_AP_FULL_ACCESS |
		MPU_REGION_EXECUTE_NEVER |
		1 << MPU_RASR_SRD_Pos |
		MPU_CalMPURegionSize(MPU_PERIPHERALS_END_ADDRESS_1 -
				MPU_PERIPHERALS_START_ADDRESS_1) |
		MPU_REGION_ENABLE;

	MPU_SetRegion(dwRegionBaseAddr, dwRegionAttr);

	/* Sub Peripheral memory region */
	dwRegionBaseAddr = MPU_PIOC_PERIPHERALS_REGION_START_ADDRESS |
		MPU_REGION_VALID |
		MPU_PIOC_REGION_REGION;

	dwRegionAttr = MPU_AP_FULL_ACCESS |
		MPU_REGION_EXECUTE_NEVER |
		MPU_CalMPURegionSize(MPU_PIOC_PERIPHERALS_REGION_END_ADDRESS -
				MPU_PIOC_PERIPHERALS_REGION_START_ADDRESS) |
		MPU_REGION_ENABLE;

	MPU_SetRegion(dwRegionBaseAddr, dwRegionAttr);

	/* Enable the memory management fault exception */
	SCB->SHCSR |= SCB_SHCSR_MEMFAULTENA_Msk;

	/* Enable the MPU region */
	MPU_Enable(MPU_ENABLE | MPU_PRIVDEFENA);
}


/**
 *  \brief Update MPU regions.
 *
 *  \return Unused (ANSI-C compatibility).
 */
static void _UpdateMPU( uint32_t dwRegionNum, uint32_t dwRegionBaseAddr,
						uint32_t dwRegionAttr)
{
	/* Raise privilege, the MPU register could be set only in privilege mode */

	__ASM volatile(" svc 0x00 ");

	while (!dwRaisePriDone);
	dwRaisePriDone = 0;

	/* Disable interrupt */
	__disable_irq();

	/* Clean up data and instruction buffer */
	__DSB();
	__ISB();

	/* Set active region */
	MPU_SetRegionNum(dwRegionNum);

	/* Disable region */
	MPU_DisableRegion();

	/* Update region attribute */
	MPU_SetRegion( dwRegionBaseAddr, dwRegionAttr);

	/* Clean up data and instruction buffer to make the new region taking
		   effect at once */
	__DSB();
	__ISB();

	/* Enable the interrupt */
	__enable_irq();

	/* Reset to thread mode */
	__set_CONTROL(USER_MODE);
}

/*----------------------------------------------------------------------------
 *        Exported functions
 *----------------------------------------------------------------------------*/

/**
 *  \brief Application entry point for MPU example.
 *
 *  \return Unused (ANSI-C compatibility).
 */
extern int main( void )
{
	uint8_t ucChoice = 0;

	/* Disable watchdog */
	WDT_Disable(WDT);

	uint32_t *pdw = (uint32_t *)MPU_SRAM_PRIVILEGE_START_ADDRESS;

	/* Output example information */
	printf("\n\r\n\r\n\r");
	printf("-- MPU Example %s --\n\r", SOFTPACK_VERSION);
	printf("-- %s\n\r", BOARD_NAME);
	printf("-- Compiled: %s %s With %s--\n\r", __DATE__, __TIME__, COMPILER_NAME);

	LED_Configure(LED_YELLOW0);

	/* Set up the default memory regions */
	_SetupMPU();

	/* Set the environment to thread mode */
	__set_CONTROL(USER_MODE);

	while (ucChoice != '9') {
		printf("----------------------------------------\n\r");
		printf("    Choose an option below:\n\r");
		printf("    1. Protect the Yellow LED region\n\r");
		printf("    2. UN-protect the Yellow LED region\n\r");
		printf("    3. Toggle the green LED\n\r");
		printf("    4. Set the RAM region to read only \n\r");
		printf("    5. Set the RAM region to read/write\n\r");
		printf("    6. Read the RAM content at offset 0\n\r");
		printf("    7. Write the RAM context at offset 0\n\r");
		printf("    8. Quit the external program\n\r");
		printf("\n\r");
		printf("  Choice: ");

		ucChoice = DBG_GetChar();
		DBG_PutChar(ucChoice);
		printf("\n\r");

		switch (ucChoice) {
		case '1':
			_UpdateMPU(MPU_PIOC_REGION_REGION,
					MPU_PIOC_PERIPHERALS_REGION_START_ADDRESS |
					MPU_REGION_VALID |
					MPU_PIOC_REGION_REGION,
					MPU_AP_UNPRIVILEGED_READONLY |
					MPU_REGION_EXECUTE_NEVER |
					MPU_CalMPURegionSize(MPU_PIOC_PERIPHERALS_REGION_END_ADDRESS - MPU_PIOC_PERIPHERALS_REGION_START_ADDRESS) |
						MPU_REGION_ENABLE);
			break;

		case '2':
			_UpdateMPU(MPU_PIOC_REGION_REGION,
					MPU_PIOC_PERIPHERALS_REGION_START_ADDRESS |
					MPU_REGION_VALID |
					MPU_PIOC_REGION_REGION,
					MPU_AP_FULL_ACCESS |
					MPU_REGION_EXECUTE_NEVER |
					MPU_CalMPURegionSize(MPU_PIOC_PERIPHERALS_REGION_END_ADDRESS -
						MPU_PIOC_PERIPHERALS_REGION_START_ADDRESS) |
					MPU_REGION_ENABLE);
			break;

		case '3':
		/* Set back to unprivileged mode*/

			LED_Toggle(LED_YELLOW0);
			__set_CONTROL(USER_MODE);
			break;

		case '4':
			_UpdateMPU(MPU_PRIVILEGE_RAM_REGION,
					MPU_SRAM_PRIVILEGE_START_ADDRESS |
					MPU_REGION_VALID |
					MPU_PRIVILEGE_RAM_REGION,
					MPU_AP_UNPRIVILEGED_READONLY |
					MPU_REGION_CACHEABLE |
					MPU_REGION_BUFFERABLE |
					MPU_TEX_B001|
					MPU_CalMPURegionSize(MPU_SRAM_PRIVILEGE_END_ADDRESS -
					MPU_SRAM_PRIVILEGE_START_ADDRESS) |
					MPU_REGION_ENABLE);
			break;

		case '5':
			_UpdateMPU(MPU_PRIVILEGE_RAM_REGION,
					MPU_SRAM_PRIVILEGE_START_ADDRESS |
					MPU_REGION_VALID |
					MPU_PRIVILEGE_RAM_REGION,
					MPU_AP_FULL_ACCESS |
					MPU_REGION_CACHEABLE |
					MPU_REGION_BUFFERABLE |
					MPU_TEX_B001|
					MPU_CalMPURegionSize(MPU_SRAM_PRIVILEGE_END_ADDRESS -
					MPU_SRAM_PRIVILEGE_START_ADDRESS) |
					MPU_REGION_ENABLE);

			break;

		case '6':
			printf("-I- RAM address has content %x \n\r",(unsigned)(pdw[0]));
			break;

		case '7':
			printf("-I- Write offset 0 of RAM region with content 0x55AA55AA\n\r");
			pdw[0] = 0x55AA55AA;
			break;

		case '8':
			printf("-I- MPU TEST FINISH ...\n\r");
			break;

		default:
			printf("-I- This option is not valid\n\r");
			break;
		}
	}
	/* Setback to privilege mode, before entering the main routine,
	   it is supervisor mode*/
	__ASM volatile(" svc 0x00 ");
	while (!dwRaisePriDone);
	dwRaisePriDone = 0;

	__set_CONTROL(PRIVILEGE_MODE);

	return 0;
}
