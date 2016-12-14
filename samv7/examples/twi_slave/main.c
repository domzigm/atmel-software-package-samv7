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
 * \page twi_slave TWI Slave Example
 *
 * \section Purpose
 *
 * This project demonstrates the TWI peripheral in slave mode. It mimics the
 * behavior of a serial memory, enabling the TWI master to read and write
 * data in its internal SRAM.
 *
 * \section Requirements
 *
 *  This package can be used with SAM V71 Xplained Ultra board or SAME70 Xplained board.
 *
 * \section Description
 *
 * After launching the program, the device will act as a simple TWI-enabled
 * serial memory containing 256 bytes. This enables this project to be used
 * with the twi_eeprom project as the master after modifying the slave address
 * AT24MAC_ADDRESS as 0x53 in main.c.
 *
 * To write in the memory, the TWI master must address the device first, then
 * send one byte containing the memory address to access. Additional bytes are
 * treated as the data to write.
 *
 * Reading is done in the same fashion, except that after receiving the memory
 * address, the device will start outputting data until a STOP condition is
 * sent by the master.
 *
 *  Please connect the two SAMV71/SAME70 boards with the following pins which could be
 *  easily wired from the board.
 *  - <b>SAMV71/SAME70  -- SAMV71/SAME70</b>
 *   - TWD0(PA03,PIN11 of EXT1 )          -- TWD0(PA03,PIN11 of EXT1 )
 *   - TWCK0(PA04,PIN12 of EXT1)          -- TWCK0(PA04,PIN12 of EXT1)
 *   - GND                                -- GND
 * \section Usage
 *
 * -# Build the program and download it inside the board.
 *     Please refer to the Getting Started with SAM V71/E70 Microcontrollers.pdf
 * -# On the computer, open and configure a terminal application
 *    (e.g. HyperTerminal on Microsoft Windows) with these settings:
 *   - 115200 baud rate
 *   - 8 bits of data
 *   - No parity
 *   - 1 stop bit
 *   - No flow control
 * -# Start the application.
 * -# In the terminal window, the following text should appear (values depend
*  on the board and chip used):
 *    \code
 *     -- TWI Slave Example xxx --
 *     -- SAMxxxxx-xx
 *     -- Compiled: xxx xx xxxx xx:xx:xx --
 *     -I- Configuring the TWI in slave mode
 *    \endcode
 * -# For the TWI Master board, see the description inside his project
 * -# and the "Master" board will output:
 *    \code
 *     -- TWI EEPROM Example xxx --
 *     -- SAMxxxxx-xx
 *     -- Compiled: xxx xx xxxx xx:xx:xx --
 *     -I- Filling page #0 with zeroes ...
 *     -I- Filling page #1 with zeroes ...
 *     ......
 *     -I- 0 comparison error(s) found
 *     ......
 *     -I- 0 comparison error(s) found
 *    \endcode
 *
 * \section References
 * - twi_slave/main.c
 * - twi_eeprom/main.c
 * - twi.c
 * - twid.h
 */

/**
 * \file
 *
 * This file contains all the specific code for the twi_slave example.
 */

/*----------------------------------------------------------------------------
 *        Headers
 *----------------------------------------------------------------------------*/

#include "board.h"

#include <stdio.h>
#include <string.h>

/*----------------------------------------------------------------------------
 *        Local definitions
 *----------------------------------------------------------------------------*/
/** Slave address of the device on the TWI bus. */
#define SLAVE_ADDRESS       0x53

/** Memory size in bytes (example AT24C512)*/
#define MEMORY_SIZE         256

/** Eeprom Pins definition */
#define BOARD_PINS_TWI_SLAVE PINS_TWI0
/** TWI0 peripheral ID for eeprom device*/
#define BOARD_ID_TWI_EEPROM  ID_TWIHS0
/** TWI0 base address for eeprom device */
#define BOARD_BASE_TWI_SLAVE TWIHS0

/*----------------------------------------------------------------------------
 *        Local variables
 *----------------------------------------------------------------------------*/
/** Pio pins to configure. */
const Pin pins[] = BOARD_PINS_TWI_SLAVE;

/** The slave device instance*/
typedef struct _SlaveDeviceDriver
{
	/** PageAddress of the slave device*/
	uint16_t pageAddress;
	/** Offset of the memory access*/
	uint16_t offsetMemory;
	/** Read address of the request*/
	uint8_t acquireAddress;
	/** Memory buffer*/
	uint8_t pMemory[MEMORY_SIZE];
} SlaveDeviceDriver;

static SlaveDeviceDriver EmulateDriver;

/*----------------------------------------------------------------------------
 *        Local functions
 *----------------------------------------------------------------------------*/
/**
 * \brief Interrupt handler for the TWI slave.
 */
/**
 * \brief TWI interrupt handler. Forwards the interrupt to the TWI driver handler.
 */
void TWIHS0_Handler( void )
{
	volatile uint32_t status;

	status = TWI_GetStatus( BOARD_BASE_TWI_SLAVE);
	if (((status & TWIHS_SR_SVACC) == TWIHS_SR_SVACC)) {
		if (EmulateDriver.acquireAddress == 0) {
			TWI_DisableIt(BOARD_BASE_TWI_SLAVE, TWIHS_IDR_SVACC);
			TWI_EnableIt(BOARD_BASE_TWI_SLAVE, TWIHS_IER_RXRDY
						| TWIHS_IER_EOSACC
						| TWIHS_IER_SCL_WS );
			EmulateDriver.acquireAddress++;
			EmulateDriver.pageAddress = 0;
			EmulateDriver.offsetMemory = 0;
		}

		if ((status & TWIHS_SR_GACC) == TWIHS_SR_GACC) {
			printf("General Call Treatment\n\r");
			printf("not treated");
		} else {
			if ((status & TWIHS_SR_SVREAD) == TWIHS_SR_SVREAD) {
				/*Slave Read */
				if (((status & TWIHS_SR_TXRDY) == TWIHS_SR_TXRDY)
					&& ((status & TWIHS_SR_NACK) == 0)) {
					if ((EmulateDriver.acquireAddress == 2)) {
						/* Write one byte of data from slave to master device */
						TWI_WriteByte( BOARD_BASE_TWI_SLAVE,
								EmulateDriver.pMemory[EmulateDriver.pageAddress
								+ EmulateDriver.offsetMemory]);
						EmulateDriver.offsetMemory++;
					}
				}
			} else {
				/*Slave Write*/
				if ((status & TWIHS_SR_RXRDY) == TWIHS_SR_RXRDY) {
					if (EmulateDriver.acquireAddress == 1) {
						/* Acquire MSB address */
						EmulateDriver.pageAddress = (TWI_ReadByte(BOARD_BASE_TWI_SLAVE) & 0xFF);
						EmulateDriver.acquireAddress++;
					} else {
						/* Read one byte of data from master to slave device */
						EmulateDriver.pMemory[EmulateDriver.pageAddress + EmulateDriver.offsetMemory]
								= (TWI_ReadByte(BOARD_BASE_TWI_SLAVE) & 0xFF);
						EmulateDriver.offsetMemory++;
					}
				}
			}
		}
	} else {
		if ((status & TWIHS_SR_EOSACC) == TWIHS_SR_EOSACC) {
			if ((status & TWIHS_SR_TXCOMP) == TWIHS_SR_TXCOMP) {
				/* End of transfer, end of slave access */
				EmulateDriver.offsetMemory = 0;
				EmulateDriver.acquireAddress = 0;
				EmulateDriver.pageAddress = 0;
				TWI_EnableIt( BOARD_BASE_TWI_SLAVE, TWIHS_SR_SVACC);
				TWI_DisableIt( BOARD_BASE_TWI_SLAVE, TWIHS_IDR_RXRDY
											| TWIHS_IDR_EOSACC
											| TWIHS_IDR_SCL_WS );
			}
		}
	}
}

/*----------------------------------------------------------------------------
 *        Global functions
 *----------------------------------------------------------------------------*/

/**
 * \brief Application entry point for TWI slave example.
 *
 * \return Unused (ANSI-C compatibility).
 */
int main(void)
{
	uint32_t i;

 /* Disable watchdog */
	WDT_Disable(WDT);

	/* Enable I and D cache */
	SCB_EnableICache();
	SCB_EnableDCache();

	PIO_Configure(pins, PIO_LISTSIZE(pins));

	printf("-- TWI Slave Example %s --\n\r", SOFTPACK_VERSION);
	printf("-- %s\n\r", BOARD_NAME);
	printf("-- Compiled: %s %s --\n\r", __DATE__, __TIME__);

	/* Enable TWI peripheral clock */
	PMC_EnablePeripheral(BOARD_ID_TWI_EEPROM);

	for (i = 0; i < MEMORY_SIZE; i++)
		EmulateDriver.pMemory[i] = 0;

	EmulateDriver.offsetMemory = 0;
	EmulateDriver.acquireAddress = 0;
	EmulateDriver.pageAddress = 0;

	/* Configure TWI as slave */
	printf( "-I- Configuring the TWI in slave mode\n\r" );
	TWI_ConfigureSlave(BOARD_BASE_TWI_SLAVE, SLAVE_ADDRESS);

	/* Clear receipt buffer */
	TWI_ReadByte(BOARD_BASE_TWI_SLAVE);

	printf("TWI is in slave mode\n\r");

	/* Configure TWI interrupts */
	NVIC_ClearPendingIRQ(TWIHS0_IRQn);
	NVIC_EnableIRQ(TWIHS0_IRQn);

	TWI_EnableIt(BOARD_BASE_TWI_SLAVE, TWIHS_SR_SVACC);

	while (1) {
	}
}


