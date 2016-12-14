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
 *  \page usart_spi USART SPI example with DMA
 *
 *  This example demonstrates the SPI mode provided by the USART peripherals on
 *  SAMV7/E7 Microcontrollers.
 *
 *  \section Requirements
 *
 *  This package can be used with SAMV71 Xplained Ultra board or SAME70 Xplained board.
 *  Please connect the USART and SPI pins on one board as following matching table:
 *  - <b> USART0 -- SPI0 </b> (for USART0 as SPI master)
 *   - SCK0(PB13 pin05 on J504)            - SCK  (SCK on J506)
 *   - TXD0(PB01 pin14 on EXT1)            - MOSI (MOSI on J506)
 *   - RXD0(PB00 pin13 on EXT1)            - MISO (MISO on J506)
 *   - RTS0(PB03 PIN05 on EXT1)            - NSS  (PB02 pin06 on EXT1)
 *
 *  - <b> USART0 -- SPI0 </b> (for USART0 as SPI slave)
 *   - SCK0(PB13 pin05 on J504)            - SCK  (SCK on J506)
 *   - RXD0(PB00 pin13 on EXT1)            - MOSI (MOSI on J506)
 *   - TXD0(PB01 pin14 on EXT1)            - MISO (MISO on J506)
 *   - CTS0(PB02 PIN06 on EXT1)            - NPCS1(PD25 pin15 on EXT1)
 *
 *  \section Description
 *
 * This example demonstrates how to use USART in SPI mode. The USART is
 * configured as SPI master and slave. Meanwhile, the SPI peripheral in the
 * Microcontroller is configured respectively, making it to communicate with the
 * USART peripheral.
 *
 * The application first initializes DBGU as the interface to interact with
 * users.
 * The application waits for input from DBGU:
 *
 * Menu :
 * ------
 *  - M: Configure USART as spi master
 *  - S: Configure USART as spi slave
 *  - H: Display this menu
 * \section Usage
 *
 *  -# Build the program and download it inside the board.
 *  Please refer to the Getting Started with SAM V71/E70 Microcontrollers.pdf
 *  -# Connect a serial cable to the DBGU port on the evaluation kit.
 *  -# On the computer, open and configure a terminal application (e.g.
 *     HyperTerminal on Microsoft Windows) with these settings:
 *        - 115200 baud rate
 *        - 8 bits of data
 *        - No parity
 *        - 1 stop bit
 *        - No flow control
 *  -# Start the application. The following traces shall appear on the terminal:
 *     \code
 *     -- USART SPI Example xxx --
 *     -- SAMxxxxx-xx
 *     -- Compiled: xxx xx xxxx xx:xx:xx --
 *     \endcode
 *
 *  \section References
 *  - usart_spi/main.c
 *  - pio.h
 *  - usart.h
 */

/*----------------------------------------------------------------------------
 *        Headers
 *----------------------------------------------------------------------------*/

#include "board.h"

#include <stdbool.h>
#include <stdio.h>
#include <string.h>

/*----------------------------------------------------------------------------
 *        Local definitions
 *----------------------------------------------------------------------------*/

/** Size of the receive buffer in bytes.*/

/** Pins for USART */
#define PINS_USART_MASTER PIN_USART0_TXD, PIN_USART0_RXD, PIN_USART0_SCK, PIN_USART0_RTS
#define PINS_USART_SLAVE  PIN_USART0_TXD, PIN_USART0_RXD, PIN_USART0_SCK, PIN_USART0_CTS

/** Pins for SPI */
#define PINS_SPI0_MASTER  PIN_SPI_MISO, PIN_SPI_MOSI, PIN_SPI_SPCK,  PIN_SPI_NPCS1
#define PINS_SPI0_SLAVE   PIN_SPI_MISO, PIN_SPI_MOSI, PIN_SPI_SPCK,  PIN_SPI_NPCS0

/** Register base for USART */
#define USART             USART0
#define ID_USART          ID_USART0
#define USART_IRQn        USART0_IRQn
#define USART_Handler     USART0_Handler

/** Register base for SPI */
#define SPI               SPI0
#define ID_SPI            ID_SPI0
#define SPI_IRQn          SPI0_IRQn
#define SPI_Handler       SPI0_Handler


/*----------------------------------------------------------------------------
 *        Local variables
 *----------------------------------------------------------------------------*/

char pTxBuffer1[] = {"ABCDEFGHIJKLMNOPQRSTUVWXYZ\n\r"};
char pTxBuffer2[] = {"abcdefghijklmnopqrstuvwxyz\n\r"};

/**  Pins to configure for the application.*/
const Pin pins1[] = {PINS_USART_MASTER, PINS_SPI0_SLAVE};
const Pin pins2[] = {PINS_USART_SLAVE, PINS_SPI0_MASTER};

/** Clock -- SPI as master */
static uint32_t spiClock = 5000000;

/** Clock -- usart as SPI master */
static uint32_t baudRate = 5000000;
/*----------------------------------------------------------------------------
 *        Local functions
 *----------------------------------------------------------------------------*/

/**
 * \brief SPI handler
 */
void SPI_Handler(void)
{
	SPI_GetStatus(SPI);
	printf("%c", (char) SPI_Read(SPI));
}

/**
 * \brief USART handler
 */
void USART_Handler(void)
{
	volatile uint32_t status;
	status = USART_GetStatus(USART);
	if ((status & 0x1) == 1)
		printf("%c", USART_Read(USART, 0));
}

/**
 * \brief Configures an USART baudrate when USART_MODE=SPI.
 *
 *
 *  \param pUsart  Pointer to the USART peripheral to configure.
 *  \param baudrate  Baudrate at which the USART should operate (in Hz).
 *  \param masterClock  Frequency of the system master clock (in Hz).
 */
static void USART_SPI_SetBaudrate(Usart *pUsart,
								uint32_t baudrate,
								uint32_t masterClock)
{
	unsigned int CD, FP;

	/* Configure baudrate*/
	CD = (masterClock / baudrate);
	FP = ((masterClock / baudrate) - CD);

	pUsart->US_BRGR = (US_BRGR_CD(CD) | US_BRGR_FP(FP));
}

/**
 * \brief Configures an USART peripheral with the specified parameters.
 *
 *
 *  \param pUsart  Pointer to the USART peripheral to configure.
 *  \param mode  Desired value for the USART mode register (see the datasheet).
 *  \param baudrate  Baudrate at which the USART should operate (in Hz).
 *  \param masterClock  Frequency of the system master clock (in Hz).
 */
static void USART_SPI_Configure(Usart *pUsart,
		uint32_t mode,
		uint32_t baudrate,
		uint32_t masterClock)
{

	/* Reset and disable receiver & transmitter*/
	pUsart->US_CR = US_CR_RSTRX | US_CR_RSTTX
		| US_CR_RXDIS | US_CR_TXDIS | US_CR_RSTSTA;

	pUsart->US_IDR = 0xFFFFFFFF;

	/* Configure baudrate*/
	USART_SPI_SetBaudrate(pUsart, baudrate, masterClock);

	/* Configure mode*/
	pUsart->US_MR = mode;

	/* Enable receiver and transmitter*/
	pUsart->US_CR = US_CR_RXEN | US_CR_TXEN;


	/* Disable buffering for printf(). */
#if ( defined (__GNUC__) && !defined (__SAMBA__) )
	setvbuf(stdout, (char *)NULL, _IONBF, 0);
#endif

}

/**
 * \brief Configures spi in slave mode.
 */
static void _ConfigureSpiSlave( void )
{
	/* Configure SPI slave mode */
	SPI_Configure(SPI, ID_SPI, SPI_PCS(0));

	NVIC_ClearPendingIRQ(SPI_IRQn);
	NVIC_SetPriority(SPI_IRQn ,1);
	NVIC_EnableIRQ(SPI_IRQn);
	SPI_DisableIt(SPI, 0xffffffff);

	SPI_ConfigureNPCS(SPI, 0, 0);
}

/**
 * \brief Configures USART in spi master mode
 */
static void _ConfigureUsartAsSpiMaster(void)
{
	uint32_t usartMode;

	/* Configure usart master mode */
	usartMode = 0
		| US_MR_USART_MODE_SPI_MASTER
		| US_MR_USCLKS_MCK
		| US_MR_CHRL_8_BIT
		| US_SPI_BPMODE_1
		| US_MR_CLKO;

	PMC_EnablePeripheral(ID_USART);
	USART_SPI_Configure(USART, usartMode, baudRate, BOARD_MCK);

	NVIC_ClearPendingIRQ(USART_IRQn);
	NVIC_SetPriority(USART_IRQn ,1);
	NVIC_EnableIRQ(USART_IRQn);
	USART_DisableIt(USART, 0xffffffff);
}

/**
 * \brief Configures spi in master mode.
 */
static void _ConfigureSpiMaster(void)
{
	/* Configure SPI master mode */
	SPI_Configure(SPI, ID_SPI, (SPI_MR_MSTR | SPI_MR_MODFDIS | SPI_PCS(1)));

	NVIC_ClearPendingIRQ(SPI_IRQn);
	NVIC_SetPriority(SPI_IRQn ,1);
	NVIC_EnableIRQ(SPI_IRQn);
	SPI_DisableIt(SPI, 0xffffffff);

	SPI_ConfigureNPCS(SPI, 1,
					  SPI_DLYBCT( 100, BOARD_MCK ) |
					  SPI_DLYBS(100, BOARD_MCK) |
					  SPI_SCBR( spiClock, BOARD_MCK) |
					  SPI_CSR_BITS_8_BIT);
}

/**
 * \brief Configures USART in spi slave mode
 */
static void _ConfigureUsartAsSpiSlave(void)
{
	uint32_t usartMode;

	/* Configure usart slave mode */
	usartMode = 0
		| US_MR_USART_MODE_SPI_SLAVE
		| US_MR_CHRL_8_BIT
		| US_SPI_BPMODE_1;
	PMC_EnablePeripheral(ID_USART);
	USART_SPI_Configure(USART, usartMode, spiClock, BOARD_MCK);

	NVIC_ClearPendingIRQ(USART_IRQn);
	NVIC_SetPriority(USART_IRQn ,1);
	NVIC_EnableIRQ(USART_IRQn);
	USART_DisableIt(USART, 0xffffffff);
}

/**
 * \brief Display main menu.
 */
static void _DisplayMainmenu(void)
{
	printf("\n\rMenu :\n\r");
	printf("------\n\r");
	printf(" - M: Configure USART as spi master\n\r");
	printf(" - S: Configure USART as spi slave\n\r");
	printf(" - H: Display this menu \n\r");
}

/*----------------------------------------------------------------------------
 *        Exported functions
 *----------------------------------------------------------------------------*/
/**
 *  \brief usart_spi Application entry point.
 *
 *  \return Unused (ANSI-C compatibility).
 */
extern int main(void)
{
	uint8_t ucKey, i;

	/* Disable watchdog */
	WDT_Disable(WDT);

	SCB_EnableICache();
	SCB_EnableDCache();

	/* Configure systick for 1 ms. */
	TimeTick_Configure();

	/* Output example information */
	printf("-- USART SPI Example %s --\n\r", SOFTPACK_VERSION);
	printf("-- %s\n\r", BOARD_NAME);
	printf("-- Compiled: %s %s  With %s--\n\r", __DATE__, __TIME__, COMPILER_NAME);

	/* Display menu */
	_DisplayMainmenu();

	while (1) {
		ucKey = DBG_GetChar();
		switch (ucKey) {
			/*usart as spi master*/
		case 'm':
		case 'M':
			/* Configure pins*/
			PIO_Configure(pins1, PIO_LISTSIZE(pins1));
			/* Configure USART as SPI master */
			_ConfigureUsartAsSpiMaster();

			/* Configure SPi slave */
			_ConfigureSpiSlave();
			printf("-I- Configure USART as spi master ...\n\r");
			SPI_EnableIt(SPI, SPI_IER_RDRF);
			SPI_Enable(SPI);

			USART_EnableIt(USART, UART_IER_RXRDY);

			for (i = 0; (pTxBuffer1[i]!='\0' && pTxBuffer2[i]!='\0'); i++) {
				while ((SPI->SPI_SR & SPI_SR_TXEMPTY) == 0);
				SPI->SPI_TDR = ((uint16_t)pTxBuffer2[i]) | SPI_PCS( 0 );
				USART_Write( USART, pTxBuffer1[i], 0);
			}
			break;

			/*usart as spi slave*/
		case 's':
		case 'S':
			printf("-I- Configure USART as spi slave...\n\r");
			/* Configure pins*/
			PIO_Configure(pins2, PIO_LISTSIZE(pins2));
			/* Configure USART as SPI slave */
			_ConfigureUsartAsSpiSlave();
			/* Configure SPI master */
			_ConfigureSpiMaster();
			USART_EnableIt(USART, UART_IER_RXRDY);
			SPI_EnableIt(SPI, SPI_IER_RDRF);
			SPI_Enable(SPI);
			for (i = 0; (pTxBuffer1[i]!='\0' && pTxBuffer2[i]!='\0'); i++) {
				USART_Write(USART, (uint16_t)pTxBuffer2[i], 0);
				SPI_Write( SPI, 1, (uint16_t)pTxBuffer1[i]);
			}
			break;

		case 'h':
		case 'H':
			_DisplayMainmenu();
			break;
		}
	}
}
/** \endcond */
