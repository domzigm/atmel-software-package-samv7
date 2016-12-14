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

#include "conf_usb_host.h"
#include "board.h"


/* Default option */
static uint32_t usart_options = (US_MR_CHRL_8_BIT | US_MR_PAR_NO |
								  US_MR_NBSTOP_1_BIT | US_MR_CHMODE_NORMAL);

void USART_HANDLER(void)
{
	uint32_t sr = USART_GetStatus(USART_BASE);

	if (sr & US_CSR_RXRDY) {
		/* Data received */
		ui_com_tx_start();
		uint32_t value;

		if ((!USART_IsRxReady(USART_BASE))
			|| (sr & (US_CSR_FRAME | US_CSR_TIMEOUT | US_CSR_PARE))) {
			USART_ResetRx(USART_BASE);
			USART_EnableRx(USART_BASE);
			ui_com_error();
		} else
			value = USART_BASE->US_RHR;

		/* Transfer UART RX fifo to CDC TX */
		if (!uhi_cdc_is_tx_ready(0)) {
			/* Fifo full */
			ui_com_overflow();
		} else
			uhi_cdc_putc(0, value);

		ui_com_tx_stop();
		return;
	}

	if (sr & US_CSR_TXRDY) {
		/* Data ready to be sent */
		if (uhi_cdc_is_rx_ready(0)) {
			/* Transmit next data */
			ui_com_rx_start();
			int c = uhi_cdc_getc(0);
			USART_Write(USART_BASE, c, 0);
			//TRACE_INFO_WP("%c", c);
		} else {
			/* Fifo empty then Stop UART transmission */
			USART_DisableTx(USART_BASE);
			USART_DisableIt(USART_BASE, US_IDR_TXRDY);
			ui_com_rx_stop();
		}
	}
}

void uart_rx_notify(void)
{
	/* If UART is open */
	if (USART_GetItMask(USART_BASE)
		& US_IMR_RXRDY) {
		/* Enable UART TX interrupt to send a new value */
		USART_EnableTx(USART_BASE);
		USART_EnableIt(USART_BASE, US_IER_TXRDY);
	}
}

void uart_config(CDCLineCoding *cfg)
{
	uint32_t stopbits, parity, databits;
	uint32_t imr;

	switch (cfg->bCharFormat) {
	case CDCLineCoding_TWOSTOPBITS:
		stopbits = US_MR_NBSTOP_2_BIT;
		TRACE_INFO_WP("\n\rUS_MR_NBSTOP_2_BIT");
		break;

	case CDCLineCoding_ONE5STOPBIT:
		stopbits = US_MR_NBSTOP_1_5_BIT;
		TRACE_INFO_WP("\n\rUS_MR_NBSTOP_1_5_BIT");
		break;

	case CDCLineCoding_ONESTOPBIT:
	default:
		/* Default stop bit = 1 stop bit */
		stopbits = US_MR_NBSTOP_1_BIT;
		TRACE_INFO_WP("\n\rUS_MR_NBSTOP_1_BIT");
		break;
	}

	switch (cfg->bParityType) {
	case CDCLineCoding_EVENPARITY:
		parity = US_MR_PAR_EVEN;
		TRACE_INFO_WP("\n\rUS_MR_PAR_EVEN");
		break;

	case CDCLineCoding_ODDPARITY:
		parity = US_MR_PAR_ODD;
		TRACE_INFO_WP("\n\rUS_MR_PAR_ODD");
		break;

	case CDCLineCoding_MARKPARITY:
		parity = US_MR_PAR_MARK;
		TRACE_INFO_WP("\n\rUS_MR_PAR_MARK");
		break;

	case CDCLineCoding_SPACEPARITY:
		parity = US_MR_PAR_SPACE;
		TRACE_INFO_WP("\n\rUS_MR_PAR_SPACE");
		break;

	default:
	case CDCLineCoding_NOPARITY:
		parity = US_MR_PAR_NO;
		TRACE_INFO_WP("\n\rUS_MR_PAR_NO");
		break;
	}

	switch (cfg->bDataBits) {
	case 5: case 6: case 7:
		databits = cfg->bDataBits - 5;
		break;

	default:
	case 8:
		databits = US_MR_CHRL_8_BIT;
		TRACE_INFO_WP("\n\rUS_MR_CHRL_8_BIT");
		break;
	}

	/* Options for USART. */
	usart_options = (databits | parity | stopbits | US_MR_CHMODE_NORMAL);

	imr = USART_GetItMask(USART_BASE);
	USART_DisableIt(USART_BASE, 0xFFFFFFFF);
	TRACE_INFO_WP("\n\r115200 BAUDRATE");
	USART_Configure(USART_BASE, usart_options, cfg->dwDTERate, BOARD_MCK);
	/* Restore both RX and TX */
	USART_SetTransmitterEnabled(USART_BASE, true);
	USART_SetReceiverEnabled(USART_BASE, true);
	USART_EnableIt(USART_BASE, imr);
}

void uart_open(void)
{
	/* IO is initialized in board init
	 * Enable interrupt with priority higher than USB
	 */
	NVIC_SetPriority(USART_INT_IRQn, USART_INT_LEVEL);
	NVIC_EnableIRQ(USART_INT_IRQn);

	/* Initialize it in RS232 mode. */
	PMC_EnablePeripheral(USART_ID);
	USART_Configure(USART_BASE, usart_options, 115200, BOARD_MCK)

	/* Enable USART */
	USART_ENABLE();

	/* Enable both RX and TX */
	USART_SetTransmitterEnabled(USART_BASE, true);
	USART_SetReceiverEnabled(USART_BASE, true);
	/* Enable interrupts */
	USART_EnableIt(USART_BASE, US_IER_RXRDY | US_IER_TXRDY);
}

void uart_close(void)
{
	/* Disable interrupts */
	USART_DisableIt(USART_BASE, 0xFFFFFFFF);
	/* Close RS232 communication */
	USART_DISABLE();
}
