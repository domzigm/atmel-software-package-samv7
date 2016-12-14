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
 *  \page aes Advanced Encryption Standard Example
 *
 *  \section Purpose
 *  This application demonstrates the Advanced Encryption Standard (AES)
 *  peripheral integrated in some of SAMV7 micro-controllers family. It encrypts
 *  and decrypts several test values in Electronic CodeBook (ECB) and Cipher Block
 *  Chaining (CBC),OBC,OFB,TRC modes and checks them against the known answers.
 *
 * \section Requirements
 *
 *  This package can be used with SAMV71 Xplained Ultra board or SAME70 Xplained board.
 *
 *  \section Description
 * This example shows how to configure AES in encryption and decryption mode.
 * In encryption mode, it encrypts plain text with one of ECB, CBC, OFB ,CFB and
 * CTR mode. Programmable key mode with processing using with or without DMA
 * support.
 * In decryption mode, it decrypts cipher data generate from encryption mode and
 * get the known plain value.
 *
 *  \section Usage
 *
 *  -# Build the program and download it inside the board.
 *     Please refer to the Getting Started with SAM V71/E70 Microcontrollers.pdf
 *  -# On the computer, open and configure a terminal application
 *     (e.g. HyperTerminal on Microsoft Windows) with these settings:
 *    - 115200 baud rate
 *    - 8 bits of data
 *    - No parity
 *    - 1 stop bit
 *    - No flow control
 *  -# In the terminal window, the
 *     following text should appear (values depend on the board and chip used):
 *     \code
 *      -- AES Example xxx --
 *      -- SAMxxxxx-xx
 *      -- Compiled: xxx xx xxxx xx:xx:xx --
 *      -- Menu Choices for this example--
 *      \endcode
 *  -# Input command according to the menu.
 *
 *  \section References
 *  - aes/main.c
 *  - aes.c
 *  - aes.h */

/** \file
 *
 *  This file contains all the specific code for the AES
 */

/*----------------------------------------------------------------------------
 *        Headers
 *----------------------------------------------------------------------------*/
#include <board.h>
#include <string.h>
/*----------------------------------------------------------------------------
 *        Local definitions
 *----------------------------------------------------------------------------*/
#define DATA_LEN_INBYTE   640
#define DATA_LEN_INWORD (DATA_LEN_INBYTE/4)
#define DATA_LEN_INDWORD (DATA_LEN_INBYTE/8)

#define AES_VECTOR_0     0x11223344
#define AES_VECTOR_1     0x55667788
#define AES_VECTOR_2     0x11112222
#define AES_VECTOR_3     0x33334444

#define AES_KEY_0       0x01234567
#define AES_KEY_1       0x89ABCDEF
#define AES_KEY_2       0x76543210
#define AES_KEY_3       0xFEDCBA98
#define AES_KEY_4       0x55AA55AA
#define AES_KEY_5       0xAA55AA55
#define AES_KEY_6       0x0000FFFF
#define AES_KEY_7       0xFFFF0000

/*----------------------------------------------------------------------------
 *        Local variables
 *----------------------------------------------------------------------------*/
const uint32_t aes_keys[8] = {AES_KEY_0, AES_KEY_1, AES_KEY_2, AES_KEY_3,
								AES_KEY_4, AES_KEY_5, AES_KEY_6, AES_KEY_7
								};
const uint32_t aes_vectors[4] = { AES_VECTOR_0,
								AES_VECTOR_1,
								AES_VECTOR_2,
								AES_VECTOR_3
								};

COMPILER_ALIGNED(32) static uint32_t bufPlaint[DATA_LEN_INWORD];
COMPILER_ALIGNED(32) static uint32_t bufCipher[DATA_LEN_INWORD];
COMPILER_ALIGNED(32) static uint32_t bufOut[DATA_LEN_INWORD];

const char plaintext[DATA_LEN_INBYTE] = "\
  The Advanced Encryption Standard (AES) is compliant with the A\
merican FIPS (Federal Information Processing Standard) Publicati\
on 197 specification. AES supports all five confidentiality mode\
s of operation for symmetrical key block cipher algorithms (ECB,\
CBC,OFB, CFB and CTR), as specified in the NIST Special Publicat\
ion 80038A. It is compatible with all these modes via Peripheral\
 DMA Controller channels, minimizing processor intervention for \
large buffer transfers.The 128-bit/192-bit/256-bit key is stored\
in four/six/eight 32-bit registers (AES_KEYWRx) which are all wr\
ite-only .......................................................";

static uint32_t operationMode, startMode, keyMode, keylength;
static uint32_t desDone;
/** Global DMA driver instance for all DMA transfers in application. */
static sXdmad xdmad;
static sXdmadCfg xdmadCfg;
static uint32_t dmaWriteChannel, dmaReadChannel;
COMPILER_ALIGNED(32) static LinkedListDescriporView1
dmaWriteLinkList[DATA_LEN_INWORD];
COMPILER_ALIGNED(32) static LinkedListDescriporView1
dmaReadLinkList[DATA_LEN_INWORD];

/*----------------------------------------------------------------------------
 *        Local functions
 *----------------------------------------------------------------------------*/
/**
 * \brief Display main menu.
 */
static void _displayMenu(void)
{
	uint8_t ucChar[5];
	printf("\n\rAES Menu :\n\r");
	printf("Press [0|1|2|3|4] to set Operation Mode \n\r");
	ucChar[0] = (operationMode == AES_MR_OPMOD_ECB) ? 'X' : ' ';
	ucChar[1] = (operationMode == AES_MR_OPMOD_CBC) ? 'X' : ' ';
	ucChar[2] = (operationMode == AES_MR_OPMOD_OFB) ? 'X' : ' ';
	ucChar[3] = (operationMode == AES_MR_OPMOD_CFB) ? 'X' : ' ';
	ucChar[4] = (operationMode == AES_MR_OPMOD_CTR) ? 'X' : ' ';
	printf("   0: ECB[%c] 1: CBC[%c] 2: OFB[%c] 3: CFB[%c] 4: CTR[%c] \n\r",
			ucChar[0], ucChar[1], ucChar[2], ucChar[3], ucChar[4]);
	printf("Press [5|6|7| set key size\n\r");
	ucChar[0] = (keyMode == AES_MR_KEYSIZE_AES128) ? 'X' : ' ';
	ucChar[1] = (keyMode == AES_MR_KEYSIZE_AES192) ? 'X' : ' ';
	ucChar[2] = (keyMode == AES_MR_KEYSIZE_AES256) ? 'X' : ' ';
	printf("    5: key 128-bits[%c] 6: key 192-bits[%c] 7: key 256-bits[%c]\n\r",
			ucChar[0], ucChar[1], ucChar[2]);
	printf("Press [m|a|d] to set Start Mode \n\r");
	ucChar[0] = (startMode == AES_MR_SMOD_MANUAL_START) ? 'X' : ' ';
	ucChar[1] = (startMode == AES_MR_SMOD_AUTO_START) ? 'X' : ' ';
	ucChar[2] = (startMode == AES_MR_SMOD_IDATAR0_START) ? 'X' : ' ';
	printf("   m: MANUAL_START[%c] a: AUTO_START[%c] d: DMA[%c]\n\r",
			ucChar[0], ucChar[1], ucChar[2]);
	printf("   p: Begin the encryption/decryption process \n\r");
	printf("   h: Display this menu\n\r");
	printf("\n\r");
}

/**
 * \brief xDMA handler.
 */
void XDMAC_Handler(void)
{
	XDMAD_Handler(&xdmad);
}

/**
 * \brief xDMA initialization.
 */

static void _xdma_init(void)
{
	/* Initialize XDMA driver instance with polling mode */
	XDMAD_Initialize(&xdmad, 1);

	/* Allocate a XDMA channel, Write accesses into AES_IDATARx */
	dmaWriteChannel = XDMAD_AllocateChannel(&xdmad, XDMAD_TRANSFER_MEMORY, ID_AES);

	if (dmaWriteChannel == XDMAD_ALLOC_FAILED)
		printf("-E- Can't allocate XDMA channel\n\r");

	XDMAD_PrepareChannel(&xdmad, dmaWriteChannel );

	/* Allocate a XDMA channel, Read accesses into AES_ODATARx */
	dmaReadChannel = XDMAD_AllocateChannel(&xdmad, ID_AES, XDMAD_TRANSFER_MEMORY);

	if (dmaReadChannel == XDMAD_ALLOC_FAILED)
		printf("-E- Can't allocate XDMA channel\n\r");

	XDMAD_PrepareChannel(&xdmad, dmaReadChannel );
}

/**
 * \brief Configure xDMA write linker list for AES transfer.
 */
static void _xdma_configure_write(uint32_t *buf, uint32_t len)
{
	uint32_t i;
	uint32_t xdmaCndc;

	for (i = 0; i < len; i++) {
		dmaWriteLinkList[i].mbr_ubc = XDMA_UBC_NVIEW_NDV1
										| ((i == len - 1) ? 0 : XDMA_UBC_NDE_FETCH_EN)
										| XDMA_UBC_NDEN_UPDATED
										| 4;
		dmaWriteLinkList[i].mbr_sa = (uint32_t)&buf[i * 4];
		dmaWriteLinkList[i].mbr_da = (uint32_t)&(AES->AES_IDATAR[0]);

		if (i == len - 1) dmaWriteLinkList[i].mbr_nda = 0;
		else dmaWriteLinkList[i].mbr_nda = (uint32_t)&dmaWriteLinkList[ i + 1 ];
	}

	xdmadCfg.mbr_cfg = XDMAC_CC_TYPE_PER_TRAN
						| XDMAC_CC_MBSIZE_SINGLE
						| XDMAC_CC_DSYNC_MEM2PER
						| XDMAC_CC_CSIZE_CHK_4
						| XDMAC_CC_DWIDTH_WORD
						| XDMAC_CC_SIF_AHB_IF1
						| XDMAC_CC_DIF_AHB_IF1
						| XDMAC_CC_SAM_INCREMENTED_AM
						| XDMAC_CC_DAM_FIXED_AM
						| XDMAC_CC_PERID(XDMAIF_Get_ChannelNumber(ID_AES, XDMAD_TRANSFER_TX));

	xdmaCndc = XDMAC_CNDC_NDVIEW_NDV1
				| XDMAC_CNDC_NDE_DSCR_FETCH_EN
				| XDMAC_CNDC_NDSUP_SRC_PARAMS_UPDATED
				| XDMAC_CNDC_NDDUP_DST_PARAMS_UPDATED;

	SCB_CleanDCache_by_Addr((uint32_t *)dmaWriteLinkList, sizeof(dmaWriteLinkList));

	XDMAD_ConfigureTransfer(&xdmad, dmaWriteChannel, &xdmadCfg, xdmaCndc,
								(uint32_t)&dmaWriteLinkList[0], XDMAC_CIE_LIE);
}

/**
 * \brief Configure xDMA read linker list for AES transfer.
 */
static void _xdma_configure_read(uint32_t *buf, uint32_t len)
{
	uint32_t i;
	uint32_t xdmaCndc;

	for (i = 0; i < len; i++) {
		dmaReadLinkList[i].mbr_ubc = XDMA_UBC_NVIEW_NDV1
									| ((i == len - 1) ? 0 : XDMA_UBC_NDE_FETCH_EN)
									| XDMA_UBC_NDEN_UPDATED
									| 4;
		dmaReadLinkList[i].mbr_sa  = (uint32_t) & (AES->AES_ODATAR[0]);
		dmaReadLinkList[i].mbr_da = (uint32_t)&buf[i * 4];

		if (i == len - 1) dmaReadLinkList[i].mbr_nda = 0;
		else
			dmaReadLinkList[i].mbr_nda = (uint32_t)&dmaReadLinkList[ i + 1 ];
	}

	xdmadCfg.mbr_cfg = XDMAC_CC_TYPE_PER_TRAN
						| XDMAC_CC_MBSIZE_SINGLE
						| XDMAC_CC_DSYNC_PER2MEM
						| XDMAC_CC_CSIZE_CHK_4
						| XDMAC_CC_DWIDTH_WORD
						| XDMAC_CC_SIF_AHB_IF1
						| XDMAC_CC_DIF_AHB_IF1
						| XDMAC_CC_SAM_FIXED_AM
						| XDMAC_CC_DAM_INCREMENTED_AM
						| XDMAC_CC_PERID(XDMAIF_Get_ChannelNumber(ID_AES, XDMAD_TRANSFER_RX));
	xdmaCndc = XDMAC_CNDC_NDVIEW_NDV1
				| XDMAC_CNDC_NDE_DSCR_FETCH_EN
				| XDMAC_CNDC_NDSUP_SRC_PARAMS_UPDATED
				| XDMAC_CNDC_NDDUP_DST_PARAMS_UPDATED;

	SCB_CleanDCache_by_Addr((uint32_t *)dmaReadLinkList, sizeof(dmaReadLinkList));
	XDMAD_ConfigureTransfer(&xdmad, dmaReadChannel, &xdmadCfg, xdmaCndc,
								(uint32_t)&dmaReadLinkList[0], XDMAC_CIE_LIE);
}

/**
 * \brief AES interrupt hander.
 */
void AES_Handler(void)
{
	if ((AES_GetStatus() & AES_ISR_DATRDY) == AES_ISR_DATRDY) {
		/* Disable AES interrupt */
		AES_DisableIt(AES_IER_DATRDY);
		desDone = 1;
	}
}

static void _startAES(void)
{
	char *p;
	uint32_t i;
	uint32_t loop = DATA_LEN_INWORD;
	/* A software triggered hardware reset of the AES interface is performed */
	AES_SoftReset();
	/* Configure and enable interrupt on RC compare */
	NVIC_ClearPendingIRQ(AES_IRQn);
	NVIC_EnableIRQ(AES_IRQn);

	if (startMode == AES_MR_SMOD_IDATAR0_START) {
		_xdma_init();
		loop = 1;
	}

	memcpy((char *)bufPlaint, plaintext, DATA_LEN_INBYTE);

	for (i = 0; i < DATA_LEN_INWORD; i++) {
		bufCipher[i] = 0xffffffff;
		bufOut[i]  = 0xffffffff;
	}

	for (i = 0; i < loop; i += 4) {
		/* Encrypts data */
		desDone = 0;
		/* Enable AES interrupt */
		AES_EnableIt(AES_IER_DATRDY);

		AES_Configure(AES_MR_CIPHER_ENCRYPT
						| keyMode
						| startMode
						| operationMode
						);
		/* Write the 128-bit/192-bit/256-bit key(s) in the Key Registers
				(AES_KEYxWRx)*/
		AES_WriteKey(aes_keys, keylength);

		/* The Initialization Vector Registers concern all modes except ECB. */
		if (operationMode != AES_MR_OPMOD_ECB)
			AES_SetVector(aes_vectors);

		if (startMode != AES_MR_SMOD_IDATAR0_START) {
			/* Write the data to be encrypted in the authorized Input Data
				Registers */
			AES_SetInput(&bufPlaint[i]);

			if (startMode == AES_MR_SMOD_MANUAL_START)
				/* Set the START bit in the AES Control register AES_CR to
					begin the encryption process. */
				AES_Start();

			while (!desDone);

			AES_GetOutput(&bufCipher[i]);
		} else {
			AES_SetDataLen(DATA_LEN_INBYTE);
			_xdma_configure_write(bufPlaint, DATA_LEN_INWORD / 4);
			_xdma_configure_read(bufCipher, DATA_LEN_INWORD / 4);
			SCB_CleanDCache_by_Addr((uint32_t *)bufPlaint, DATA_LEN_INBYTE);
			XDMAD_StartTransfer(&xdmad, dmaWriteChannel);
			XDMAD_StartTransfer(&xdmad, dmaReadChannel );

			while (XDMAD_IsTransferDone(&xdmad, dmaReadChannel));

			SCB_InvalidateDCache_by_Addr((uint32_t *)bufCipher, DATA_LEN_INBYTE);
		}

		/* Decrypts data */
		desDone = 0;
		/* Enable AES interrupt */
		AES_EnableIt(AES_IER_DATRDY);
		AES_Configure(AES_MR_CIPHER_DECRYPT
						| keyMode
						| startMode
						| operationMode);
		/* Write the 128-bit/192-bit/256-bit key(s) in the Key Registers
			(AES_KEYxWRx)*/
		AES_WriteKey(aes_keys, keylength);

		/* The Initialization Vector Registers concern all modes except ECB. */
		if (operationMode != AES_MR_OPMOD_ECB)
			AES_SetVector(aes_vectors);

		if (startMode != AES_MR_SMOD_IDATAR0_START) {
			/* Write the data to be decrypted in the authorized Input Data
				Registers */
			AES_SetInput(&bufCipher[i]);

			if (startMode == AES_MR_SMOD_MANUAL_START)
				/* Set the START bit in the AES Control register AES_CR to
				begin the decryption process. */
				AES_Start();

			while (!desDone);

			AES_GetOutput(&bufOut[i]);
		} else {
			XDMAD_FreeChannel(&xdmad, dmaWriteChannel);
			XDMAD_FreeChannel(&xdmad, dmaReadChannel);
			_xdma_init();
			AES_SetDataLen(DATA_LEN_INBYTE);
			_xdma_configure_write(bufCipher, DATA_LEN_INWORD / 4);
			_xdma_configure_read(bufOut, DATA_LEN_INWORD / 4);
			SCB_CleanDCache_by_Addr((uint32_t *)bufCipher, DATA_LEN_INBYTE);
			XDMAD_StartTransfer(&xdmad, dmaWriteChannel);
			XDMAD_StartTransfer(&xdmad, dmaReadChannel);

			while (XDMAD_IsTransferDone(&xdmad, dmaReadChannel));

			SCB_InvalidateDCache_by_Addr((uint32_t *)bufOut, DATA_LEN_INBYTE);
		}
	}

	p = (char *)bufOut;
	printf("Dump plain text after AES decryption ...\n\r");

	for (i = 0; i < DATA_LEN_INBYTE; i++)
		printf("%c", *p++);

	XDMAD_FreeChannel(&xdmad, dmaWriteChannel);
	XDMAD_FreeChannel(&xdmad, dmaReadChannel);
}

/*----------------------------------------------------------------------------
 *        Exported functions
 *----------------------------------------------------------------------------*/

/**
 *  \brief AES Application entry point.
 *
 *  \return Unused (ANSI-C compatibility).
 *  \callgraph
 */
int main(void)
{
	uint32_t key;
	/* Disable watchdog */
	WDT_Disable(WDT);

	/* Enable I and D cache */
	SCB_EnableICache();
	SCB_EnableDCache();

	/* Output example information */
	printf("-- AES Example %s --\n\r", SOFTPACK_VERSION );
	printf("-- %s\n\r", BOARD_NAME);
	printf("-- Compiled: %s %s With %s--\n\r", __DATE__, __TIME__ ,
			COMPILER_NAME);

	/* Enable AES peripheral clock */
	PMC_EnablePeripheral(ID_AES);

	/* Configure and enable interrupt on XDMA */
	NVIC_ClearPendingIRQ(XDMAC_IRQn);
	NVIC_SetPriority(XDMAC_IRQn, 1);
	NVIC_EnableIRQ(XDMAC_IRQn);

	/* Display menu */
	_displayMenu();
	operationMode = AES_MR_OPMOD_ECB;
	startMode = AES_MR_SMOD_MANUAL_START;

	//printf("bufPlaint-0x%x  0x%x\n\r",bufPlaint,sizeof(bufPlaint));
	keyMode = 0;

	for (; ;) {
		key = DBG_GetChar();

		switch (key) {
		case '5':
		case '6':
		case '7':
			keyMode = ((key - '5') << AES_MR_KEYSIZE_Pos );
			_displayMenu();
			keylength = (keyMode == AES_MR_KEYSIZE_AES128) ? 16 : \
						((keyMode == AES_MR_KEYSIZE_AES192) ? 24 : 32);
			break;

		case '0':
		case '1':
		case '2':
		case '3':
		case '4':
			operationMode = ((key - '0') << AES_MR_OPMOD_Pos );
			_displayMenu();
			break;

		case 'm':
		case 'M':
			startMode = AES_MR_SMOD_MANUAL_START; _displayMenu();
			break;

		case 'a':
		case 'A':
			startMode = AES_MR_SMOD_AUTO_START; _displayMenu();
			break;

		case 'd':
		case 'D':
			startMode = AES_MR_SMOD_IDATAR0_START; _displayMenu();
			break;

		case 'h':
		case 'H':
			_displayMenu();
			break;

		case 'p':
		case 'P':
			_startAES();
			break;
		}
	}
}

