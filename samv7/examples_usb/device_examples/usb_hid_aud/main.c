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
/** \cond usb_hid_aud
 *  \page usb_hid_aud USB HID(Keyboard)+Audio(Speaker) Example
 *
 *  \section Purpose
 *
 *  The HID Audio Composite Example will help you to get familiar with the
 *  USB Device Port(UDP) and SSC on SAMv7 Microcontrollers. Also
 *  it can help you to be familiar with the USB Framework that is used for
 *  rapid development of USB-compliant class drivers such as USB Composite
 *  HID and Audio classes.
 *
 *  \section Requirements
 *
 *  This package can be used with SAMV71 Xplained Ultra board that have both
 *  UDP and SSC.
 *
 *  \section win_drv_update Windows Driver Update
 *
 * The composite device is generally supported by Microsoft windows, but some
 * patches are needed for muti-interface functions such as CDC & Audio.
 *
 *  \section Description
 *
 * The demo simulates a USB device that integrates HID Keyboard and Audio
 * Desktop Speaker function.
 *
 * When the board running this program connected to a host (PC for example), with
 * USB cable, host will notice the attachment of a USB device (USB Composite
 * Device) with a USB Human Interface Device and a USB Audio Device.
 *
 *  \section Usage
 *
 *  -# Build the program and download it inside the SAM V71 Xplained Ultra board.
 *     Please refer to the Getting Started with SAM V71 Microcontrollers.pdf
 *  -# On the computer, open and configure a terminal application
 *     (e.g. HyperTerminal on Microsoft Windows) with these settings:
 *    - 115200 bauds
 *    - 8 bits of data
 *    - No parity
 *    - 1 stop bit
 *    - No flow control
 *  -# Start the application.
 *  -# In the terminal window, the following text should appear:
 *  \code
 *  -- USB HID + Audio Device Example xxx --
 *  -- SAMxxxxx-xx
 *  -- Compiled: xxx xx xxxx xx:xx:xx --
 *  \endcode
 *  -# When connecting USB cable to windows, the LED blinks, and the host
 *     reports a new USB device attachment (if it's the first time you connect
 *     an %audio speaker demo board to your host). You can find new
 *     "USB Composite Device" and "USB Audio Device" appear in the hardware
 *     device list.
 *  -# You can play sound in host side through the USB Audio Device, and it
 *     can be heard from the earphone connected to the board.
 *
 *  \section References
 *  - usb_hid_aud/main.c
 *  - pio: Pin configurations and peripheral configure.
 *  - ssc: SSC interface driver
 *  - usb: USB Framework, HID, Audio function driver and UDP interface driver
 *      - \ref usbd_framework
 *         - \ref usbd_api
 *      - \ref usbd_composite "composite"
 *         - \ref usbd_composite_drv
 *      - \ref usbd_aud "audio"
 *         - \ref usbd_audio_rec_drv
 *  - projects: more detailed information for HID(Keyboard) and Audio(Speaker)
 *      - \ref usb_core
 *      - \ref usb_audio_speaker, \ref usb_audio_headphone
 *
 */

/**
 *  \file
 *
 *  This file contains all the specific code for the
 *  usb_iad_hid_aud example.
 */

/*----------------------------------------------------------------------------
 *         Headers
 *----------------------------------------------------------------------------*/

#include "board.h"

#include <HIDAUDDDriver.h>
#include <HIDDKeyboard.h>
#include <AUDDFunction.h>

#include <USBD_Config.h>
#include <USBD_LEDs.h>

#include <string.h>

/*----------------------------------------------------------------------------
 *         Definitions
 *----------------------------------------------------------------------------*/

#define NO_PUSHBUTTON

/*- HID */
/** Number of keys used in the example. */
#define NUM_KEYS                    2

/** Number of non-modifiers keys. */
#define NUM_NORMAL_KEYS             1

/** Number of modifier keys. */
#define NUM_MODIFIER_KEYS           (NUM_KEYS - NUM_NORMAL_KEYS)

/** NumLock LED index. */
#define LED_NUMLOCK                 USBD_LEDOTHER

/*- Audio */
/**  Number of available audio buffers. */
#define BUFFER_NUMBER       500
/**  Size of one buffer in bytes. */
#define BUFFER_SIZE     (AUDDevice_BYTESPERFRAME)

/**  Delay in ms for starting the DAC transmission
     after a frame has been received. */
#define DAC_DELAY           2

/** SSC: Number of slots in a frame */
#define SLOT_BY_FRAME           (2)
/** SSC: Number of bits in a slot */
#define BITS_BY_SLOT            (16)
#define I2S_SLAVE_TX_SETTING      ((SSC_TCMR_CKS_TK) |        \
								   (SSC_TCMR_CKO_NONE) |      \
								   (SSC_TCMR_START_TF_EDGE) | \
								   (SSC_TCMR_STTDLY(1)) |     \
								   (SSC_TCMR_PERIOD(0)))

#define I2S_SLAVE_TX_FRM_SETTING  ((SSC_TFMR_DATLEN(BITS_BY_SLOT - 1)) |\
								   (SSC_TFMR_MSBF) |                    \
								   (SSC_TFMR_DATNB(SLOT_BY_FRAME - 1)) |\
								   (SSC_TFMR_FSOS_NONE))


#define I2S_SLAVE_RX_SETTING      ((SSC_RCMR_CKS_TK) |       \
								   (SSC_RCMR_CKO_NONE) |     \
								   (SSC_RCMR_CKI) |          \
								   (SSC_RCMR_START_RF_EDGE) |\
								   (SSC_RCMR_STTDLY(1)) |    \
								   (SSC_RCMR_PERIOD(0)))

#define I2S_SLAVE_RX_FRM_SETTING  ((SSC_RFMR_DATLEN(BITS_BY_SLOT - 1)) |\
								   (SSC_RFMR_MSBF) |                    \
								   (SSC_RFMR_DATNB(SLOT_BY_FRAME - 1)) |\
								   (SSC_RFMR_FSOS_NONE))

/** TWI clock */
#define TWI_CLOCK               400000
/** Audio sample rate */
#define SAMPLE_RATE             (48000)


/*----------------------------------------------------------------------------
 *         External variables
 *----------------------------------------------------------------------------*/

/** Descriptor list for USB Audio Speaker Device Driver */
extern const USBDDriverDescriptors hidauddDriverDescriptors;

/*----------------------------------------------------------------------------
 *         Internal variables
 *----------------------------------------------------------------------------*/

/*- HID */

#ifdef NO_PUSHBUTTON
#else
/** List of pinsPushButtons to configure for the application. */
static Pin pinsPushButtons[] = {PINS_PUSHBUTTONS};
#endif

/** Array of key codes produced by each button. */
static uint8_t keyCodes[NUM_KEYS] = {
	HIDKeypad_A,
	HIDKeypad_NUMLOCK,
};

/** Current status (pressed or not) for each key. */
static uint8_t keyStatus[NUM_KEYS];

/*- Audio */
/**  Data buffers for receiving audio frames from the USB host. */

COMPILER_ALIGNED(32) static uint8_t buffers[BUFFER_NUMBER][BUFFER_SIZE];

/**  Number of samples stored in each data buffer. */
static uint32_t bufferSizes[BUFFER_NUMBER];
/**  Next buffer in which USB data can be stored. */
static uint32_t inBufferIndex = 0;
/**  Next buffer which should be sent to the DAC. */
static uint32_t outBufferIndex = 0;
/**  Number of buffers that can be sent to the DAC. */
static volatile uint32_t numBuffersToSend = 0;

/**  Current state of the DAC transmission. */
static volatile uint32_t isDacActive = 0;
static volatile uint32_t isFirstFrame = 1;
/**  Number of buffers to wait for before the DAC starts to transmit data. */
static volatile uint32_t dacDelay;

/** Twi instance*/
static Twid twid;
/** Global DMA driver for all transfer */
COMPILER_ALIGNED(32) static sXdmad dmad;
/** DMA channel for TX */
static uint32_t sscDmaTxChannel;

/** List of pins to configure. */
static const Pin pinsAudio[] = { PIN_TWI_TWD0,
								 PIN_TWI_TWCK0,
								 PIN_SSC_TD,
								 PIN_SSC_TK,
								 PIN_SSC_TF,
								 PIN_SSC_RD,
								 PIN_SSC_RK,
								 PIN_SSC_RF,
								 PIN_PCK2
							   };

/*----------------------------------------------------------------------------
 *         Internal functions
 *----------------------------------------------------------------------------*/
/**
 * Interrupt handler for the XDMAC.
 */
void XDMAC_Handler(void)
{
	XDMAD_Handler(&dmad);
}

/**
 *  \brief Start DMA sending/waiting data.
 */
static void _SscDma(volatile uint32_t *pReg, uint32_t dmaChannel,
					void *pBuffer, uint16_t wSize)
{
	sXdmad *pDmad = &dmad;
	sXdmadCfg xdmadCfg;


	xdmadCfg.mbr_ubc = wSize;
	xdmadCfg.mbr_sa = (uint32_t) pBuffer;
	xdmadCfg.mbr_da = (uint32_t) pReg;
	xdmadCfg.mbr_cfg = XDMAC_CC_TYPE_PER_TRAN
					| XDMAC_CC_MBSIZE_SINGLE
					| XDMAC_CC_DSYNC_MEM2PER
					| XDMAC_CC_CSIZE_CHK_1
					| XDMAC_CC_DWIDTH_HALFWORD
					| XDMAC_CC_SIF_AHB_IF1
					| XDMAC_CC_DIF_AHB_IF1
					| XDMAC_CC_SAM_INCREMENTED_AM
					| XDMAC_CC_DAM_FIXED_AM
					| XDMAC_CC_PERID(XDMAIF_Get_ChannelNumber(
									ID_SSC, XDMAD_TRANSFER_TX));
	xdmadCfg.mbr_bc = 0;
	xdmadCfg.mbr_ds = 0;
	xdmadCfg.mbr_sus = 0;
	xdmadCfg.mbr_dus = 0;

	memory_sync();
	XDMAD_ConfigureTransfer(pDmad, dmaChannel, &xdmadCfg, 0, 0, (
								 XDMAC_CIE_BIE   |
								 XDMAC_CIE_DIE   |
								 XDMAC_CIE_FIE   |
								 XDMAC_CIE_RBIE  |
								 XDMAC_CIE_WBIE  |
								 XDMAC_CIE_ROIE));
	SCB_CleanDCache_by_Addr((uint32_t *)pBuffer, wSize);
	XDMAD_StartTransfer(pDmad, dmaChannel);
	SSC_EnableTransmitter(SSC);
}

/**
 *  \brief DMA TX callback
 */
static void _SscTxCallback(uint8_t status, void *pArg)
{
	sXdmad *pDmad = &dmad;
	Xdmac *pXdmac = pDmad->pXdmacs;
    /*dummy */
    status = status;
	pArg = pArg; 

	if (numBuffersToSend == 0) {
		/* End of transmission */
		isDacActive = 0;
		return;
	}

	/* Load next buffer */
	memory_sync();
	XDMAC_SetSourceAddr(pXdmac, sscDmaTxChannel,
						(uint32_t) buffers[outBufferIndex]);
	XDMAC_SetMicroblockControl(pXdmac, sscDmaTxChannel,
							   bufferSizes[outBufferIndex]);
	SCB_CleanDCache_by_Addr((uint32_t *) buffers[outBufferIndex],
							bufferSizes[outBufferIndex]);
	outBufferIndex = (outBufferIndex + 1) % BUFFER_NUMBER;
	numBuffersToSend --;
	XDMAD_StartTransfer(pDmad, sscDmaTxChannel);
}

/**
 * \brief DMA driver configuration
 */
static void _ConfigureDma(void)
{
	sXdmad *pDmad = &dmad;
	/* Driver initialize */
	XDMAD_Initialize(pDmad, 0);
	/* IRQ configure */
	NVIC_EnableIRQ(XDMAC_IRQn);

	/* Allocate DMA channels for SSC */
	sscDmaTxChannel = XDMAD_AllocateChannel(pDmad, XDMAD_TRANSFER_MEMORY, ID_SSC);

	if (  sscDmaTxChannel == XDMAD_ALLOC_FAILED) {
		printf("xDMA channel allocation error\n\r");

		while (1);
	}

	/* Set TX callback */
	XDMAD_SetCallback(pDmad, sscDmaTxChannel, (XdmadTransferCallback)_SscTxCallback,
					  0);
	XDMAD_PrepareChannel(pDmad, sscDmaTxChannel);
}

/**
 * Enable/Disable audio channels
 */
static void AudioPlayEnable(uint8_t enable)
{
	if (enable == 1)
		SSC_EnableTransmitter(SSC);
	else if (enable == 0)
		SSC_DisableTransmitter(SSC);
}

/**
 * Adjust codec for sync
 * \param adjust Sync case: default(0)/faster(>0)/slower(<0)
 */
static void _SyncAdjust(int32_t adjust)
{
	if (adjust > 0) {
		/* Fractional multiply for FLL_K, Fref = 0x8000 (1/2) */
		WM8904_Write(&twid, WM8904_SLAVE_ADDRESS, WM8904_REG_FLL_CRTL3, 0xFF00);
		/* FLL_GAIN=0, FLL_N=187 */
		return;
	}

	if (adjust < 0) {
		/* Fractional multiply for FLL_K, Fref = 0x8000 (1/2) */
		WM8904_Write(&twid, WM8904_SLAVE_ADDRESS, WM8904_REG_FLL_CRTL3, 0x5000);
		/* FLL_GAIN=0, FLL_N=187 */
		return;
	}

	/* Default: 32K -> 48K*256, FLL: 32768*187.5/16/8
	 */
	/* FLL_FRATIO=4 (/16), FLL_OUTDIV= 7 (/8) */
	/* Fractional multiply for FLL_K, Fref = 0x8000 (1/2) */
	WM8904_Write(&twid, WM8904_SLAVE_ADDRESS, WM8904_REG_FLL_CRTL3,
				 0x8000 + 0x3000);
	/* FLL_GAIN=0, FLL_N=187 */
	return;
}

/**
 * Configure the TWI and DACC for audio output.
 * \param sampleRate Audio sample rate.
 * \param mck        MCK frequency.
 */
static void _ConfigureAudioPlay(uint32_t sampleRate, uint32_t mck)
{
	/* -- Pins Configuration -- */
	PIO_Configure(pinsAudio, PIO_LISTSIZE(pinsAudio));

	/* -- SSC Configuration -- */
	sampleRate = sampleRate; /*dummy */
	SSC_Configure(SSC, 0, mck);
	SSC_DisableTransmitter(SSC);
	SSC_DisableReceiver(SSC);
	SSC_ConfigureTransmitter(SSC, I2S_SLAVE_TX_SETTING, I2S_SLAVE_TX_FRM_SETTING);
	SSC_DisableTransmitter(SSC);

	/* Enable TWI peripheral clock */
	PMC_EnablePeripheral(ID_TWIHS0);
	/* Configure and enable the TWI (required for accessing the DAC) */
	TWI_ConfigureMaster(TWIHS0, TWI_CLOCK, mck);
	TWID_Initialize(&twid, TWIHS0);

	/* Initialize the audio DAC
	 */
	WM8904_Write(&twid, WM8904_SLAVE_ADDRESS, WM8904_REG_RESET, 0);
	Wait(100);

	/* WM8904 as master */
	if (WM8904_Read(&twid, WM8904_SLAVE_ADDRESS, 0) != 0x8904) {
		printf("WM8904 not found!\n\r");

		while (1);
	}

	WM8904_Init(&twid, WM8904_SLAVE_ADDRESS, PMC_MCKR_CSS_SLOW_CLK);
	_SyncAdjust(0);
	PMC_ConfigurePCK2(PMC_MCKR_CSS_SLOW_CLK, PMC_MCKR_PRES_CLK_1);
	/* Mute */
	AudioPlayEnable(0);
}

/**
 * Monitor keyboard buttons & Update key status in HID driver
 */
static void HIDDKeyboardProcessKeys(void)
{
	uint32_t i;
	uint8_t pressedKeys[NUM_KEYS];
	uint8_t pressedKeysSize = 0;
	uint8_t releasedKeys[NUM_KEYS];
	uint8_t releasedKeysSize = 0;

	/* Monitor buttons */
#ifdef NO_PUSHBUTTON

	if (DBG_IsRxReady()) {
		uint8_t key = DBG_GetChar();

		switch (key) {
		case '1': case '2':
			i = key - '1';

			if (keyStatus[i]) {
				/* Key press simulation */
				TRACE_INFO("Key %u pressed\n\r", (unsigned int)i);
				keyStatus[i] = 0;
				pressedKeys[pressedKeysSize] = keyCodes[i];
				pressedKeysSize ++;
				HIDDKeyboard_RemoteWakeUp();
			} else {
				/* Key release simulation */
				TRACE_INFO("Key %u released\n\r", (unsigned int)i);
				keyStatus[i] = 1;
				releasedKeys[releasedKeysSize] = keyCodes[i];
				releasedKeysSize++;
			}

			break;

		default: DBG_PutChar(key);
		}
	}

#else

	for (i = 0; i < PIO_LISTSIZE(pinsPushButtons); i++) {
		/* Check if button state has changed */
		uint8_t isButtonPressed = PIO_Get(&(pinsPushButtons[i]));

		if (isButtonPressed != keyStatus[i]) {
			/* Update button state */
			if (!isButtonPressed)   {
				/* Key has been pressed */
				TRACE_INFO("Key %u has been pressed\n\r", i);
				keyStatus[i] = 0;
				pressedKeys[pressedKeysSize] = keyCodes[i];
				pressedKeysSize++;
				HIDDKeyboard_RemoteWakeUp();
			} else {
				/* Key has been released */
				TRACE_INFO("Key %u has been released\n\r", i);
				keyStatus[i] = 1;
				releasedKeys[releasedKeysSize] = keyCodes[i];
				releasedKeysSize++;
			}
		}
	}

#endif

	/* Update key status in the HID driver if necessary */
	if ((pressedKeysSize != 0) || (releasedKeysSize != 0)) {
		uint8_t status;

		do {
			status = HIDDKeyboard_ChangeKeys(pressedKeys,
											 pressedKeysSize,
											 releasedKeys,
											 releasedKeysSize);
		} while (status != USBD_STATUS_SUCCESS);
	}
}

/**
 *  Invoked when a frame has been received.
 */
static void FrameReceived(uint32_t unused,
						  uint8_t status,
						  uint32_t transferred,
						  uint32_t remaining)
{
	unused = unused; /*dummy */
	remaining = remaining; /*dummy */

	if (status == USBD_STATUS_SUCCESS) {
		bufferSizes[inBufferIndex] = transferred / AUDDevice_BYTESPERSAMPLE;
		inBufferIndex = (inBufferIndex + 1) % BUFFER_NUMBER;
		numBuffersToSend++;

		/* Start DAc transmission if necessary */
		if (!isDacActive) {
			dacDelay = DAC_DELAY;
			isDacActive = 1;
		}
		/* Wait until a few buffers have been received */
		else if (dacDelay > 0)
			dacDelay--;
		/* Start sending buffers */
		else if (isFirstFrame) {
			isFirstFrame = 0;
			//AudioPlayEnable(1);
			_SscDma(&(SSC->SSC_THR), sscDmaTxChannel, buffers[outBufferIndex],
					bufferSizes[outBufferIndex]);
			outBufferIndex = (outBufferIndex + 1) % BUFFER_NUMBER;
			numBuffersToSend --;
		}
	} else if (status == USBD_STATUS_ABORTED) {
		/* Error , ABORT, add NULL buffer */
		bufferSizes[inBufferIndex] = 0;
	} else {
		/* Packet is discarded */
	}

	/* Receive next packet */
	AUDDFunction_Read(buffers[inBufferIndex],
					  AUDDevice_BYTESPERFRAME,
					  (TransferCallback) FrameReceived,
					  0); // No optional argument
}

/*-------------------------------------------
 *      USB Device Driver callbacks
 *-------------------------------------------*/
/**
 * Invoked when the configuration of the device changes. Parse used endpoints.
 * \param cfgnum New configuration number.
 */
void USBDDriverCallbacks_ConfigurationChanged(unsigned char cfgnum)
{
	HIDAUDDDriver_ConfigurationChangedHandler(cfgnum);
}

/**
 * Invoked whenever the active setting of an interface is changed by the
 * host. Reset streaming interface.
 * \param interface Interface number.
 * \param setting Newly active setting.
 */
void USBDDriverCallbacks_InterfaceSettingChanged(unsigned char interface,
		unsigned char setting)
{
	HIDAUDDDriver_InterfaceSettingChangedHandler(interface, setting);
}

/**
 *  Invoked whenever a SETUP request is received from the host. Forwards the
 *  request to the standard handler.
 */
void USBDCallbacks_RequestReceived(const USBGenericRequest *request)
{
	HIDAUDDDriver_RequestHandler(request);
}

/*-------------------------------------------
 *      USB Function driver callbacks
 *-------------------------------------------*/

/**
 *  Invoked when an audio channel get muted or unmuted. Mutes/unmutes the
 *  channel at the DAC level.
 *  \param mic      Microphone/Speaker stream changed.
 *  \param channel  Channel number that changed.
 *  \param muted    Indicates the new mute status of the channel.
 */
void AUDDFunction_MuteChanged(uint8_t mic, uint8_t channel, uint8_t muted)
{
	if (mic) return;

	/* Speaker Master channel */
	if (channel == AUDD_CH_Master) {
		if (muted) {
			AudioPlayEnable(0);
			TRACE_WARNING("MuteMaster ");
		} else {
			TRACE_INFO("UnmuteMaster ");
			AudioPlayEnable(1);
		}
	}
}

/**
 *  Invoked when an audio streaming interface setting changed. Actually control
 *  streaming rate.
 *  \param mic         1 to indicate microphone mute changed.
 *  \param newSetting  New stream (interface) setting.
 */
void AUDDFunction_StreamSettingChanged(uint8_t mic, uint8_t newSetting)
{
	mic = mic; /* dummy */

	if (newSetting) {
		LED_Set(USBD_LEDOTHER);
		//XDMAD_StopTransfer(&dmad, sscDmaTxChannel);
		numBuffersToSend = 0;
	} else LED_Clear(USBD_LEDOTHER);
}

/**
 * Invoked when the status of the keyboard LEDs changes. Turns the num. lock
 * LED on or off.
 * \param numLockStatus Indicates the current status of the num. lock key.
 * \param capsLockStatus Indicates the current status of the caps lock key.
 * \param scrollLockStatus Indicates the current status of the scroll lock key
 */
void HIDDKeyboardCallbacks_LedsChanged(
	uint8_t numLockStatus,
	uint8_t capsLockStatus,
	uint8_t scrollLockStatus)
{
	capsLockStatus = capsLockStatus; /* dummy */
	scrollLockStatus = scrollLockStatus; /* dummy */

	/* Num. lock */
	if (numLockStatus)
		LED_Set(LED_NUMLOCK);
	else
		LED_Clear(LED_NUMLOCK);
}

/**
 * Configure USB settings for USB device
 */
static void _ConfigureUsbhs(void)
{
	/* UTMI parallel mode, High/Full/Low Speed */
	/* UUSBCK not used in this configuration (High Speed) */
	PMC->PMC_SCDR = PMC_SCDR_USBCLK;
	/* USB clock register: USB Clock Input is UTMI PLL */
	PMC->PMC_USB = PMC_USB_USBS;
	/* Enable peripheral clock for USBHS */
	PMC_EnablePeripheral(ID_USBHS);
	USBHS->USBHS_CTRL = USBHS_CTRL_UIMOD_DEVICE;
	/* Enable PLL 480 MHz */
	PMC->CKGR_UCKR = CKGR_UCKR_UPLLEN | CKGR_UCKR_UPLLCOUNT(0xF);

	/* Wait that PLL is considered locked by the PMC */
	while (!(PMC->PMC_SR & PMC_SR_LOCKU));

	/* IRQ */
	NVIC_EnableIRQ(USBHS_IRQn);
}
/*----------------------------------------------------------------------------
 *         Exported functions
 *----------------------------------------------------------------------------*/

/**
 *  \brief usb_iad_hid_aud Application entry point.
 *
 *  Starts the driver and waits for an audio input stream to forward to the DAC.
 */
int main(void)
{
	volatile uint8_t usbConn = 0;
	volatile uint8_t audioOn = 0;
	int32_t  numDiff = 0, prevDiff = 0;
	int8_t   clockAdjust = 0;

	/* Disable watchdog */
	WDT_Disable(WDT);

	SCB_EnableICache();
	SCB_EnableDCache();


	printf("-- USB HID + Audio Device Example %s --\n\r", SOFTPACK_VERSION);
	printf("-- %s\n\r", BOARD_NAME);
	printf("-- Compiled: %s %s With %s--\n\r", __DATE__, __TIME__ ,
			COMPILER_NAME);

	TimeTick_Configure();
	/* Interrupt priority */
	NVIC_SetPriority(USBHS_IRQn, 2);

	/* If they are present, configure Vbus & Wake-up pins */
	PIO_InitializeInterrupts(0);

	/* Initialize all USB power (off) */
	_ConfigureUsbhs();

	/* ----- HID Function Initialize */
#ifdef NO_PUSHBUTTON
	printf("-- : DBG key 1 2 used as buttons\n\r");
	printf("-- : 1st press to push, 2nd press to release\n\r");
#else
	/* Initialize key statuses and configure push buttons */
	PIO_Configure(pinsPushButtons, PIO_LISTSIZE(pinsPushButtons));
#endif
	memset(keyStatus, 1, NUM_KEYS);
	//LED_Configure(LED_NUMLOCK);

	/* Audio STREAM LED */
	LED_Configure(USBD_LEDOTHER);

	/* Configure Audio */
	_ConfigureAudioPlay(AUDDevice_SAMPLERATE, BOARD_MCK);

	/* Configure DMA */
	_ConfigureDma();

	/* USB audio driver initialization */
	HIDAUDDDriver_Initialize(&hidauddDriverDescriptors);

	/* connect if needed */
	USBD_Connect();

	/* Infinite loop */
	while (1) {
		if (USBD_GetState() < USBD_STATE_CONFIGURED) {
			usbConn = 0;
			continue;
		}

		if (audioOn) {
			if (isDacActive == 0) {
				AudioPlayEnable(0);
				printf("audE ");
				isFirstFrame = 1;
				audioOn = 0;
			} else {
				numDiff = numBuffersToSend - DAC_DELAY;

				if (prevDiff != numDiff) {
					prevDiff = numDiff;

					if (numDiff > 1 && clockAdjust != 1) {
						printf("+");
						/* USB too fast or SSC too slow: faster clock */
						clockAdjust = 1;
						_SyncAdjust(1);
					}

					if (numDiff < -1 && clockAdjust != -1) {
						printf("-");
						/* USB too slow or SSC too fast: slower clock */
						clockAdjust = -1;
						_SyncAdjust(-1);
					}

					if (numDiff == 0 && clockAdjust != 0) {
						clockAdjust = 0;
						_SyncAdjust(0);
					}
				}
			}
		} else if (isDacActive) {
			printf("audS ");
			audioOn = 1;
		}

		if (usbConn == 0) {
			usbConn = 1;
			/* Start Reading the incoming audio stream */
			AUDDFunction_Read(buffers[inBufferIndex],
							  AUDDevice_BYTESPERFRAME,
							  (TransferCallback) FrameReceived,
							  0); // No optional argument
		}

		HIDDKeyboardProcessKeys();
	}
}
/** \endcond */
