softpack
========

Atmel Software Package
----------------------

This repository contains samv7 softpack.
Samv7 softpack is under development.
Examples that have been ported to samv7 are:

#### Examples
  - **afe_temp_sensor**:-
     The example is aimed to demonstrate the temperature sensor feature 
	 inside the device. The channel 11 is connected to the sensor by default.
	 
  - **afe12_dma**:-
     The AFE12 example demonstrates how to use AFE peripheral with several modes.

  - **eefc_pgm**:-
     This basic example shows how to use the Enhance Embedded Flash (EEFC) peripheral to
	 program the internal flash, and manage secure and lock bits.

  - **Getting_started**:-
     This example uses Cortex M7 systick and TC0 on chip to print message on debug console and toggle the two leds available on Xplained board 

  - **isi**:-
     This example shows how to use OV camera module with isi peripheral. (this example is not functional due to limitations of Xplained board)
	 
  - **lcd**:-
     This example uses 3 wire spi interface for maXtouch xplained LCD board (ILI9488) to drive the LCD. PWM to control the brightness of LCD

  - **low_power**:-
     This example show how to use different low power modes available on SAMV7 micro-controller. Backup mode of SAMV7 does not work due to limitations of Xplained board.
	 
  - **mpu**:-
     This example shows how to use mpu to protect different region of memory and make them available for normal or privilege users.

  - **periph_prtotect**:-
     This example show how we can protect the PIOS of SAMV7. After using protection of PIOS they cant be modified.

  - **pmc_clock_switching**:-
     This example shows how to switch system clock from one to another (PLLA, SLCK, MAINCK) or change to fast RC.

  - **pwm**:-
     This example demonstrates a simple configuration of three PWM channels to generate variable duty cycle signals.
	 The update of the duty cycle values is made automatically by the Peripheral DMA Controller .
	 
  - **rtc**:-
     This basic example shows how to use the Real-Time Clock (RTC) peripheral available on the Atmel SAMV7 micro-controllers.

  - **rtt**:-
     This example demonstrates the Real-Time Timer (RTT) provided on SAMV7 micro-controllers. It enables the user to set an alarm and watch
	 it being triggered when the timer reaches the corresponding value.
	 
  - **sdram**:-
     This basic application shows Shows how to initialize and perform read and write a SDRAM memory.
  
  - **spi**:-
     Spi is configured in local loopback mode and uses SPI interrupt and DMA to store data from RDR reg of SPI to an input buffer
	 
  - **ssc_dma_audio**:-
     This example uses the Synchronous Serial Controller (SSC) of an SAMV7x micro-controller to output an audio stream through the on-board WM8904 CODEC.
	 This example is fully functional
	 
  - **tc_capture_waveform**:-
     This example indicates how to use TC in capture mode to measure the pulse frequency and count the total pulse number of an external signal injected on TIOA pin.
	 
  - **tcm**:-
     This example indicates how to use TCM memories on SAMV7. It uses ITCm and DTCM memory for small code and data. and copies buffer using TCm memory and then compare with operations done using sram memory.
	 
  - **trng**:-
     The TRNG example shows how to generate random data with TRNG peripheral.

  - **twi_eeprom**:-
     This example is fully functional and show how to uses twi with dma to transfer data to a flash memory.	 
  
  - **uart**:-
     UART example uses polling mode, and runs in local loopback mode
     In another mode in example UART0 is sending and receiving characters using XDMA channel
  
  - **usart**:-
     USART example uses DMA to transfer and receive buffer. In example it runs in loopback mode as well as in normal mode.
	 In normal mode you board should be connected to D14 and D15 (Rx and Tx of USART0 on Samv7 Xplained board) end then it will echo the characters on the terminal. It shows the how Time-out functionality of USART can be used as well.

  - **usart_hw_handhsaking**:-
     This example shows how to use RTS and CTS pin of USART. Connect the board to PC with appropriate USART and then send a file over terminal. This example will try to copy incoming buffer from PC to a local buffer using a ring buffer. and will raise CTS or RTS pin according to situation.

  - **usart_spi**:-
     This example show how to use UART's SPI functionality to use it as a SPI.
	 
  - **wdt**:-
     This example shows how to use samv7x family micro-controller's watchdog interrupt. This example use WDT0. 
     
  - **xdma**:-
     In this example chunk of data store in memory transfers to another area using DMA.
	
#### Crypto Examples  	
  - **aes**:-
    It encrypts and decrypts several test values in Electronic CodeBook (ECB) and Cipher Block Chaining (CBC),OBC,OFB,TRC modes and checks them against the known answers.
	
  - **icm**:-
    The first one is used to hash a list of memory regions and save the digests to memory (ICM Hash Area). The second operation mode is an active monitoring of the memory.

#### Ethernet Examples     
  - **gmac**:-
    Gmac example using lwip library is functional. 
	
  - **gmac_lwip**:-
    Gmac example using lwip library to show a small web-page. It is semi-functional.
	
  - **gmac_uip_helloworld**:-
    Example using uip library and it is functional. 
	
  - **gmac_uip_telnetd**:-
    Example using uip library and it is functional. It show how we can use telenet functionality.
	
  - **gmac_uip_webserver**:-
    Example using uip library and it is functional. It loads a small page on PC if Xplained board is connected to network.
    
	
#### Storage Examples     

  - **hsmci_multimedia_card**:-
    This example uses HSMCI interface to show how to uses a mmc connected to the Xplained board.
	
  - **hsmci_sdcard**:-
    This example uses HSMCI interface to show how to uses a sdcard connected to the Xplained board.
	
  - **hsmci_sdcard_fatfs**:-
    This example uses HSMCI interface to show how to uses a sdcard connected to the Xplained board witha Fat file system to write and read the files on sdcard.

  - **hsmci_sdio**:-
    This example uses HSMCI interface to show how to uses a sdio connected to the Xplained board.
	
  - **qspi_flash**:-
    This example uses QSPI interface to show how to read/write a QSPI flash on board.

	
#### USB Device Examples  


  - **usb_audio_looprec**:-
	When an Xplained board running this program connected to a host (PC for example), with USB cable, the Xplained board appears as a desktop speaker for the host. 
	Then the host can play sound through host software. The audio stream from the host is then sent to the Xplained board. At the same time, the audio stream received is also
	sent back to host from Xplained board for recording.

  - **usb_audio_speaker**:-
  	When an Xplained board running this program connected to a host (PC for example), with USB cable, the Xplained board appears as a desktop speaker for the host. 
	Then the host can play sound through host software. The audio stream from the host is then sent to the Xplained board, and eventually sent to audio DAC connected to the amplifier. 
	
  - **usb_cdc**:-
  	CDC example shows how xplained board can be connected to a PC as a CDC (serial port) class USB.

  - **usb_core**:-
  	This a basic code to show the the implementataion of USB device. It has all function for USB core for intital USB enumerations.

  - **usb_hid_aud*:-
  	The demo simulates a USB device that integrates HID Keyboard and Audio Desktop Speaker function.

  - **usb_hid_keyboard**:-
	The demo simulates a simple keyboard with a capslock and 'a' on it.
	
  - **usb_hid_mouse**:-
    This package can be used with all Atmel Xplained board that has UDP interface and have push button or joystick on it.
	
  - **usb_hid_msd**:-
    The demo simulates a USB device that integrates HID Keyboard and mass storage class USB.

  - **usb_hid_transfer**:-
    The demo simulates a customized HID device that reports customized data  stream, in which informations on LEDs and buttons are packed, to host.

  - **usb_iad_cdc_aud**:-
	When the board running this program connected to a host (PC for example), with  USB cable, host will notice the attachment of a USB %device (USB Composite Device) with a USB Virtual COM port(AT91 USB to Serial Converter) and a USB Audio Device.

  - **usb_iad_cdc_cdc**:-
    This demo simulates 2 USB to RS-232 Serial Port Converter.

  - **usb_iad_cdc_hid**:-
	This demo simulates a USB composite device that has USB Serial RS232 Converter and USB HID Keyboard functions.

  - **usb_iad_cdc_msd**:-
    The demo simulates a USB composite device that integrates USB CDC Serial RS232 Converter function and USB Disk function.

  - **usb_massstorage**:-
    It shows how USB can be used as MSC class to use memories on board.