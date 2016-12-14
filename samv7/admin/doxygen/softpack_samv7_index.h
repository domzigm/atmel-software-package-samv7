/*! 
\if document_SAMV71_XULT
\mainpage SAMV71 Xplained Ultra Software Package
The SAMV7 Software Package will guide you through the best programming usage. Hefty examples of applications can be found \ref page_examples "here".
\elseif document_SAME70_XPLD
\mainpage SAME70 Xplained Software Package
The SAME7 Software Package will guide you through the best programming usage. Hefty examples of applications can be found \ref page_examples "here".
\endif

\section microcontroller_sec Microcontroller features
- ARM Cortex-M running at up to 300MHz
- 16 Kbytes of Icache and 16 Kbytes of Dcache with Error Code Correction (ECC)
- 16 zones Memory Protection Unit (MPU)
- DSP Instructions, Thumb®-2 instruction set
- Embedded Trace Module (ETM) with instruction trace stream, including Trace Port Interface Unit (TPIU)

\section sec_getStarted Get Started
\if document_SAMV71_XULT
The SAMV7 is a Cortex-M based processor. This processor is designed to achieve high system performance in power and cost-sensitive embedded applications.  It is also designed to be fast and easy to program, no assembler code or deep knowledge of the architecture to create simple application are required. The \ref getting-started "Getting Started" example provides a simple template to start a new project on SAMV7.
\elseif document_SAME70_XPLD
The SAME7 is a Cortex-M based processor. This processor is designed to achieve high system performance in power and cost-sensitive embedded applications.  It is also designed to be fast and easy to program, no assembler code or deep knowledge of the architecture to create simple application are required. The \ref getting-started "Getting Started" example provides a simple template to start a new project on SAME7.
\endif

\section libChip_sec Chip Library 
\ref libchip_page library consists of a set of functions, data structures and macros covering all peripherals.
 The development of each driver is driven by a common API (application programming interface)
which standardizes the driver structure, the functions and the names of parameters.
- The Power Management Controller (\ref pmc_module "PMC") optimizes power consumption by controlling all system and user peripheral clocks. The PMC enables/disables the clock inputs to many of the peripherals and the Core.
\ref pmc_clock_switching gives an example of how to switch from a clock to another (PLLA, PLLUTMI, 32K SLCK/internal RC, MCK) or change divider. \ref sysc allows to measure the consumption of the core in different modes (idle mode, slow clock mode, backup mode), and how to wake-up from back-up mode with \ref rtc_module "RTC" alarm or WKUP pin.

- \ref AFEC_module "AFE" (Analog-Front-End Controller) is based on a 12-bit Analog-to-Digital Converter (ADC) managed by an AFE Controller.
 \ref afe12_dma shows how to transfer several samples processed automatically without any intervention of
the processor thanks to the DMA.  \ref afe_temp_sensor example is aimed to demonstrate the temperature sensor feature inside the device.
 
- \ref twi_module "TWI" interconnects components on a unique two-wire bus, \ref twi_eeprom demonstrates how to use the TWI peripheral to access an external serial EEPROM.
\ref twi_slave demonstrates the TWI peripheral in slave mode. It mimics the behavior of a serial memory, enabling the TWI master to read and write data in its internal SRAM.\n

- \ref tc_module "TC" includes four identical 16-bit Timer Counter channels, Each channel can be independently programmed to perform
frequency measurement, event counting, interval measurement, pulse generation. \ref tc_capture_waveform indicate how to use TC in capture mode to measure the pulse width and count the total pulse number of an external signal injected on TIOA pin.

- \ref pwm_module "PWM" can generate output waveforms and configure it's characteristics. \ref pwm make two leds blink thanks to the PWM.
- \ref wdt_module "WDT" can be used to prevent system lock-up if the software becomes trapped in a deadlock. \ref wdt "Watchdog with IRQ Interrupt Example" configures the watchdog and trigger a interrupt(or reset).

- The peripheral DMA operation will done by their own drivers \ref dmac_module "DMAC" transfers 
data from a source peripheral to a destination peripheral over one or more AMBA buses.

- \ref hsmci_module "HSMCI"  supports the MultiMedia Card (MMC) Specification V4.3, the SD Memory Card Specification V2.0, the SDIO V2.0 specification and CE-ATA V1.1. The \ref hsmci_sdcard "Basic SD/MMC Card Example"  gives details on how to implement SD drivers through HSMCI interface.\ref hsmci_multimedia_card offers a set of functions to perform MultiMedia Card tests, and \ref hsmci_sdio detects SDIO device (Test on SPB104 Wifi SIP) connected and perform R/W operation on it.


- \ref spi_module "SPI" is a synchronous serial data link capable of full-duplex communication with external devices in master or slave mode. \ref spi "SPI Example" shows control of the SPI in loop back mode.

\if document_SAMV71_XULT
- Quad SPI Interface (QSPI) is a synchronous serial data link that provides communication with external devices in master, it can be used in SPI mode or in Serial Memory mode. \ref qspi_flash shows how to setup the QSPI in order to initialize, read and write a serial dataflash.
\endif

- \ref ssc_module "SSC" provides a synchronous communication link with external devices, \ref ssc_dma_audio describes the configuration required to connect the SSC to a device with an I2S-compatible serial bus.

- \ref usart_module "USART" provides one full duplex universal synchronous asynchronous serial link. The USART can works in SPI mode, \ref usart_spi would show the example of reading and writing as master or slave.
The usart asynchronous mode using DMA is shown in the \ref usart "USART example with DMA", and hardware flow control implementation is illustrated in \ref usart_hw_handshaking "USART hardware handshaking Example".

- \ref gmac_module "GMAC" implements a 10/100 Mbps Ethernet MAC compatible with the IEEE 802.3 standard. \ref gmac enables the device to respond to a ping command sent by a host computer.

- \ref usb_lib "UDP" is compliant with the Universal Serial Bus (USB) V2.0 high-speed device specification. An example on how to implement USB Mass Storage class is detailed in \ref usb_massstorage "USB Device Mass Storage Example". The \ref usb_cdc_serial "USB CDC serial converter" shows how to
   implement USB Communication Device class based on the UDP peripheral. \ref usb_audio_speaker "USB Audio Speaker Example" details the implementation on USB Audio Device class.

\section libboard_sec Board Library 
\ref lib_board library includes a description of the board related device drivers plus some source code corresponding to components driver. 
The firmware library allows any device to be used in the user application without the need for in-depth study of each peripheral specifications.

- The \ref dmad_module "DMAD" provides DMA interface that can be invoked by the API. 
The driver use the routines to perform Direct Memory Access (DMA) operations. 
There are several peripherals (\ref spi_module "SPI" , \ref mcid_module "HSMCI", \ref ssc_module "SSC" , \ref AFEC_module "AFEC", \ref twi_module "TWI", \ref usart_module "USART") that can be handled to transfer between these peripherals and memory via DMA.
The programming of a device's DMA controller is hardware specific, which is described is \ref dmac_module "DMAC".
- \ref lcdd_module Implement driver functions for LCD control and image display. \ref lcdd_base implement LCD initialization, on/off and LCD back-light control. \ref lcdd_draw provide draw text, image and basic shapes (line, rectangle, circle), see detail in \ref lcd.
- The \ref sdram_module "SDRAMC" provides the detail initialization sequence to configure SDRAM using The SDRAM Controller (SDRMC).
 
\section usb_sec USB Library
\ref usb_library provides a library of highly re-usable USB driver which can be integrated in user applications.

*/

