@echo off
echo build samav7 softpack for EWARM IAR
echo ============================================================= 
echo building afe_temp_sensor example for sram configuration
echo ============================================================= 
cd examples/afe_temp_sensor/build/ewarm
 iarbuild afe_temp_sensor.ewp -clean sram
 iarbuild afe_temp_sensor.ewp -build sram
echo ============================================================= 
echo building afe_temp_sensor example for flash configuration
echo ============================================================= 
 iarbuild afe_temp_sensor.ewp -clean flash
 iarbuild afe_temp_sensor.ewp -build flash
 cd ../../../../
echo ============================================================= 
echo building afe12_dma example for sram configuration
echo ============================================================= 
cd examples/afe12_dma/build/ewarm
 iarbuild afe12_dma.ewp -clean sram
 iarbuild afe12_dma.ewp -build sram
echo ============================================================= 
echo building afe12_dma example for flash configuration
echo ============================================================= 
 iarbuild afe12_dma.ewp -clean flash
 iarbuild afe12_dma.ewp -build flash
 cd ../../../../
echo ============================================================= 
echo building eefc_pgm example for sram configuration
echo ============================================================= 
cd examples/eefc_pgm/build/ewarm
 iarbuild eefc_pgm.ewp -clean sram
 iarbuild eefc_pgm.ewp -build sram
echo ============================================================= 
echo building eefc_pgm example for flash configuration
echo ============================================================= 
 iarbuild eefc_pgm.ewp -clean flash
 iarbuild eefc_pgm.ewp -build flash
 cd ../../../../
echo ============================================================= 
echo building fft_demo example for sram configuration
echo ============================================================= 
cd examples/fft_demo/build/ewarm
 iarbuild fft_demo.ewp -clean sram
 iarbuild fft_demo.ewp -build sram
echo ============================================================= 
echo building fft_demo example for flash configuration
echo ============================================================= 
 iarbuild fft_demo.ewp -clean flash
 iarbuild fft_demo.ewp -build flash
 cd ../../../../
echo ============================================================= 
echo building flashloader-qspi example for sram configuration
echo ============================================================= 
cd examples/flashloader-qspi/build/ewarm
 iarbuild flashloader-qspi.ewp -clean sram
 iarbuild flashloader-qspi.ewp -build sram
echo ============================================================= 
echo building flashloader-qspi example for flash configuration
echo ============================================================= 
 iarbuild flashloader-qspi.ewp -clean flash
 iarbuild flashloader-qspi.ewp -build flash
 cd ../../../../
echo ============================================================= 
echo building getting-started example for sram configuration
echo ============================================================= 
cd examples/getting-started/build/ewarm
 iarbuild getting-started.ewp -clean sram
 iarbuild getting-started.ewp -build sram
echo ============================================================= 
echo building getting-started example for flash configuration
echo ============================================================= 
 iarbuild getting-started.ewp -clean flash
 iarbuild getting-started.ewp -build flash
 cd ../../../../
echo ============================================================= 
echo building isi example for sram configuration
echo ============================================================= 
cd examples/isi/build/ewarm
 iarbuild isi.ewp -clean sram
 iarbuild isi.ewp -build sram
echo ============================================================= 
echo building isi example for flash configuration
echo ============================================================= 
 iarbuild isi.ewp -clean flash
 iarbuild isi.ewp -build flash
 cd ../../../../
echo ============================================================= 
echo building isi_gray example for sram configuration
echo ============================================================= 
cd examples/isi_gray/build/ewarm
 iarbuild isi_gray.ewp -clean sram
 iarbuild isi_gray.ewp -build sram
echo ============================================================= 
echo building isi_gray example for flash configuration
echo ============================================================= 
 iarbuild isi_gray.ewp -clean flash
 iarbuild isi_gray.ewp -build flash
 cd ../../../../
echo ============================================================= 
echo building isi_gray_bmp example for sram configuration
echo ============================================================= 
cd examples/isi_gray_bmp/build/ewarm
 iarbuild isi_gray_bmp.ewp -clean sram
 iarbuild isi_gray_bmp.ewp -build sram
echo ============================================================= 
echo building isi_gray_bmp example for flash configuration
echo ============================================================= 
 iarbuild isi_gray_bmp.ewp -clean flash
 iarbuild isi_gray_bmp.ewp -build flash
 cd ../../../../
echo ============================================================= 
echo building lcd example for sram configuration
echo ============================================================= 
cd examples/lcd/build/ewarm
 iarbuild lcd.ewp -clean sram
 iarbuild lcd.ewp -build sram
echo ============================================================= 
echo building lcd example for flash configuration
echo ============================================================= 
 iarbuild lcd.ewp -clean flash
 iarbuild lcd.ewp -build flash
 cd ../../../../
echo ============================================================= 
echo building lcd_ebi example for sram configuration
echo ============================================================= 
cd examples/lcd_ebi/build/ewarm
 iarbuild lcd_ebi.ewp -clean sram
 iarbuild lcd_ebi.ewp -build sram
echo ============================================================= 
echo building lcd_ebi example for flash configuration
echo ============================================================= 
 iarbuild lcd_ebi.ewp -clean flash
 iarbuild lcd_ebi.ewp -build flash
 cd ../../../../
echo ============================================================= 
echo building loader example for sram configuration
echo ============================================================= 
cd examples/loader/build/ewarm
 iarbuild loader.ewp -clean sram
 iarbuild loader.ewp -build sram
echo ============================================================= 
echo building loader example for flash configuration
echo ============================================================= 
 iarbuild loader.ewp -clean flash
 iarbuild loader.ewp -build flash
 cd ../../../../
echo ============================================================= 
echo building low_power example for sram configuration
echo ============================================================= 
cd examples/low_power/build/ewarm
 iarbuild low_power.ewp -clean sram
 iarbuild low_power.ewp -build sram
echo ============================================================= 
echo building low_power example for flash configuration
echo ============================================================= 
 iarbuild low_power.ewp -clean flash
 iarbuild low_power.ewp -build flash
 cd ../../../../
echo ============================================================= 
echo building mcan example for sram configuration
echo ============================================================= 
cd examples/mcan/build/ewarm
 iarbuild mcan.ewp -clean sram
 iarbuild mcan.ewp -build sram
echo ============================================================= 
echo building mcan example for flash configuration
echo ============================================================= 
 iarbuild mcan.ewp -clean flash
 iarbuild mcan.ewp -build flash
 cd ../../../../
echo ============================================================= 
echo building mpu example for sram configuration
echo ============================================================= 
cd examples/mpu/build/ewarm
 iarbuild mpu.ewp -clean sram
 iarbuild mpu.ewp -build sram
echo ============================================================= 
echo building mpu example for flash configuration
echo ============================================================= 
 iarbuild mpu.ewp -clean flash
 iarbuild mpu.ewp -build flash
 cd ../../../../
echo ============================================================= 
echo building periph_protect example for sram configuration
echo ============================================================= 
cd examples/periph_protect/build/ewarm
 iarbuild periph_protect.ewp -clean sram
 iarbuild periph_protect.ewp -build sram
echo ============================================================= 
echo building periph_protect example for flash configuration
echo ============================================================= 
 iarbuild periph_protect.ewp -clean flash
 iarbuild periph_protect.ewp -build flash
 cd ../../../../
echo ============================================================= 
echo building pmc_clock_switching example for sram configuration
echo ============================================================= 
cd examples/pmc_clock_switching/build/ewarm
 iarbuild pmc_clock_switching.ewp -clean sram
 iarbuild pmc_clock_switching.ewp -build sram
echo ============================================================= 
echo building pmc_clock_switching example for flash configuration
echo ============================================================= 
 iarbuild pmc_clock_switching.ewp -clean flash
 iarbuild pmc_clock_switching.ewp -build flash
 cd ../../../../
echo ============================================================= 
echo building pwm example for sram configuration
echo ============================================================= 
cd examples/pwm/build/ewarm
 iarbuild pwm.ewp -clean sram
 iarbuild pwm.ewp -build sram
echo ============================================================= 
echo building pwm example for flash configuration
echo ============================================================= 
 iarbuild pwm.ewp -clean flash
 iarbuild pwm.ewp -build flash
 cd ../../../../
echo ============================================================= 
echo building qspi_xip example for sram configuration
echo ============================================================= 
cd examples/qspi_xip/build/ewarm
 iarbuild qspi_xip.ewp -clean sram
 iarbuild qspi_xip.ewp -build sram
echo ============================================================= 
echo building qspi_xip example for flash configuration
echo ============================================================= 
 iarbuild qspi_xip.ewp -clean flash
 iarbuild qspi_xip.ewp -build flash
 cd ../../../../
echo ============================================================= 
echo building rtc example for sram configuration
echo ============================================================= 
cd examples/rtc/build/ewarm
 iarbuild rtc.ewp -clean sram
 iarbuild rtc.ewp -build sram
echo ============================================================= 
echo building rtc example for flash configuration
echo ============================================================= 
 iarbuild rtc.ewp -clean flash
 iarbuild rtc.ewp -build flash
 cd ../../../../
echo ============================================================= 
echo building rtt example for sram configuration
echo ============================================================= 
cd examples/rtt/build/ewarm
 iarbuild rtt.ewp -clean sram
 iarbuild rtt.ewp -build sram
echo ============================================================= 
echo building rtt example for flash configuration
echo ============================================================= 
 iarbuild rtt.ewp -clean flash
 iarbuild rtt.ewp -build flash
 cd ../../../../
echo ============================================================= 
echo building sdram example for sram configuration
echo ============================================================= 
cd examples/sdram/build/ewarm
 iarbuild sdram.ewp -clean sram
 iarbuild sdram.ewp -build sram
echo ============================================================= 
echo building sdram example for flash configuration
echo ============================================================= 
 iarbuild sdram.ewp -clean flash
 iarbuild sdram.ewp -build flash
 cd ../../../../
echo ============================================================= 
echo building spi example for sram configuration
echo ============================================================= 
cd examples/spi/build/ewarm
 iarbuild spi.ewp -clean sram
 iarbuild spi.ewp -build sram
echo ============================================================= 
echo building spi example for flash configuration
echo ============================================================= 
 iarbuild spi.ewp -clean flash
 iarbuild spi.ewp -build flash
 cd ../../../../
echo ============================================================= 
echo building ssc_dma_audio example for sram configuration
echo ============================================================= 
cd examples/ssc_dma_audio/build/ewarm
 iarbuild ssc_dma_audio.ewp -clean sram
 iarbuild ssc_dma_audio.ewp -build sram
echo ============================================================= 
echo building ssc_dma_audio example for flash configuration
echo ============================================================= 
 iarbuild ssc_dma_audio.ewp -clean flash
 iarbuild ssc_dma_audio.ewp -build flash
 cd ../../../../
echo ============================================================= 
echo building tc_capture_waveform example for sram configuration
echo ============================================================= 
cd examples/tc_capture_waveform/build/ewarm
 iarbuild tc_capture_waveform.ewp -clean sram
 iarbuild tc_capture_waveform.ewp -build sram
echo ============================================================= 
echo building tc_capture_waveform example for flash configuration
echo ============================================================= 
 iarbuild tc_capture_waveform.ewp -clean flash
 iarbuild tc_capture_waveform.ewp -build flash
 cd ../../../../
echo ============================================================= 
echo building tcm example for sram configuration
echo ============================================================= 
cd examples/tcm/build/ewarm
 iarbuild tcm.ewp -clean sram
 iarbuild tcm.ewp -build sram
echo ============================================================= 
echo building tcm example for flash configuration
echo ============================================================= 
 iarbuild tcm.ewp -clean flash
 iarbuild tcm.ewp -build flash
 cd ../../../../
echo ============================================================= 
echo building trng example for sram configuration
echo ============================================================= 
cd examples/trng/build/ewarm
 iarbuild trng.ewp -clean sram
 iarbuild trng.ewp -build sram
echo ============================================================= 
echo building trng example for flash configuration
echo ============================================================= 
 iarbuild trng.ewp -clean flash
 iarbuild trng.ewp -build flash
 cd ../../../../
echo ============================================================= 
echo building twi_eeprom example for sram configuration
echo ============================================================= 
cd examples/twi_eeprom/build/ewarm
 iarbuild twi_eeprom.ewp -clean sram
 iarbuild twi_eeprom.ewp -build sram
echo ============================================================= 
echo building twi_eeprom example for flash configuration
echo ============================================================= 
 iarbuild twi_eeprom.ewp -clean flash
 iarbuild twi_eeprom.ewp -build flash
 cd ../../../../
echo ============================================================= 
echo building twi_slave example for sram configuration
echo ============================================================= 
cd examples/twi_slave/build/ewarm
 iarbuild twi_slave.ewp -clean sram
 iarbuild twi_slave.ewp -build sram
echo ============================================================= 
echo building twi_slave example for flash configuration
echo ============================================================= 
 iarbuild twi_slave.ewp -clean flash
 iarbuild twi_slave.ewp -build flash
 cd ../../../../
echo ============================================================= 
echo building uart example for sram configuration
echo ============================================================= 
cd examples/uart/build/ewarm
 iarbuild uart.ewp -clean sram
 iarbuild uart.ewp -build sram
echo ============================================================= 
echo building uart example for flash configuration
echo ============================================================= 
 iarbuild uart.ewp -clean flash
 iarbuild uart.ewp -build flash
 cd ../../../../
echo ============================================================= 
echo building usart example for sram configuration
echo ============================================================= 
cd examples/usart/build/ewarm
 iarbuild usart.ewp -clean sram
 iarbuild usart.ewp -build sram
echo ============================================================= 
echo building usart example for flash configuration
echo ============================================================= 
 iarbuild usart.ewp -clean flash
 iarbuild usart.ewp -build flash
 cd ../../../../
echo ============================================================= 
echo building usart_7816 example for sram configuration
echo ============================================================= 
cd examples/usart_7816/build/ewarm
 iarbuild usart_7816.ewp -clean sram
 iarbuild usart_7816.ewp -build sram
echo ============================================================= 
echo building usart_7816 example for flash configuration
echo ============================================================= 
 iarbuild usart_7816.ewp -clean flash
 iarbuild usart_7816.ewp -build flash
 cd ../../../../
echo ============================================================= 
echo building usart_hw_handshaking example for sram configuration
echo ============================================================= 
cd examples/usart_hw_handshaking/build/ewarm
 iarbuild usart_hw_handshaking.ewp -clean sram
 iarbuild usart_hw_handshaking.ewp -build sram
echo ============================================================= 
echo building usart_hw_handshaking example for flash configuration
echo ============================================================= 
 iarbuild usart_hw_handshaking.ewp -clean flash
 iarbuild usart_hw_handshaking.ewp -build flash
 cd ../../../../
echo ============================================================= 
echo building usart_lon example for sram configuration
echo ============================================================= 
cd examples/usart_lon/build/ewarm
 iarbuild usart_lon.ewp -clean sram
 iarbuild usart_lon.ewp -build sram
echo ============================================================= 
echo building usart_lon example for flash configuration
echo ============================================================= 
 iarbuild usart_lon.ewp -clean flash
 iarbuild usart_lon.ewp -build flash
 cd ../../../../
echo ============================================================= 
echo building usart_rs485 example for sram configuration
echo ============================================================= 
cd examples/usart_rs485/build/ewarm
 iarbuild usart_rs485.ewp -clean sram
 iarbuild usart_rs485.ewp -build sram
echo ============================================================= 
echo building usart_rs485 example for flash configuration
echo ============================================================= 
 iarbuild usart_rs485.ewp -clean flash
 iarbuild usart_rs485.ewp -build flash
 cd ../../../../
echo ============================================================= 
echo building usart_spi example for sram configuration
echo ============================================================= 
cd examples/usart_spi/build/ewarm
 iarbuild usart_spi.ewp -clean sram
 iarbuild usart_spi.ewp -build sram
echo ============================================================= 
echo building usart_spi example for flash configuration
echo ============================================================= 
 iarbuild usart_spi.ewp -clean flash
 iarbuild usart_spi.ewp -build flash
 cd ../../../../
echo ============================================================= 
echo building wdt example for sram configuration
echo ============================================================= 
cd examples/wdt/build/ewarm
 iarbuild wdt.ewp -clean sram
 iarbuild wdt.ewp -build sram
echo ============================================================= 
echo building wdt example for flash configuration
echo ============================================================= 
 iarbuild wdt.ewp -clean flash
 iarbuild wdt.ewp -build flash
 cd ../../../../
echo ============================================================= 
echo building xdma example for sram configuration
echo ============================================================= 
cd examples/xdma/build/ewarm
 iarbuild xdma.ewp -clean sram
 iarbuild xdma.ewp -build sram
echo ============================================================= 
echo building xdma example for flash configuration
echo ============================================================= 
 iarbuild xdma.ewp -clean flash
 iarbuild xdma.ewp -build flash
 cd ../../../../
echo ============================================================= 
echo building aes example for sram configuration
echo ============================================================= 
cd examples_crypto/aes/build/ewarm
 iarbuild aes.ewp -clean sram
 iarbuild aes.ewp -build sram
echo ============================================================= 
echo building aes example for flash configuration
echo ============================================================= 
 iarbuild aes.ewp -clean flash
 iarbuild aes.ewp -build flash
 cd ../../../../
echo ============================================================= 
echo building icm example for sram configuration
echo ============================================================= 
cd examples_crypto/icm/build/ewarm
 iarbuild icm.ewp -clean sram
 iarbuild icm.ewp -build sram
echo ============================================================= 
echo building icm example for flash configuration
echo ============================================================= 
 iarbuild icm.ewp -clean flash
 iarbuild icm.ewp -build flash
 cd ../../../../
echo ============================================================= 
echo building gmac example for sram configuration
echo ============================================================= 
cd examples_ethernet/gmac/build/ewarm
 iarbuild gmac.ewp -clean sram
 iarbuild gmac.ewp -build sram
echo ============================================================= 
echo building gmac example for flash configuration
echo ============================================================= 
 iarbuild gmac.ewp -clean flash
 iarbuild gmac.ewp -build flash
 cd ../../../../
echo ============================================================= 
echo building gmac_lwip example for sram configuration
echo ============================================================= 
cd examples_ethernet/gmac_lwip/build/ewarm
 iarbuild gmac_lwip.ewp -clean sram
 iarbuild gmac_lwip.ewp -build sram
echo ============================================================= 
echo building gmac_lwip example for flash configuration
echo ============================================================= 
 iarbuild gmac_lwip.ewp -clean flash
 iarbuild gmac_lwip.ewp -build flash
 cd ../../../../
echo ============================================================= 
echo building gmac_uip_helloworld example for sram configuration
echo ============================================================= 
cd examples_ethernet/gmac_uip_helloworld/build/ewarm
 iarbuild gmac_uip_helloworld.ewp -clean sram
 iarbuild gmac_uip_helloworld.ewp -build sram
echo ============================================================= 
echo building gmac_uip_helloworld example for flash configuration
echo ============================================================= 
 iarbuild gmac_uip_helloworld.ewp -clean flash
 iarbuild gmac_uip_helloworld.ewp -build flash
 cd ../../../../
echo ============================================================= 
echo building gmac_uip_telnetd example for sram configuration
echo ============================================================= 
cd examples_ethernet/gmac_uip_telnetd/build/ewarm
 iarbuild gmac_uip_telnetd.ewp -clean sram
 iarbuild gmac_uip_telnetd.ewp -build sram
echo ============================================================= 
echo building gmac_uip_telnetd example for flash configuration
echo ============================================================= 
 iarbuild gmac_uip_telnetd.ewp -clean flash
 iarbuild gmac_uip_telnetd.ewp -build flash
 cd ../../../../
echo ============================================================= 
echo building gmac_uip_webserver example for sram configuration
echo ============================================================= 
cd examples_ethernet/gmac_uip_webserver/build/ewarm
 iarbuild gmac_uip_webserver.ewp -clean sram
 iarbuild gmac_uip_webserver.ewp -build sram
echo ============================================================= 
echo building gmac_uip_webserver example for flash configuration
echo ============================================================= 
 iarbuild gmac_uip_webserver.ewp -clean flash
 iarbuild gmac_uip_webserver.ewp -build flash
 cd ../../../../
echo ============================================================= 
echo building hsmci_multimedia_card example for sram configuration
echo ============================================================= 
cd examples_storage/hsmci_multimedia_card/build/ewarm
 iarbuild hsmci_multimedia_card.ewp -clean sram
 iarbuild hsmci_multimedia_card.ewp -build sram
echo ============================================================= 
echo building hsmci_multimedia_card example for flash configuration
echo ============================================================= 
 iarbuild hsmci_multimedia_card.ewp -clean flash
 iarbuild hsmci_multimedia_card.ewp -build flash
 cd ../../../../
echo ============================================================= 
echo building hsmci_sdcard example for sram configuration
echo ============================================================= 
cd examples_storage/hsmci_sdcard/build/ewarm
 iarbuild hsmci_sdcard.ewp -clean sram
 iarbuild hsmci_sdcard.ewp -build sram
echo ============================================================= 
echo building hsmci_sdcard example for flash configuration
echo ============================================================= 
 iarbuild hsmci_sdcard.ewp -clean flash
 iarbuild hsmci_sdcard.ewp -build flash
 cd ../../../../
echo ============================================================= 
echo building hsmci_sdcard_fatfs example for sram configuration
echo ============================================================= 
cd examples_storage/hsmci_sdcard_fatfs/build/ewarm
 iarbuild hsmci_sdcard_fatfs.ewp -clean sram
 iarbuild hsmci_sdcard_fatfs.ewp -build sram
echo ============================================================= 
echo building hsmci_sdcard_fatfs example for flash configuration
echo ============================================================= 
 iarbuild hsmci_sdcard_fatfs.ewp -clean flash
 iarbuild hsmci_sdcard_fatfs.ewp -build flash
 cd ../../../../
echo ============================================================= 
echo building hsmci_sdio example for sram configuration
echo ============================================================= 
cd examples_storage/hsmci_sdio/build/ewarm
 iarbuild hsmci_sdio.ewp -clean sram
 iarbuild hsmci_sdio.ewp -build sram
echo ============================================================= 
echo building hsmci_sdio example for flash configuration
echo ============================================================= 
 iarbuild hsmci_sdio.ewp -clean flash
 iarbuild hsmci_sdio.ewp -build flash
 cd ../../../../
echo ============================================================= 
echo building qspi_flash example for sram configuration
echo ============================================================= 
cd examples_storage/qspi_flash/build/ewarm
 iarbuild qspi_flash.ewp -clean sram
 iarbuild qspi_flash.ewp -build sram
echo ============================================================= 
echo building qspi_flash example for flash configuration
echo ============================================================= 
 iarbuild qspi_flash.ewp -clean flash
 iarbuild qspi_flash.ewp -build flash
 cd ../../../../
echo ============================================================= 
echo building usb_audio_looprec example for sram configuration
echo ============================================================= 
cd examples_usb\device_examples/usb_audio_looprec/build/ewarm
 iarbuild usb_audio_looprec.ewp -clean sram
 iarbuild usb_audio_looprec.ewp -build sram
echo ============================================================= 
echo building usb_audio_looprec example for flash configuration
echo ============================================================= 
 iarbuild usb_audio_looprec.ewp -clean flash
 iarbuild usb_audio_looprec.ewp -build flash
 cd ../../../../
echo ============================================================= 
echo building usb_audio_speaker example for sram configuration
echo ============================================================= 
cd examples_usb\device_examples/usb_audio_speaker/build/ewarm
 iarbuild usb_audio_speaker.ewp -clean sram
 iarbuild usb_audio_speaker.ewp -build sram
echo ============================================================= 
echo building usb_audio_speaker example for flash configuration
echo ============================================================= 
 iarbuild usb_audio_speaker.ewp -clean flash
 iarbuild usb_audio_speaker.ewp -build flash
 cd ../../../../
echo ============================================================= 
echo building usb_cdc example for sram configuration
echo ============================================================= 
cd examples_usb\device_examples/usb_cdc/build/ewarm
 iarbuild usb_cdc.ewp -clean sram
 iarbuild usb_cdc.ewp -build sram
echo ============================================================= 
echo building usb_cdc example for flash configuration
echo ============================================================= 
 iarbuild usb_cdc.ewp -clean flash
 iarbuild usb_cdc.ewp -build flash
 cd ../../../../
echo ============================================================= 
echo building usb_core example for sram configuration
echo ============================================================= 
cd examples_usb\device_examples/usb_core/build/ewarm
 iarbuild usb_core.ewp -clean sram
 iarbuild usb_core.ewp -build sram
echo ============================================================= 
echo building usb_core example for flash configuration
echo ============================================================= 
 iarbuild usb_core.ewp -clean flash
 iarbuild usb_core.ewp -build flash
 cd ../../../../
echo ============================================================= 
echo building usb_eem example for sram configuration
echo ============================================================= 
cd examples_usb\device_examples/usb_eem/build/ewarm
 iarbuild usb_eem.ewp -clean sram
 iarbuild usb_eem.ewp -build sram
echo ============================================================= 
echo building usb_eem example for flash configuration
echo ============================================================= 
 iarbuild usb_eem.ewp -clean flash
 iarbuild usb_eem.ewp -build flash
 cd ../../../../
echo ============================================================= 
echo building usb_hid_aud example for sram configuration
echo ============================================================= 
cd examples_usb\device_examples/usb_hid_aud/build/ewarm
 iarbuild usb_hid_aud.ewp -clean sram
 iarbuild usb_hid_aud.ewp -build sram
echo ============================================================= 
echo building usb_hid_aud example for flash configuration
echo ============================================================= 
 iarbuild usb_hid_aud.ewp -clean flash
 iarbuild usb_hid_aud.ewp -build flash
 cd ../../../../
echo ============================================================= 
echo building usb_hid_keyboard example for sram configuration
echo ============================================================= 
cd examples_usb\device_examples/usb_hid_keyboard/build/ewarm
 iarbuild usb_hid_keyboard.ewp -clean sram
 iarbuild usb_hid_keyboard.ewp -build sram
echo ============================================================= 
echo building usb_hid_keyboard example for flash configuration
echo ============================================================= 
 iarbuild usb_hid_keyboard.ewp -clean flash
 iarbuild usb_hid_keyboard.ewp -build flash
 cd ../../../../
echo ============================================================= 
echo building usb_hid_mouse example for sram configuration
echo ============================================================= 
cd examples_usb\device_examples/usb_hid_mouse/build/ewarm
 iarbuild usb_hid_mouse.ewp -clean sram
 iarbuild usb_hid_mouse.ewp -build sram
echo ============================================================= 
echo building usb_hid_mouse example for flash configuration
echo ============================================================= 
 iarbuild usb_hid_mouse.ewp -clean flash
 iarbuild usb_hid_mouse.ewp -build flash
 cd ../../../../
echo ============================================================= 
echo building usb_hid_msd example for sram configuration
echo ============================================================= 
cd examples_usb\device_examples/usb_hid_msd/build/ewarm
 iarbuild usb_hid_msd.ewp -clean sram
 iarbuild usb_hid_msd.ewp -build sram
echo ============================================================= 
echo building usb_hid_msd example for flash configuration
echo ============================================================= 
 iarbuild usb_hid_msd.ewp -clean flash
 iarbuild usb_hid_msd.ewp -build flash
 cd ../../../../
echo ============================================================= 
echo building usb_hid_transfer example for sram configuration
echo ============================================================= 
cd examples_usb\device_examples/usb_hid_transfer/build/ewarm
 iarbuild usb_hid_transfer.ewp -clean sram
 iarbuild usb_hid_transfer.ewp -build sram
echo ============================================================= 
echo building usb_hid_transfer example for flash configuration
echo ============================================================= 
 iarbuild usb_hid_transfer.ewp -clean flash
 iarbuild usb_hid_transfer.ewp -build flash
 cd ../../../../
echo ============================================================= 
echo building usb_iad_cdc_aud example for sram configuration
echo ============================================================= 
cd examples_usb\device_examples/usb_iad_cdc_aud/build/ewarm
 iarbuild usb_iad_cdc_aud.ewp -clean sram
 iarbuild usb_iad_cdc_aud.ewp -build sram
echo ============================================================= 
echo building usb_iad_cdc_aud example for flash configuration
echo ============================================================= 
 iarbuild usb_iad_cdc_aud.ewp -clean flash
 iarbuild usb_iad_cdc_aud.ewp -build flash
 cd ../../../../
echo ============================================================= 
echo building usb_iad_cdc_cdc example for sram configuration
echo ============================================================= 
cd examples_usb\device_examples/usb_iad_cdc_cdc/build/ewarm
 iarbuild usb_iad_cdc_cdc.ewp -clean sram
 iarbuild usb_iad_cdc_cdc.ewp -build sram
echo ============================================================= 
echo building usb_iad_cdc_cdc example for flash configuration
echo ============================================================= 
 iarbuild usb_iad_cdc_cdc.ewp -clean flash
 iarbuild usb_iad_cdc_cdc.ewp -build flash
 cd ../../../../
echo ============================================================= 
echo building usb_iad_cdc_hid example for sram configuration
echo ============================================================= 
cd examples_usb\device_examples/usb_iad_cdc_hid/build/ewarm
 iarbuild usb_iad_cdc_hid.ewp -clean sram
 iarbuild usb_iad_cdc_hid.ewp -build sram
echo ============================================================= 
echo building usb_iad_cdc_hid example for flash configuration
echo ============================================================= 
 iarbuild usb_iad_cdc_hid.ewp -clean flash
 iarbuild usb_iad_cdc_hid.ewp -build flash
 cd ../../../../
echo ============================================================= 
echo building usb_iad_cdc_msd example for sram configuration
echo ============================================================= 
cd examples_usb\device_examples/usb_iad_cdc_msd/build/ewarm
 iarbuild usb_iad_cdc_msd.ewp -clean sram
 iarbuild usb_iad_cdc_msd.ewp -build sram
echo ============================================================= 
echo building usb_iad_cdc_msd example for flash configuration
echo ============================================================= 
 iarbuild usb_iad_cdc_msd.ewp -clean flash
 iarbuild usb_iad_cdc_msd.ewp -build flash
 cd ../../../../
echo ============================================================= 
echo building usb_isi_msd example for sram configuration
echo ============================================================= 
cd examples_usb\device_examples/usb_isi_msd/build/ewarm
 iarbuild usb_isi_msd.ewp -clean sram
 iarbuild usb_isi_msd.ewp -build sram
echo ============================================================= 
echo building usb_isi_msd example for flash configuration
echo ============================================================= 
 iarbuild usb_isi_msd.ewp -clean flash
 iarbuild usb_isi_msd.ewp -build flash
 cd ../../../../
echo ============================================================= 
echo building usb_massstorage example for sram configuration
echo ============================================================= 
cd examples_usb\device_examples/usb_massstorage/build/ewarm
 iarbuild usb_massstorage.ewp -clean sram
 iarbuild usb_massstorage.ewp -build sram
echo ============================================================= 
echo building usb_massstorage example for flash configuration
echo ============================================================= 
 iarbuild usb_massstorage.ewp -clean flash
 iarbuild usb_massstorage.ewp -build flash
 cd ../../../../
echo ============================================================= 
echo building usb_video example for sram configuration
echo ============================================================= 
cd examples_usb\device_examples/usb_video/build/ewarm
 iarbuild usb_video.ewp -clean sram
 iarbuild usb_video.ewp -build sram
echo ============================================================= 
echo building usb_video example for flash configuration
echo ============================================================= 
 iarbuild usb_video.ewp -clean flash
 iarbuild usb_video.ewp -build flash
 cd ../../../../
echo ============================================================= 
echo building usb_video_gray example for sram configuration
echo ============================================================= 
cd examples_usb\device_examples/usb_video_gray/build/ewarm
 iarbuild usb_video_gray.ewp -clean sram
 iarbuild usb_video_gray.ewp -build sram
echo ============================================================= 
echo building usb_video_gray example for flash configuration
echo ============================================================= 
 iarbuild usb_video_gray.ewp -clean flash
 iarbuild usb_video_gray.ewp -build flash
 cd ../../../../
echo ============================================================= 
echo building usb_host_cdc example for sram configuration
echo ============================================================= 
cd examples_usb\host_examples/usb_host_cdc/build/ewarm
 iarbuild usb_host_cdc.ewp -clean sram
 iarbuild usb_host_cdc.ewp -build sram
echo ============================================================= 
echo building usb_host_cdc example for flash configuration
echo ============================================================= 
 iarbuild usb_host_cdc.ewp -clean flash
 iarbuild usb_host_cdc.ewp -build flash
 cd ../../../../
echo ============================================================= 
echo building usb_host_hid example for sram configuration
echo ============================================================= 
cd examples_usb\host_examples/usb_host_hid/build/ewarm
 iarbuild usb_host_hid.ewp -clean sram
 iarbuild usb_host_hid.ewp -build sram
echo ============================================================= 
echo building usb_host_hid example for flash configuration
echo ============================================================= 
 iarbuild usb_host_hid.ewp -clean flash
 iarbuild usb_host_hid.ewp -build flash
 cd ../../../../
echo ============================================================= 
echo building usb_host_msc example for sram configuration
echo ============================================================= 
cd examples_usb\host_examples/usb_host_msc/build/ewarm
 iarbuild usb_host_msc.ewp -clean sram
 iarbuild usb_host_msc.ewp -build sram
echo ============================================================= 
echo building usb_host_msc example for flash configuration
echo ============================================================= 
 iarbuild usb_host_msc.ewp -clean flash
 iarbuild usb_host_msc.ewp -build flash
 cd ../../../../
echo ============================================================= 
echo building usb_host_msc_hid example for sram configuration
echo ============================================================= 
cd examples_usb\host_examples/usb_host_msc_hid/build/ewarm
 iarbuild usb_host_msc_hid.ewp -clean sram
 iarbuild usb_host_msc_hid.ewp -build sram
echo ============================================================= 
echo building usb_host_msc_hid example for flash configuration
echo ============================================================= 
 iarbuild usb_host_msc_hid.ewp -clean flash
 iarbuild usb_host_msc_hid.ewp -build flash
 cd ../../../../
