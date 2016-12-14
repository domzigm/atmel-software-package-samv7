

  !ifdef BOARD_SAMV71_XULT
    !appendfile "${EXCLUDE_FILE}" "\resources_e70$\n"
    !appendfile "${EXCLUDE_FILE}" "board_e70_xpld.h$\n"
    !appendfile "${EXCLUDE_FILE}" "\ATSAME70$\n"
    !appendfile "${EXCLUDE_FILE}" "\Trace_SAME70$\n"
    !appendfile "${EXCLUDE_FILE}" "\same70$\n"
  !else ifdef BOARD_SAME70_XPLD
    !appendfile "${EXCLUDE_FILE}" "\resources_v71$\n"
    !appendfile "${EXCLUDE_FILE}" "board_v71_xult.h$\n"
    !appendfile "${EXCLUDE_FILE}" "\ATSAMV71$\n"
    !appendfile "${EXCLUDE_FILE}" "\Trace_SAMV71$\n"
    !appendfile "${EXCLUDE_FILE}" "\samv71$\n"
  !endif

; not package S70 related files
    !appendfile "${EXCLUDE_FILE}" "\resources_s70$\n"
    !appendfile "${EXCLUDE_FILE}" "\ATSAMS70$\n"
    !appendfile "${EXCLUDE_FILE}" "\Trace_SAMS70$\n"
    !appendfile "${EXCLUDE_FILE}" "\sams70$\n"

; remove all previous files
  !if "${TOOL_CHAIN}" != IAR
    !system 'rmdir /Q /S "${BUILD_DIR}\${INSTALL_ADDPATH}\documentation"'
    !system 'rmdir /Q /S "${BUILD_DIR}\${INSTALL_ADDPATH}\examples"'
    !system 'rmdir /Q /S "${BUILD_DIR}\${INSTALL_ADDPATH}\examples_crypto"'
    !system 'rmdir /Q /S "${BUILD_DIR}\${INSTALL_ADDPATH}\examples_ethernet"'
    !system 'rmdir /Q /S "${BUILD_DIR}\${INSTALL_ADDPATH}\examples_storage"'
    !system 'rmdir /Q /S "${BUILD_DIR}\${INSTALL_ADDPATH}\examples_usb"'
    !system 'rmdir /Q /S "${BUILD_DIR}\${INSTALL_ADDPATH}\libraries"'
  !else
    !system 'rmdir /Q /S "${BUILD_DIR}"'
    !system 'xcopy toolkits\ewarm\config "${BUILD_DIR}\config\" /Q /Y /E /F /EXCLUDE:${EXCLUDE_FILE}'
    !system 'xcopy toolkits\ewarm\inc    "${BUILD_DIR}\inc\"    /Q /Y /E /F /EXCLUDE:${EXCLUDE_FILE}'
    !system 'xcopy toolkits\ewarm\src    "${BUILD_DIR}\src\"    /Q /Y /E /F /EXCLUDE:${EXCLUDE_FILE}'
  !endif


; copy examples
    !system 'xcopy documentation "${BUILD_DIR}\${INSTALL_ADDPATH}\documentation\" /Q /Y /E /F'
    !system 'xcopy examples "${BUILD_DIR}\${INSTALL_ADDPATH}\examples\" /Q /Y /E /F /EXCLUDE:${EXCLUDE_FILE}'
    !system 'xcopy examples_crypto "${BUILD_DIR}\${INSTALL_ADDPATH}\examples_crypto\" /Q /Y /E /F /EXCLUDE:${EXCLUDE_FILE}'
    !system 'xcopy examples_ethernet "${BUILD_DIR}\${INSTALL_ADDPATH}\examples_ethernet\" /Q /Y /E /F /EXCLUDE:${EXCLUDE_FILE}'
    !system 'xcopy examples_storage "${BUILD_DIR}\${INSTALL_ADDPATH}\examples_storage\" /Q /Y /E /F /EXCLUDE:${EXCLUDE_FILE}'
    !system 'xcopy examples_usb "${BUILD_DIR}\${INSTALL_ADDPATH}\examples_usb\" /Q /Y /E /F /EXCLUDE:${EXCLUDE_FILE}'

    ; copy libraries
    !system 'xcopy libraries "${BUILD_DIR}\${INSTALL_ADDPATH}\libraries\" /Q /Y /E /F /EXCLUDE:${EXCLUDE_FILE}'
    !system 'xcopy utils "${BUILD_DIR}\${INSTALL_ADDPATH}\utils\" /Q /Y /E /F /EXCLUDE:${EXCLUDE_FILE}'
 
    !system 'rmdir /Q /S "${BUILD_DIR}\${INSTALL_ADDPATH}\libraries\libchip\include\cmsis\CMSIS\Driver"'
    !system 'rmdir /Q /S "${BUILD_DIR}\${INSTALL_ADDPATH}\libraries\libchip\include\cmsis\CMSIS\DSP_Lib"'
    !system 'del /Q /S   "${BUILD_DIR}\${INSTALL_ADDPATH}\libraries\libboard\source\hamming.c"'

; copy build
    ;!system 'xcopy build "${BUILD_DIR}\${INSTALL_ADDPATH}\build\" /Q /Y /E /F /EXCLUDE:${EXCLUDE_FILE}'

    !system 'copy softpack_samv7.dir "${BUILD_DIR}\${INSTALL_ADDPATH}\"'

; delete exclusions file
    !delfile "${EXCLUDE_FILE}"
    !undef EXCLUDE_FILE

!define LOG_FILE "${SOFTPACK_DIR}/nsis_build_${PARAM}_${TOOL_CHAIN}.log"
    !system 'echo %TIME% nsis_build_${PARAM}_${TOOL_CHAIN}.log >"${LOG_FILE}"'

; run php script to re-generate all the project files
    !system 'xcopy admin\php       "${BUILD_DIR}\${INSTALL_ADDPATH}\admin\php\"       /Q /Y /E /F '
    !system 'xcopy admin\templates "${BUILD_DIR}\${INSTALL_ADDPATH}\admin\templates\" /Q /Y /E /F '
    !system 'xcopy admin\doxygen   "${BUILD_DIR}\${INSTALL_ADDPATH}\admin\doxygen\"   /Q /Y /E /F '
    !cd "${BUILD_DIR}\${INSTALL_ADDPATH}\admin\php\"
    !system 'php samv7_generate_examples.php "${PARAM}" 1>>"${LOG_FILE}" 2>&1'
    !cd "..\..\"
    !system 'rmdir /Q /S "${BUILD_DIR}\${INSTALL_ADDPATH}\admin\php"'
    !system 'rmdir /Q /S "${BUILD_DIR}\${INSTALL_ADDPATH}\admin\templates"'

; remove unreleased examples for all board
    !system 'rmdir /Q /S "${BUILD_DIR}\${INSTALL_ADDPATH}\examples\flashloader-qspi"'
  !ifdef BOARD_SAME70_XPLD
    !system 'rmdir /Q /S "${BUILD_DIR}\${INSTALL_ADDPATH}\examples\isi"'
    !system 'rmdir /Q /S "${BUILD_DIR}\${INSTALL_ADDPATH}\examples\isi_gray"'
  !endif
    !system 'rmdir /Q /S "${BUILD_DIR}\${INSTALL_ADDPATH}\examples\isi_gray_bmp"'
    !system 'rmdir /Q /S "${BUILD_DIR}\${INSTALL_ADDPATH}\examples_usb\device_examples\usb_isi_msd"'

  !if "${TOOL_CHAIN}" != IAR
    !system 'rmdir /Q /S "${BUILD_DIR}\${INSTALL_ADDPATH}\examples\fft_demo"'
  !endif

; remove unreleased examples for each board
  !ifdef BOARD_SAMV71_XULT
  !else ifdef BOARD_SAME70_XPLD
    !system 'rmdir /Q /S "${BUILD_DIR}\${INSTALL_ADDPATH}\examples\lcd_ebi"'
    !system 'rmdir /Q /S "${BUILD_DIR}\${INSTALL_ADDPATH}\examples\qspi_xip"'
    !system 'rmdir /Q /S "${BUILD_DIR}\${INSTALL_ADDPATH}\examples\ssc_dma_audio"'
    !system 'rmdir /Q /S "${BUILD_DIR}\${INSTALL_ADDPATH}\examples_storage\qspi_flash"'
    !system 'rmdir /Q /S "${BUILD_DIR}\${INSTALL_ADDPATH}\examples_usb\device_examples\usb_audio_speaker"'
    !system 'rmdir /Q /S "${BUILD_DIR}\${INSTALL_ADDPATH}\examples_usb\device_examples\usb_hid_aud"'
    !system 'rmdir /Q /S "${BUILD_DIR}\${INSTALL_ADDPATH}\examples_usb\device_examples\usb_iad_cdc_aud"'
    !system 'rmdir /Q /S "${BUILD_DIR}\${INSTALL_ADDPATH}\examples_usb\host_examples"'
  !endif

  !ifdef BOARD_SAMV71_XULT
    !system 'for /r .\ /d %i in (same70q21_*.ld)     do (del /f /q /s %i) 1>>"${LOG_FILE}" 2>&1'
  !else ifdef BOARD_SAME70_XPLD
    !system 'for /r .\ /d %i in (samv71q21_*.ld)     do (del /f /q /s %i) 1>>"${LOG_FILE}" 2>&1'
  !endif

  !if "${TOOL_CHAIN}" == IAR
    !system 'for /r .\ /d %i in (gcc, mdk, studio)     do (rmdir /q /s %i) 1>>"${LOG_FILE}" 2>&1'
  !else if "${TOOL_CHAIN}" == GCC
    !system 'for /r .\ /d %i in (ewarm, mdk, studio)   do (rmdir /q /s %i) 1>>"${LOG_FILE}" 2>&1'
    !system 'for /r .\ %i in (ExampleDirInfo.ENU.xml) do (del /f /q /s %i) 1>>"${LOG_FILE}" 2>&1'
  !else if "${TOOL_CHAIN}" == MDK
    !system 'for /r .\ /d %i in (ewarm, gcc, studio)   do (rmdir /q /s %i) 1>>"${LOG_FILE}" 2>&1'
    !system 'for /r .\ %i in (ExampleDirInfo.ENU.xml) do (del /f /q /s %i) 1>>"${LOG_FILE}" 2>&1'
  !else if "${TOOL_CHAIN}" == STUDIO
    !system 'for /r .\ /d %i in (ewarm, gcc, mdk)      do (rmdir /q /s %i) 1>>"${LOG_FILE}" 2>&1'
    !system 'for /r .\ %i in (ExampleDirInfo.ENU.xml) do (del /f /q /s %i) 1>>"${LOG_FILE}" 2>&1'
  !endif

  !if "${TOOL_CHAIN}" == STUDIO
    !cd "${SOFTPACK_DIR}"
  !ifdef BOARD_SAMV71_XULT
    !system 'xcopy libraries\libboard\resources_v71\gcc "${BUILD_DIR}\${INSTALL_ADDPATH}\libraries\libboard\resources_v71\gcc\" /Q /Y /E /F'
    !system 'xcopy libraries\libboard\resources_v71\nocache_region\gcc "${BUILD_DIR}\${INSTALL_ADDPATH}\libraries\libboard\resources_v71\nocache_region\gcc\" /Q /Y /E /F'
  !else ifdef BOARD_SAME70_XPLD
    !system 'xcopy libraries\libboard\resources_e70\gcc "${BUILD_DIR}\${INSTALL_ADDPATH}\libraries\libboard\resources_e70\gcc\" /Q /Y /E /F'
    !system 'xcopy libraries\libboard\resources_e70\nocache_region\gcc "${BUILD_DIR}\${INSTALL_ADDPATH}\libraries\libboard\resources_e70\nocache_region\gcc\" /Q /Y /E /F'
  !endif
  !endif

; generated documentation
    !cd "${BUILD_DIR}\${INSTALL_ADDPATH}\admin\doxygen\"
;    !system 'echo %cd%'
;    !system 'subst e: .'
;    !system 'pushd .'
;    !system 'cd /d e:\admin\doxygen\'
;    !system 'echo %cd%'
    !system 'doxygen Doxyfile-samv7 1>>"${LOG_FILE}" 2>&1'
;    !system 'popd'
;    !system 'subst e: /d'

; remove files not need to packed before packaging
    !cd ${BUILD_DIR}\${INSTALL_ADDPATH}
    !system 'rmdir /Q /S "${BUILD_DIR}\${INSTALL_ADDPATH}\admin"'
    !system 'for /r .\ %i in (dependency.ini, *.dir)  do (del /f /q /s %i) 1>>"${LOG_FILE}" 2>&1'

    !cd ${SOFTPACK_DIR}

; package all the header files (e70/s70/v71)
    !system 'xcopy libraries\libchip\include\same70 "${BUILD_DIR}\${INSTALL_ADDPATH}\libraries\libchip\include\same70\" /Q /Y /E /F'
    !system 'xcopy libraries\libchip\include\sams70 "${BUILD_DIR}\${INSTALL_ADDPATH}\libraries\libchip\include\sams70\" /Q /Y /E /F'
    !system 'xcopy libraries\libchip\include\samv71 "${BUILD_DIR}\${INSTALL_ADDPATH}\libraries\libchip\include\samv71\" /Q /Y /E /F'
    !system 'del /f /q /s "${BUILD_DIR}\${INSTALL_ADDPATH}\libraries\libchip\include\same70\same70.dir"'
    !system 'del /f /q /s "${BUILD_DIR}\${INSTALL_ADDPATH}\libraries\libchip\include\sams70\sams70.dir"'
    !system 'del /f /q /s "${BUILD_DIR}\${INSTALL_ADDPATH}\libraries\libchip\include\samv71\samv71.dir"'

