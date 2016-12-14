; ----------------------------------------------------------------------------
;         ATMEL Microcontroller Software Support
; ----------------------------------------------------------------------------
; Copyright (c) 2010, Atmel Corporation
;
; All rights reserved.
; ----------------------------------------------------------------------------
;
; Used variables
; $2 - Will contain studio install directory
;
;
;

;--------------------------------
; Include Modern UI

  !include "MUI2.nsh"
  !include "StrFunc.nsh"

;--------------------------------
; Define symbols for packaging

!define COMPANYNAME       "Atmel"
!define SOFTPACK_VERSION  "1.5"

!define SOFTPACK_DIR "${NSISDIR}/../.."
!define ADMIN_DIR "${NSISDIR}/../../admin"
!define DOCUMENTATION_DIR "${NSISDIR}/../../documentation"
!define DOXYGEN_DIR "${NSISDIR}/../../admin/doxygen"

;--------------------------------
; Define symbols for packaging

!ifdef BOARD_SAMV71_XULT
  !define CHIP_NAME samv71
  !define PARAM v71
  !define INSTALL_ADDPATH "examples/Atmel/SAMV71_Xplained_Ultra"
!else ifdef BOARD_SAME70_XPLD
  !define CHIP_NAME same70
  !define PARAM e70
  !define INSTALL_ADDPATH "examples/Atmel/SAME70_Xplained"
!else
  !system 'echo. & echo. & echo. & echo ====^> Usage: makensis.exe /DBOARD_SAMV71_XULT nsi_script.nsi & echo. & echo.'
  Exit with error: invalid parameters!
!endif
!define TOOL_CHAIN STUDIO


!define CS_VERSION "6.2_7.0"

!define FILE_NAME "${CHIP_NAME}_softpack_${SOFTPACK_VERSION}_for_astudio_${CS_VERSION}"
!define FILE_LABEL "Atmel ${CHIP_NAME} Softpack ${SOFTPACK_VERSION} for ATMEL studio ${CS_VERSION}"

!define BUILD_DIR "${SOFTPACK_DIR}/toolkits/studio_${PARAM}"


; The default installation directory
InstallDir "c:\temp\${FILE_NAME}\studio"

;--------------------------------
; General

; Name and file
  Name "${FILE_LABEL}"
  OutFile "${BUILD_DIR}/../${FILE_NAME}.exe"

; Request application privileges for Windows Vista
  RequestExecutionLevel user

  XPStyle on

;--------------------------------
; Variables

;--------------------------------
; Interface Settings

  !define MUI_ABORTWARNING


;--------------------------------
; Compile time instructions

; put verbose to full level
    !verbose push
    !verbose 4

; build doxygen documentation
;    !cd ${DOCUMENTATION_DIR}/html
; remove all previous files
;    !system 'del /Q installdox *.css *.gif *.png *.html *.map *.md5 *.dot'
;    !system 'rmdir /s /Q search'

;    !cd ${DOXYGEN_DIR}
;    !system 'doxygen Doxyfile 1>doxyfile_log.txt 2>&1'
;    !cd ${NSISDIR}

; copy files to toolkit folder
    !cd ${SOFTPACK_DIR}

; Create excludes file for xcopy
    !tempfile EXCLUDE_FILE

; exclude doxygen internal files
    !appendfile "${EXCLUDE_FILE}" ".map$\n"
    !appendfile "${EXCLUDE_FILE}" ".md5$\n"

; exclude useless files for gcc
    !appendfile "${EXCLUDE_FILE}" "\gcc\$\n"
    !appendfile "${EXCLUDE_FILE}" "\mdk\$\n"
    !appendfile "${EXCLUDE_FILE}" "\ewarm\$\n"
    !appendfile "${EXCLUDE_FILE}" "\benchmark\$\n"
    !appendfile "${EXCLUDE_FILE}" "\benchmarks\$\n"
    !appendfile "${EXCLUDE_FILE}" "\admin\$\n"
    !appendfile "${EXCLUDE_FILE}" "\sam-ba\$\n"
    !appendfile "${EXCLUDE_FILE}" "ExampleDirInfo.ENU.xml$\n"
    !appendfile "${EXCLUDE_FILE}" "board_cstartup_iar.c$\n"
    !appendfile "${EXCLUDE_FILE}" "board_cstartup_keil.c$\n"
    !appendfile "${EXCLUDE_FILE}" "retarget.c$\n"
    !appendfile "${EXCLUDE_FILE}" "examples.eww$\n"
    !appendfile "${EXCLUDE_FILE}" "examples_usb.eww$\n"
    !appendfile "${EXCLUDE_FILE}" ".svn$\n"
    !appendfile "${EXCLUDE_FILE}" ".TMP$\n"
    !appendfile "${EXCLUDE_FILE}" "\bin\$\n"
    !appendfile "${EXCLUDE_FILE}" "\iar\$\n"
    !appendfile "${EXCLUDE_FILE}" "\obj\$\n"
    !appendfile "${EXCLUDE_FILE}" "\sram\$\n"
    !appendfile "${EXCLUDE_FILE}" "\flash\$\n"
    !appendfile "${EXCLUDE_FILE}" "\settings\$\n"
    !appendfile "${EXCLUDE_FILE}" ".dep$\n"
    !appendfile "${EXCLUDE_FILE}" ".ewt$\n"
    !appendfile "${EXCLUDE_FILE}" "\Listings\$\n"
    !appendfile "${EXCLUDE_FILE}" "\Objects\$\n"
    !appendfile "${EXCLUDE_FILE}" "\Debug\$\n"
    !appendfile "${EXCLUDE_FILE}" ".log$\n"
    !appendfile "${EXCLUDE_FILE}" ".config$\n"
    !appendfile "${EXCLUDE_FILE}" ".atsln$\n"
    !appendfile "${EXCLUDE_FILE}" ".cproj$\n"


!include "softpack_common.nsh"


; going back to nsis dir
    !cd ${NSISDIR}

; put verbose to normal level
    !verbose pop

;--------------------------------
; Pages

  !insertmacro MUI_PAGE_WELCOME

  ; Display ATMEL disclaimer
  !insertmacro MUI_PAGE_LICENSE "${NSISDIR}\build_resources\disclaimer.txt"
  ; The second licence page is used to display the Readme.txt file
  !define MUI_PAGE_HEADER_TEXT "${CHIP_NAME} Softpack ${SOFTPACK_VERSION}"
  !define MUI_PAGE_HEADER_SUBTEXT ""
  !define MUI_LICENSEPAGE_TEXT_TOP "${FILE_LABEL}"
;  !define MUI_LICENSEPAGE_TEXT_BOTTOM "All Atmel folders will be backuped as Atmel.backup.$\nYou could revert to the original MDK state using the uninstaller."
  !define MUI_LICENSEPAGE_TEXT_BOTTOM ""
  !define MUI_LICENSEPAGE_BUTTON "Next >"

;  !insertmacro MUI_PAGE_COMPONENTS


  !insertmacro MUI_PAGE_DIRECTORY
  !insertmacro MUI_PAGE_INSTFILES
  !insertmacro MUI_PAGE_FINISH

  !insertmacro MUI_UNPAGE_CONFIRM
  !insertmacro MUI_UNPAGE_INSTFILES

;--------------------------------
; Languages

  !insertmacro MUI_LANGUAGE "English"

;--------------------------------
; Callbacks

; forbid multiple instances
Function .onInit
  System::Call 'kernel32::CreateMutexA(i 0, i 0, t "AtmelCorp_Mutex") i .r1 ?e'
  Pop $R0

  StrCmp $R0 0 end_oninit
    MessageBox MB_OK|MB_ICONEXCLAMATION "The installer is already running."
    Abort
end_oninit:
FunctionEnd

;--------------------------------
; Functions

;--------------------------------
; Installer Sections

Section "-${FILE_LABEL}" section_softpack

  SetOverwrite on

  ; Set output path to the installation directory.
  SetOutPath $INSTDIR

  ; Install new files
  File /r /x ".svn" "${BUILD_DIR}\*.*"

SectionEnd
