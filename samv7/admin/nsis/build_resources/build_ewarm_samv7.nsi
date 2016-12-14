; ----------------------------------------------------------------------------
;         SAM Software Package License 
; ----------------------------------------------------------------------------
; Copyright (c) 2014, Atmel Corporation
;
; All rights reserved.
; ----------------------------------------------------------------------------
;

;--------------------------------
; Include Modern UI

  !include "MUI2.nsh"
  !include "StrFunc.nsh"
  !include "NSISArray.nsh"
  !include "WordFunc.nsh"
  
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
!define TOOL_CHAIN IAR


!define EWARM_VERSION_73 "7.30"

!define FILE_NAME "${CHIP_NAME}_softpack_${SOFTPACK_VERSION}_for_ewarm_${EWARM_VERSION_73}"
!define FILE_LABEL "${CHIP_NAME} Softpack ${SOFTPACK_VERSION} for EWARM"

!define BUILD_DIR "${SOFTPACK_DIR}/toolkits/ewarm_${PARAM}"


;--------------------------------
; General

; Name and file
  Name "${FILE_LABEL}"
  OutFile "${BUILD_DIR}/../${FILE_NAME}.exe"

;  Default installation folder
  InstallDir "C:\Temp\${FILE_NAME}"

; Request application privileges for Windows Vista
  RequestExecutionLevel user

  XPStyle on

;--------------------------------
; Variables

;--------------------------------
; Interface Settings

  !define MUI_ABORTWARNING
  !define MUI_FINISHPAGE_NOAUTOCLOSE
  !define MUI_ICON "${NSISDIR}\build_resources\images\favicon_at91.ico"
  !define MUI_UNICON "${NSISDIR}\build_resources\images\favicon_at91.ico"
  !define MUI_HEADERIMAGE
  !define MUI_HEADERIMAGE_BITMAP "${NSISDIR}\build_resources\images\Banner.bmp"

;--------------------------------
; Compile time instructions
;
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

; exclude useless files for iar 5.50
    !appendfile "${EXCLUDE_FILE}" "\gcc\$\n"
    !appendfile "${EXCLUDE_FILE}" "\mdk\$\n"
    !appendfile "${EXCLUDE_FILE}" "\benchmark\$\n"
    !appendfile "${EXCLUDE_FILE}" "\benchmarks\$\n"
    !appendfile "${EXCLUDE_FILE}" "\admin\$\n"
    !appendfile "${EXCLUDE_FILE}" "\sam-ba\$\n"
    !appendfile "${EXCLUDE_FILE}" "board_cstartup_gnu.c$\n"
    !appendfile "${EXCLUDE_FILE}" "board_cstartup_keil.c$\n"
    !appendfile "${EXCLUDE_FILE}" "syscalls.c$\n"
    !appendfile "${EXCLUDE_FILE}" "qspi_flash.icf$\n" 
    !appendfile "${EXCLUDE_FILE}" "retarget.c$\n"
    !appendfile "${EXCLUDE_FILE}" "*_gcc_dbg.a$\n"
    !appendfile "${EXCLUDE_FILE}" "*_gcc_rel.a$\n"

    !appendfile "${EXCLUDE_FILE}" ".TMP$\n"
    !appendfile "${EXCLUDE_FILE}" "\bin\$\n"
    !appendfile "${EXCLUDE_FILE}" "\obj\$\n"
    !appendfile "${EXCLUDE_FILE}" "\sram\$\n"
    !appendfile "${EXCLUDE_FILE}" "\flash\$\n"
    !appendfile "${EXCLUDE_FILE}" "\lwip_1_4_1\$\n" 
    !appendfile "${EXCLUDE_FILE}" "\settings\$\n"
    !appendfile "${EXCLUDE_FILE}" ".dep$\n"
    !appendfile "${EXCLUDE_FILE}" ".ewt$\n"
    !appendfile "${EXCLUDE_FILE}" "\Listings\$\n"
    !appendfile "${EXCLUDE_FILE}" "\Objects\$\n"
    !appendfile "${EXCLUDE_FILE}" "\validation\$\n"
    !appendfile "${EXCLUDE_FILE}" "\Debug\$\n"
    !appendfile "${EXCLUDE_FILE}" ".log$\n"
    !appendfile "${EXCLUDE_FILE}" ".config$\n"
    !appendfile "${EXCLUDE_FILE}" "\studio\$\n"


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
  !define MUI_PAGE_HEADER_SUBTEXT "Description of ${CHIP_NAME} Softpack ${SOFTPACK_VERSION}"
  !define MUI_LICENSEPAGE_TEXT_TOP "${FILE_LABEL}"
;  !define MUI_LICENSEPAGE_TEXT_BOTTOM "All Atmel folders will be backuped as Atmel.backup.$\nYou could revert to the original EWARM state using the uninstaller."
  !define MUI_LICENSEPAGE_TEXT_BOTTOM ""
  !define MUI_LICENSEPAGE_BUTTON "Next >"

;  !insertmacro MUI_PAGE_COMPONENTS

  Page custom CheckEWARM

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
;Macros to write a install log file
; Source : http://nsis.sourceforge.net/Uninstall_only_installed_files
; This allows the files from this update to be removed from all destination folders
!define UninstLog "uninstall.log"
Var UninstLog

; Uninstall log file missing.
LangString UninstLogMissing ${LANG_ENGLISH} "${UninstLog} not found!$\r$\nUninstallation cannot proceed!"

; AddItem macro
!macro AddItem Path
 FileWrite $UninstLog "${Path}$\r$\n"
!macroend
!define AddItem "!insertmacro AddItem"

; File macro
!macro File FilePath FileName
 IfFileExists "$OUTDIR\${FileName}" +2
  FileWrite $UninstLog "$OUTDIR\${FileName}$\r$\n"
 File "${FilePath}${FileName}"
!macroend
!define File "!insertmacro File"

; CreateShortcut macro
!macro CreateShortcut FilePath FilePointer
 FileWrite $UninstLog "${FilePath}$\r$\n"
 CreateShortcut "${FilePath}" "${FilePointer}"
!macroend
!define CreateShortcut "!insertmacro CreateShortcut"

; Copy files macro
!macro CopyFiles SourcePath DestPath
 IfFileExists "${DestPath}" +2
  FileWrite $UninstLog "${DestPath}$\r$\n"
 CopyFiles "${SourcePath}" "${DestPath}"
!macroend
!define CopyFiles "!insertmacro CopyFiles"

; Rename macro
!macro Rename SourcePath DestPath
 IfFileExists "${DestPath}" +2
  FileWrite $UninstLog "${DestPath}$\r$\n"
 Rename "${SourcePath}" "${DestPath}"
!macroend
!define Rename "!insertmacro Rename"

; CreateDirectory macro
!macro CreateDirectory Path
 CreateDirectory "${Path}"
 FileWrite $UninstLog "${Path}$\r$\n"
!macroend
!define CreateDirectory "!insertmacro CreateDirectory"

; SetOutPath macro
!macro SetOutPath Path
 SetOutPath "${Path}"
 FileWrite $UninstLog "${Path}$\r$\n"
!macroend
!define SetOutPath "!insertmacro SetOutPath"

; WriteUninstaller macro
!macro WriteUninstaller Path
 WriteUninstaller "${Path}"
 FileWrite $UninstLog "${Path}$\r$\n"
!macroend
!define WriteUninstaller "!insertmacro WriteUninstaller"


;--------------------------------
;Installer Sections

Section -openlogfile
 CreateDirectory "$INSTDIR"
 IfFileExists "$INSTDIR\${UninstLog}" +3
  FileOpen $UninstLog "$INSTDIR\${UninstLog}" w
 Goto +4
  SetFileAttributes "$INSTDIR\${UninstLog}" NORMAL
  FileOpen $UninstLog "$INSTDIR\${UninstLog}" a
  FileSeek $UninstLog 0 END
SectionEnd

; CleanOldReferences
Function CleanOldReferences

!ifdef BOARD_SAMV71_XULT
  Delete "$INSTDIR\arm\config\debugger\Atmel\SAMV71*.*"
  Delete "$INSTDIR\arm\config\debugger\Atmel\Trace_SAMV71"
  RMDir  /r "$INSTDIR\arm\config\devices\Atmel\SAMV71"
  RMDir  /r "$INSTDIR\arm\config\linker\Atmel\samv71j19"
  RMDir  /r "$INSTDIR\arm\config\linker\Atmel\samv71j20"
  RMDir  /r "$INSTDIR\arm\config\linker\Atmel\samv71j21"
  RMDir  /r "$INSTDIR\arm\config\linker\Atmel\samv71n19"
  RMDir  /r "$INSTDIR\arm\config\linker\Atmel\samv71n20"
  RMDir  /r "$INSTDIR\arm\config\linker\Atmel\samv71n21"
  RMDir  /r "$INSTDIR\arm\config\linker\Atmel\samv71q19"
  RMDir  /r "$INSTDIR\arm\config\linker\Atmel\samv71q20"
  RMDir  /r "$INSTDIR\arm\config\linker\Atmel\samv71q21"
  
  RMDir  /r "$INSTDIR\arm\config\flashloader\Atmel\samv71j19"
  RMDir  /r "$INSTDIR\arm\config\flashloader\Atmel\samv71j20"
  RMDir  /r "$INSTDIR\arm\config\flashloader\Atmel\samv71j21"
  RMDir  /r "$INSTDIR\arm\config\flashloader\Atmel\samv71n19"
  RMDir  /r "$INSTDIR\arm\config\flashloader\Atmel\samv71n20"
  RMDir  /r "$INSTDIR\arm\config\flashloader\Atmel\samv71n21"
  RMDir  /r "$INSTDIR\arm\config\flashloader\Atmel\samv71q19"
  RMDir  /r "$INSTDIR\arm\config\flashloader\Atmel\samv71q20"
  RMDir  /r "$INSTDIR\arm\config\flashloader\Atmel\samv71q21"
  
  RMDir  /r "$INSTDIR\arm\examples\Atmel\Atmel\SAMV71_Xplained_Ultra"
  Delete "$INSTDIR\arm\inc\Atmel\iosamv71.h"
  RMDir /r  "$INSTDIR\arm\inc\Atmel\samv71"
!else ifdef BOARD_SAME70_XPLD
  Delete "$INSTDIR\arm\config\debugger\Atmel\SAME70*.*"
  Delete "$INSTDIR\arm\config\debugger\Atmel\Trace_SAME70"
  RMDir  /r "$INSTDIR\arm\config\devices\Atmel\SAME70"
  RMDir  /r "$INSTDIR\arm\config\linker\Atmel\same70j19"
  RMDir  /r "$INSTDIR\arm\config\linker\Atmel\same70j20"
  RMDir  /r "$INSTDIR\arm\config\linker\Atmel\same70j21"
  RMDir  /r "$INSTDIR\arm\config\linker\Atmel\same70n19"
  RMDir  /r "$INSTDIR\arm\config\linker\Atmel\same70n20"
  RMDir  /r "$INSTDIR\arm\config\linker\Atmel\same70n21"
  RMDir  /r "$INSTDIR\arm\config\linker\Atmel\same70q19"
  RMDir  /r "$INSTDIR\arm\config\linker\Atmel\same70q20"
  RMDir  /r "$INSTDIR\arm\config\linker\Atmel\same70q21"
  
  RMDir  /r "$INSTDIR\arm\config\flashloader\Atmel\same70j19"
  RMDir  /r "$INSTDIR\arm\config\flashloader\Atmel\same70j20"
  RMDir  /r "$INSTDIR\arm\config\flashloader\Atmel\same70j21"
  RMDir  /r "$INSTDIR\arm\config\flashloader\Atmel\same70n19"
  RMDir  /r "$INSTDIR\arm\config\flashloader\Atmel\same70n20"
  RMDir  /r "$INSTDIR\arm\config\flashloader\Atmel\same70n21"
  RMDir  /r "$INSTDIR\arm\config\flashloader\Atmel\same70q19"
  RMDir  /r "$INSTDIR\arm\config\flashloader\Atmel\same70q20"
  RMDir  /r "$INSTDIR\arm\config\flashloader\Atmel\same70q21"
  
  RMDir  /r "$INSTDIR\arm\examples\Atmel\Atmel\SAME70_Xplained"
  Delete "$INSTDIR\arm\inc\Atmel\iosame70.h"
  RMDir /r  "$INSTDIR\arm\inc\Atmel\same70"
!endif

FunctionEnd

;--------------------------------
; Installer Sections

Section "-${FILE_LABEL}" section_softpack

  SetOverwrite on
  SetOutPath "$INSTDIR\arm"

;    ;Create uninstaller
;    ${WriteUninstaller} "$INSTDIR\${FILE_NAME}_uninstall.exe"
;
;    WriteRegStr HKLM "Software\Microsoft\Windows\CurrentVersion\Uninstall\${FILE_NAME}" \
;               "DisplayName" "${CHIP_NAME} Softpack ${SOFTPACK_VERSION} for EWARM v${EWARM_VERSION}"
;    WriteRegStr HKLM "Software\Microsoft\Windows\CurrentVersion\Uninstall\${FILE_NAME}" \
;               "UninstallString" "$INSTDIR\${FILE_NAME}_uninstall.exe"

  ; Install new files
  File /r "${BUILD_DIR}\*.*"
  ;File /r /x ".svn" "${BUILD_DIR}\*.*"
SectionEnd


;--------------------------------
;Descriptions

  ;Language strings
;  LangString DESC_SecDummy ${LANG_ENGLISH} "A test section."


;--------------------------------
;Installer Functions

var YLinePos

var HWND ; Handler number returned when creating a RadioButton

; Array used to store all EWARM install paths found
${Array} myArray 8 32
; Declare function used with array
${ArrayFunc} Read
${ArrayFunc} Push
${ArrayFunc} Clear
${ArrayFunc} Search
${ArrayFunc} Debug

; Declare String functions used here
${StrStrAdv}

Function CheckEWARM
  ;Check if the right version of EWARM is installed and fetch intall dir path from Registry

!insertmacro MUI_HEADER_TEXT "EWARM Version Check" "Get EWARM installed on your PC"

  nsDialogs::Create /NOUNLOAD 1018
  Pop $0

  ${myArray->Init}
  ${myArray->Clear}

  StrCpy $0 0
  find_version:
    EnumRegKey $1 HKLM "SOFTWARE\IAR Systems\Embedded Workbench" $0
      StrCmp $1 "5.0" list_dir
      StrCmp $1 "" no_ewarm
      IntOp $0 $0 + 1
      Goto find_version

  list_dir:
  ${NSD_CreateGroupBox} 0 0 100% 100% "EWARM installed"
  Pop $0

  ${NSD_CreateLabel} 10 18 90% 20 "The following versions of IAR EWARM for ARM have been found :"
  Pop $0

  ${NSD_CreateLabel} 10 33 90% 20 "Select the version you want to update."
  Pop $0

  ${NSD_CreateRadioButton} 20 90% 90% 10u "Extract in another folder ..."
  Pop $0
  ${NSD_OnClick} $0 OnClick
  ${myArray->Push} "$0|C:\Temp\${FILE_NAME}"

  StrCpy $0 0
  StrCpy $1 0
  StrCpy $YLinePos 50
  list_dir_loop:
    EnumRegKey $1 HKLM "SOFTWARE\IAR Systems\Embedded Workbench\5.0\Locations" $0
      ReadRegStr $2 HKLM "SOFTWARE\IAR Systems\Embedded Workbench\5.0\Locations\$1\Product Families\ARM" "10.EW"
      IfFileExists +1 continue2
      ReadRegStr $3 HKLM "SOFTWARE\IAR Systems\Embedded Workbench\5.0\Locations\$1\Product Families\ARM\10.EW" "Name"
      StrCmp $3 "EWARM" +1 continue2
      ReadRegStr $5 HKLM "SOFTWARE\IAR Systems\Embedded Workbench\5.0\Locations\$1" "InstallPath"
      IfFileExists +1 continue2
      ReadRegStr $4 HKLM "SOFTWARE\IAR Systems\Embedded Workbench\5.0\Locations\$1\Product Families\ARM\10.EW" "Version"
      ${VersionCompare} "$4" "${EWARM_VERSION_73}" $R0
      ${if} $R0 < 2
          Goto  good_version
      ${EndIf}
      
      ;StrCmp $4 ${EWARM_VERSION_72} good_version
      
      StrLen $6 $5
      ${If} $6 > 40
        StrCpy $5 $5 40 -40
        StrCpy $5 "...$5"
      ${EndIf}

      ${NSD_CreateRadioButton} 20 $YLinePos 90% 10u "$4 Cannot be updated (in $5)"
      Pop $R0
      System::Call `user32::GetWindowLong(i R0, i ${GWL_STYLE}) i.R1`
      IntOp $R1 $R1 | ${WS_DISABLED}
      IntOp $R1 $R1 | ${BS_MULTILINE}
      IntOp $R1 $R1 | ${SS_PATHELLIPSIS}
      System::Call `user32::SetWindowLong(i R0, i ${GWL_STYLE}, i R1)`
      Goto continue1

      good_version:
      ; Create a RadioButton for each version found
      ${NSD_CreateRadioButton} 20 $YLinePos 90% 10u "$4 (in $5)"
      ; Retrieve handler for this RadioButton
      Pop $HWND
      ; Create an entry (string) formatted as "HWND|<Path>" in the array
      ${myArray->Push} "$HWND|$5"
      ; Retrieve install path for array when radio button is selected
      ${NSD_OnClick} $HWND OnClick

      continue1:
      IntOp $YLinePos $YLinePos + 18

      continue2:
      StrCmp $1 "" end
      IntOp $0 $0 + 1

      Goto list_dir_loop

  no_ewarm:
    ${NSD_CreateLabel} 0 0 100% 12u "At least EWARM 7.2 should be installed"
    Pop $0
    ${NSD_CreateLabel} 0 20 100% 12u "Please select where you want the files to be extracted in the next page."
    Pop $0
    StrCpy $INSTDIR "C:\Temp\${FILE_NAME}"

  end:
    nsDialogs::Show

  ; Release memory used for array
  ${myArray->Delete}

FunctionEnd


Function OnClick
  Pop $0

  ; Variable usage
  ; $0 : Handler number (HDWN)


  ; Search for entry containing RadioButton ID
  ${myArray->Search} $1 $0 0
  ; Retrieve the string at index $1 in $2
  ${myArray->Read} $2 $1

  ; Strip the "HDWN|" sequence from the string to get only the install path
  ${StrStrAdv} $3 $2 "|" ">" ">" "0" "0" "1"

  ; Copy the install path into the default path displayed in the install page
  StrCpy $INSTDIR "$3"
  ;SetOutPath "$3/arm"
FunctionEnd




;--------------------------------
;Uninstaller Section

Section -closelogfile
  FileClose $UninstLog
  SetFileAttributes "$INSTDIR\${UninstLog}" READONLY|SYSTEM|HIDDEN
SectionEnd

Section Uninstall
  ; Can't uninstall if uninstall log is missing!
  IfFileExists "$INSTDIR\${UninstLog}" +3
    MessageBox MB_OK|MB_ICONSTOP "$(UninstLogMissing)"
    Abort

  Push $R0
  Push $R1
  Push $R2
  SetFileAttributes "$INSTDIR\${UninstLog}" NORMAL
  FileOpen $UninstLog "$INSTDIR\${UninstLog}" r
  StrCpy $R1 0

  GetLineCount:
    ClearErrors
    FileRead $UninstLog $R0
    IntOp $R1 $R1 + 1
    IfErrors 0 GetLineCount

  LoopRead:
    FileSeek $UninstLog 0 SET
    StrCpy $R2 0
    FindLine:
      FileRead $UninstLog $R0
      IntOp $R2 $R2 + 1
      StrCmp $R1 $R2 0 FindLine

    StrCpy $R0 $R0 -2
    IfFileExists "$R0\*.*" 0 +3
    RMDir $R0  #is dir
    Goto +3
    IfFileExists $R0 0 +2
    Delete $R0 #is file

  IntOp $R1 $R1 - 1
  StrCmp $R1 0 LoopDone
  Goto LoopRead

  LoopDone:
  FileClose $UninstLog
  Delete "$INSTDIR\${UninstLog}"

  Pop $R2
  Pop $R1
  Pop $R0

  Delete "$INSTDIR\${FILE_NAME}_uninstall.exe"
  DeleteRegKey HKLM "Software\Microsoft\Windows\CurrentVersion\Uninstall\${FILE_NAME}"

SectionEnd
