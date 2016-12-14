if "%1"=="v71" (
@TITLE ==== BUILD softpack for  BOARD_SAMV71_XULT ====
makensis.exe /DBOARD_SAMV71_XULT build_resources/build_ewarm_samv7.nsi
makensis.exe /DBOARD_SAMV71_XULT build_resources/build_gnu_samv7.nsi
makensis.exe /DBOARD_SAMV71_XULT build_resources/build_mdk_samv7.nsi
makensis.exe /DBOARD_SAMV71_XULT build_resources/build_studio_samv7.nsi
)

if "%1"=="e70" (
@TITLE ==== BUILD softpack for  BOARD_SAME70_XPLD ====
makensis.exe /DBOARD_SAME70_XPLD build_resources/build_ewarm_samv7.nsi
makensis.exe /DBOARD_SAME70_XPLD build_resources/build_gnu_samv7.nsi
makensis.exe /DBOARD_SAME70_XPLD build_resources/build_mdk_samv7.nsi
makensis.exe /DBOARD_SAME70_XPLD build_resources/build_studio_samv7.nsi
)
