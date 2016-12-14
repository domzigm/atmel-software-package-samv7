<?php


/**
* 
*/
class ClassMdkUvprojx extends ClassProject
{
  function __construct()
  {
    $this->toolkit = "mdk";

    $this->template_file = "\n            <File>
              <FileName>%filename_s%</FileName>
              <FileType>1</FileType>
              <FilePath>%filename%</FilePath>
            </File>";
    $this->template_group = "\n        <Group>\n          <GroupName>%groupname%</GroupName>\n          <Files>%extra_file%\n
          </Files>\n        </Group>";

    $this->file_keyword = "<FileName>";

    $this->template_inc = ";%inc_path%";
    $this->template_macro = " %macro%";

    $this->template_linkfile = array(
        array("%root_dir_i%libraries\\libboard\\%resources%\\mdk\\flash.sct",""),
        array(".\\flash.sct","",)
      );

    $this->prj_target_s = "<Target>";
    $this->prj_target_e = "</Target>";
    $this->prj_target_ins = "\n    ";
  }

  function targets_post_proc($szFilecontent, $debugger)
  {
    $arrayKey   = array("Driver.dll",     "CCJLinkInterfaceRadio",  "CCJLinkResetList");
    $arrayJlink = array("Segger\JL2CM3.dll",     "1",                      "7");
    $arrayEdbg  = array("BIN\CMSIS_AGDI.dll",  "0",                      "5");
    if(strstr($debugger, "JLINK"))
      $cfg = $arrayJlink;
    else
      $cfg = $arrayEdbg;

    foreach ($this->targetList as $target) {
      foreach ($arrayKey as $key => $value) {
        $szFilecontent = str_replace("%".$target."_".$value."%", $cfg[$key], $szFilecontent);
      }
    }

    $arrayTargetCfg = array(
        array(  "%app_TextAddressRange%", "0x00400000"),
        array("%flash_TextAddressRange%", "0x00400000"),
        array( "%qspi_TextAddressRange%", "0x00400000"),
        array( "%sram_TextAddressRange%", "0x00000000"),

        array(  "_tIfile%", "_InitializationFile%"),
        array(  "%app_InitializationFile%", ""),
        array("%flash_InitializationFile%", ""),
        array( "%qspi_InitializationFile%", ""),
        array( "%sram_InitializationFile%", "%root_dir_i%"."libraries\\libboard\\%resources%\\mdk\\%samv7%-sram.ini"),

        array(  "%app_UseTargetDll%", "1"),
        array("%flash_UseTargetDll%", "1"),
        array( "%qspi_UseTargetDll%", "1"),
        array( "%sram_UseTargetDll%", "0"),

        array(  "%app_UseExternalTool%", "0"),
        array("%flash_UseExternalTool%", "0"),
        array( "%qspi_UseExternalTool%", "0"),
        array( "%sram_UseExternalTool%", "1"),
      );
    foreach ($arrayTargetCfg as $key) {
      $szFilecontent = str_replace($key[0], $key[1], $szFilecontent);
    }
    return $szFilecontent;
  }

  function proc_directory($szFilecontent, $szPath)
  {
    if ( strstr( $szPath, "examples_usb" ) ) {
        $szFilecontent = str_replace( "%root_dir_c%", "%root_dir_c%..\\", $szFilecontent);
        $szFilecontent = str_replace( "%root_dir_i%", "%root_dir_i%..\\", $szFilecontent);
    }

    $szFilecontent = str_replace( "%prj_dir%", "", $szFilecontent);
    $szFilecontent = str_replace( "%root_dir_c%", "%root_dir_i%", $szFilecontent);
    $szFilecontent = str_replace( "%root_dir_i%", $this->PRJ_PREFIX."..\\..\\..\\..\\", $szFilecontent);

    return $szFilecontent;
  }

}

?>
