<?php


/**
* 
*/
class ClassIarEwp extends ClassProject
{
  function __construct()
  {
    $this->toolkit = "iar";

    $this->template_file = "\n      <file>\n        <name>%filename%</name>\n      </file>";
    $this->template_group = "\n    <group>\n      <name>%groupname%</name>%extra_file%\n    </group>";

    $this->file_keyword = "<file>";

    $this->template_inc = "\n          <state>%inc_path%</state>";
    $this->template_macro = "\n          <state>%macro%</state>";

    $this->template_linkfile = array(
        array("%root_dir_c%libraries\\libboard\\%resources%\\iar\\%chip_lowercase%_flash.icf", ""),
        array("\$PROJ_DIR\$\\config\\flash.icf",""),
      );

    $this->prj_target_s = "<configuration>";
    $this->prj_target_e = "</configuration>";
    $this->prj_target_ins = "\n  ";

    $this->PRJ_PREFIX = "\$PROJ_DIR\$";
  }

  function targets_post_proc($szFilecontent, $debugger)
  {
    $arrayKey   = array("JLINK_ID",     "CCJLinkResetList");
    $arrayJlink = array("JLINK_ID",     "7");
    $arrayEdbg  = array("CMSISDAP_ID",  "5");
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
        array(  "%app_MacFile%", "%root_dir_c%libraries\\libboard\\%resources%\\iar\\%samv7%-flash.mac"),
        array("%flash_MacFile%", "%root_dir_c%libraries\\libboard\\%resources%\\iar\\%samv7%-flash.mac"),
        array( "%qspi_MacFile%", ""),
        array( "%sram_MacFile%", "%root_dir_c%libraries\\libboard\\%resources%\\iar\\%samv7%-sram.mac"),

        array(  "%app_FlashLoadersV3%", "%flash_FlashLoadersV3%"),
        array("%flash_FlashLoadersV3%", "\$TOOLKIT_DIR\$\\config\\flashloader\\Atmel\\%chip_lowercase%\\%chip_lowercase%-flash.board"),
        array( "%qspi_FlashLoadersV3%", ""),
        array( "%sram_FlashLoadersV3%", ""),
        
        array(  "%app_UseFlashLoader%", "1"),
        array("%flash_UseFlashLoader%", "1"),
        array( "%qspi_UseFlashLoader%", "0"),
        array( "%sram_UseFlashLoader%", "0"),
      );
    foreach ($arrayTargetCfg as $key) {
      $szFilecontent = str_replace($key[0], $key[1], $szFilecontent);
    }

    return $szFilecontent;
  }

  function proc_directory($szFilecontent, $szPath)
  {
    if ( strstr( $szPath, "examples_usb" ) ) {
      if(file_exists($szPath."\\build\\ewarm\\config\\flash.icf")) {
        $szFilecontent = str_replace("..\\config\\sram.icf</",  "config\\sram.icf</", $szFilecontent );
        $szFilecontent = str_replace("..\\config\\flash.icf</", "config\\flash.icf</", $szFilecontent );
        $szFilecontent = str_replace("..\\config\\app.icf</",   "config\\app.icf</", $szFilecontent );
      }
      $szFilecontent = str_replace( "%root_dir_c%", "%root_dir_c%..\\", $szFilecontent);
      $szFilecontent = str_replace( "%root_dir_i%", "%root_dir_i%../", $szFilecontent);
    }

    $szFilecontent = str_replace( "%prj_dir%", $this->PRJ_PREFIX."/", $szFilecontent);
    $szFilecontent = str_replace( "%root_dir_c%", $this->PRJ_PREFIX."\\..\\..\\..\\..\\", $szFilecontent);
    $szFilecontent = str_replace( "%root_dir_i%", $this->PRJ_PREFIX."/../../../../", $szFilecontent);

    return $szFilecontent;
  }

}

?>
