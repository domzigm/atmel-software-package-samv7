<?php


/**
* 
*/
class ClassGccMakefile extends ClassProject
{
  function __construct()
  {
    $this->toolkit = "gcc";

    $this->template_file = "\nC_OBJECTS += %filename%";
    $this->template_group = "\n\n# source files of %groupname%%extra_file%";

    $this->file_keyword = "C_OBJECTS";

    $this->template_inc = "\nINCLUDES += -I%inc_path%";
    $this->template_macro = " -D%macro%";

    $this->template_linkfile = array(
        array("$(BOARD_LIB)/%resources%/gcc/%chip_lowercase%_$$@.ld", "$(BOARD_LIB)/%resources%/gcc"),
        array("%chip_lowercase%_$$@.ld", "./"),
      );

    $this->prj_target_s = "<configuration>";
    $this->prj_target_e = "</configuration>";
    $this->prj_target_ins = "\n  ";

    $this->PRJ_PREFIX = "";
  }

  function update_configurations($szFilecontent, $szConfiguration)
  {
    foreach ($this->targetList as $target)
      if(strstr($szConfiguration, $target))
        $szFilecontent = str_replace("%configuration%", $target." %configuration%", $szFilecontent);
    $szFilecontent = str_replace(" %configuration%", "", $szFilecontent);
    return $szFilecontent;
  }

  function proc_directory($szFilecontent, $szPath)
  {
    if ( strstr( $szPath, "examples_usb" ) ) {
      $szFilecontent = str_replace( "LIBRARIES = ../../../../libraries", "LIBRARIES = ../../../../../libraries", $szFilecontent );
      $szFilecontent = str_replace( "INCLUDES += -I../conf", "INCLUDES += -I../../conf", $szFilecontent );
      $szFilecontent = str_replace( "%root_dir_i%", "%root_dir_i%../", $szFilecontent);
    }

    $szFilecontent = str_replace( "%prj_dir%", "", $szFilecontent);
    $szFilecontent = str_replace( "%root_dir_c%", "", $szFilecontent);
    $szFilecontent = str_replace( "%root_dir_i%", "../../../../", $szFilecontent);

    return $szFilecontent;
  }

}

?>
