<?php


/**
* 
*/
class ClassStudioCproj extends ClassProject
{
  function __construct()
  {
    $this->toolkit = "studio";

    $this->template_file = "\n<Compile Include=\"%filename%\">
     <SubType>compile</SubType>
     <Link>%filename_s%</Link>
    </Compile>";
    $this->template_group = "%extra_file%";

    $this->file_keyword = "<Compile";

    $this->template_inc = "\n      <Value>../%inc_path%</Value>";
    $this->template_macro = "\n      <Value>%macro%</Value>";

    $this->template_linkfile = array(
        array("%chip_lowercase%_flash.ld", "../../../../../libraries/libboard/%resources%/gcc"),
        array("%chip_lowercase%_flash.ld", "."),
      );

    $this->prj_target_s = "<configuration>";
    $this->prj_target_e = "</configuration>";
    $this->prj_target_ins = "\n  ";

    $this->PRJ_PREFIX = "";
  }

  function update_configurations($szFilecontent, $szConfiguration)
  {
    $szFilecontent = str_replace("%configuration%", "flash", $szFilecontent);
    return $szFilecontent;
  }

  function proc_directory($szFilecontent, $szPath)
  {
    if ( strstr( $szPath, "examples_usb" ) ) {
      $szFilecontent = str_replace( "LIBRARIES = ../../../../libraries", "LIBRARIES = ../../../../../libraries", $szFilecontent );
      $szFilecontent = str_replace( "INCLUDES += -I../conf", "INCLUDES += -I../../conf", $szFilecontent );

      $szFilecontent = str_replace( "..\\libraries", "..\\..\\libraries", $szFilecontent );
      $szFilecontent = str_replace( "../libraries", "../../libraries", $szFilecontent );
      $szFilecontent = str_replace( "<Value>../../conf</Value>", "<Value>../../../conf</Value>", $szFilecontent );

      $szFilecontent = str_replace( "%root_dir_c%", "%root_dir_c%..\\", $szFilecontent);
      $szFilecontent = str_replace( "%root_dir_i%", "%root_dir_i%../", $szFilecontent);
    }

    $szFilecontent = str_replace( "%prj_dir%", "", $szFilecontent);
    $szFilecontent = str_replace( "%root_dir_c%", $this->PRJ_PREFIX."..\\..\\..\\..\\", $szFilecontent);
    $szFilecontent = str_replace( "%root_dir_i%", $this->PRJ_PREFIX."../../../../", $szFilecontent);

    return $szFilecontent;
  }

}

?>
