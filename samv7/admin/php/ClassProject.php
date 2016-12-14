<?php


/**
* 
*/
class ClassProject
{
  var $toolkit;
  public $targetList = array("sram", "flash", "app", "qspi");

  var $template_file;
  var $template_group;
  var $template_inc;
  var $file_keyword;
  var $template_linkfile;

  var $prj_target_s;
  var $prj_target_e;
  var $prj_target_ins;

  var $PRJ_PREFIX;

  var $arraySotrageFiles;

  function __construct($argument)
  {
    $this->PRJ_PREFIX = "";
  }

  function process($szFilecontent, $szPath)
  {
    global $boardInfo;
    $arrayConfig = array(
        array("getting-started", "sram flash qspi"),
        array("fft_demo",        "flash app"),
        array("usb_isi_msd",     "flash"),
        array("usb_video_gray",  "sram flash"),
        array("usb_video",       "sram flash app"),
      );
    $exampleCfg = "sram flash";
    foreach ($arrayConfig as $cfg) {
      if (strstr($szPath, $cfg[0])) {
        $exampleCfg = $cfg[1];
        break;
      }
    }
    $szFilecontent = $this->update_configurations($szFilecontent, $exampleCfg);

    $exampleDbg = "CMSISDAP";
    $arrayDebugger = array( "fft_demo", "sdram", "qspi_xip", "icm", "gmac", "loader",
      "isi", "usb_hid_keyboard","usb_hid_msd","usb_hid_transfer", "usb_hid_mouse", "usb_video");
    foreach ($arrayDebugger as $cfg) {
      if (strstr($szPath, $cfg)) {
        $exampleDbg = "JLINK";
        break;
      }
    }
    $szFilecontent = $this->targets_post_proc($szFilecontent, $exampleDbg);

    $szFilecontent = str_replace("%board%", $boardInfo[2], $szFilecontent);
    $szFilecontent = str_replace("%chip%",  $boardInfo[3], $szFilecontent);

    $exampleConfig = get_configurations($szPath, $this->toolkit);

    /***************************************************************************
         process linker script files
    ***************************************************************************/
    if( file_exists($szPath."\\build\\ewarm\\config\\flash.icf") ||
        file_exists($szPath."\\build\\gcc\\sam_flash.ld") ||
        file_exists($szPath."\\build\\mdk\\flash.sct") ||
        file_exists($szPath."\\build\\studio\\sam_flash.ld")
       ) {
      $flash_icf = $this->template_linkfile[1][0];
      $path_icf = $this->template_linkfile[1][1];
      if ( strstr($this->toolkit, "studio") ) $path_icf .=".";
    } else {
      $flash_icf = $this->template_linkfile[0][0];
      $path_icf = $this->template_linkfile[0][1];
    }

    foreach ($exampleConfig->predefine as $macro) {
      if(strstr($macro, "MPU_HAS_NOCACHE_REGION")){
        $flash_icf = str_replace("%resources%\\", "%resources%\\nocache_region\\", $flash_icf);
        $flash_icf = str_replace("%resources%/",  "%resources%/nocache_region/", $flash_icf);
        $path_icf = str_replace("%resources%\\", "%resources%\\nocache_region\\", $path_icf);
        $path_icf = str_replace("%resources%/",  "%resources%/nocache_region/", $path_icf);
        break;
      }
    }

    $szFilecontent = str_replace("%linker_script_path%", $path_icf, $szFilecontent);

    $szFilecontent = str_replace("%flash_icf%", $flash_icf, $szFilecontent);
    $sram_icf  = str_replace("flash", "sram", $flash_icf);
    $szFilecontent = str_replace("%sram_icf%",  $sram_icf,  $szFilecontent);
    $app_icf  = str_replace("flash", "app",   $flash_icf);
    $szFilecontent = str_replace("%app_icf%",    $app_icf,  $szFilecontent);
    $qspi_icf = str_replace("flash", "qspi",  $flash_icf);
    $szFilecontent = str_replace("%qspi_icf%",  $qspi_icf,  $szFilecontent);

    $t_icf = "\r    <file>\n      <name>%file%</name>\n    </file>";

    $icf_file = "";
    if ($sram_icf  && strstr($exampleCfg, "sram"))  $icf_file .= str_replace("%file%", $sram_icf,  $t_icf);
    if ($flash_icf && strstr($exampleCfg, "flash")) $icf_file .= str_replace("%file%", $flash_icf, $t_icf);
    if ($app_icf   && strstr($exampleCfg, "app"))   $icf_file .= str_replace("%file%", $app_icf,   $t_icf);
    if ($qspi_icf  && strstr($exampleCfg, "qspi"))  $icf_file .= str_replace("%file%", $qspi_icf,  $t_icf);
    $szFilecontent = str_replace("%icf_file%", $icf_file, $szFilecontent);


    /***************************************************************************
         process predefined macros
    ***************************************************************************/
    $traceLevelConfig = array(
        array("usb_isi_msd",     "2"),
        array("usb_massstorage", "2"),
        array("usb_hid_msd",     "2"),
        array("usb_iad_cdc_msd", "2"),
      );
    $traceLevel = "4";
    foreach ($traceLevelConfig as $cfg) {
      if (strstr($szPath, $cfg[0])) {
        $traceLevel = $cfg[1];
        break;
      }
    }
    if (strstr($this->toolkit, "gcc")) {
      $szFilecontent = str_replace( "%gcc_TRACE_LEVEL%", $traceLevel, $szFilecontent);
    } else {
      $exampleConfig->predefine[] = "TRACE_LEVEL=".$traceLevel;
    }
    foreach ($exampleConfig->predefine as $macro) {
      $szFilecontent = str_replace( "%predefine%",
          str_replace("%macro%", $macro, $this->template_macro)."%predefine%",
          $szFilecontent);
    }

    /***************************************************************************
         process additional source files
    ***************************************************************************/
    $libusb    = str_replace("%groupname%", "libusb",    $this->template_group);
    $libfatfs  = str_replace("%groupname%", "libfatfs",  $this->template_group);
    $libmemory = str_replace("%groupname%", "libmemory", $this->template_group);
    $liblwip   = str_replace("%groupname%", "liblwip",   $this->template_group);
    $libuip    = str_replace("%groupname%", "libuip",    $this->template_group);
    $libjpeg   = str_replace("%groupname%", "libjpeg",   $this->template_group);

    $inc_path = "";
    if(strstr($szPath, "flashloader-qspi"))  {
      $inc_path .= str_replace("%inc_path%",
              "%root_dir_i%libraries/benchmark/dhrystone",
              $this->template_inc);;
    }

    $gcc_lib_dir = "";
    $gcc_vpath = "";

    foreach ($exampleConfig->files as $file) {
       if ( strstr($szPath, "examples_usb") ) {
         $file = substr($file, 3);
      }
      if ( strstr($file, "arm_common_tables.c") ) continue;
      if (strstr($this->toolkit, "mdk")) {
        if ( strstr($file, ".h") ) continue;
      }
      if (strstr($this->toolkit, "gcc")) {
        if ( strstr($file, ".h") ) continue;
        $file = str_replace(".c", ".o", $file);
      }

      $pos = strrpos($file, "\\");
      $path = substr($file, 0, $pos);
      if( strstr($this->toolkit, "mdk") ) {
        $filename_s = substr($file, $pos+1);
      }elseif( strstr($this->toolkit, "gcc") ) {
        $file = substr($file, $pos+1);
        $filename_s = substr($file, strrpos($file, "\\") + 1);
      } elseif (strstr($this->toolkit, "studio")) {
        $filename_s = substr($file, strrpos($file, "..\\") + 3);
      }

      $file = str_replace("..\\..\\", "", $file);

      if ( strstr($path, "libboard") ) {
        $szFilecontent=str_replace( '%libboard_file%',
            str_replace("%filename%", "%root_dir_c%".$file, $this->template_file)."%libboard_file%", 
            $szFilecontent ) ;
      } elseif ( strstr($path, "libchip") ) {
        $szFilecontent=str_replace( '%libchip_file%',
            str_replace("%filename%", "%root_dir_c%".$file, $this->template_file)."%libchip_file%", 
            $szFilecontent ) ;
      } elseif ( strstr($path, "libusb") ) {
        $tmp = str_replace("\n", "\n  ", $this->template_file);
        $libusb = str_replace("%extra_file%",
            str_replace("%filename%", "%root_dir_c%".$file, $tmp)."%extra_file%",
            $libusb);

        $inc_path = $this->add_inc_path_4_usb_host($inc_path, $szPath);

        if ( false == strstr($gcc_lib_dir, "libusb") ) {
          $gcc_lib_dir .= "\n# USB library directory\nUSB_LIB = \$(LIBRARIES)/libusb\n";
          $usb_inc_path = array(
              "%root_dir_i%libraries/libusb",
              "%root_dir_i%libraries/libusb/include",
              );
          foreach ($usb_inc_path as $tmpPath) {
            $inc_path .= str_replace("%inc_path%", $tmpPath, $this->template_inc);
          }
          if( strstr($this->toolkit, "gcc") ) 
            $inc_path = str_replace("%root_dir_i%libraries/libusb", "\$(USB_LIB)", $inc_path);
        }
        if ( false == strstr($gcc_vpath, $path."%end%") ) {
          $gcc_vpath .= "\nVPATH += ".$path."%end%";
        }
      } elseif ( strstr($path, "libfatfs" )) {
        $tmp = str_replace("\n", "\n  ", $this->template_file);
        $libfatfs = str_replace("%extra_file%",
            str_replace("%filename%", "%root_dir_c%".$file, $tmp)."%extra_file%",
            $libfatfs);
        if(false == strstr($gcc_lib_dir, "libfatfs")) {
          $gcc_lib_dir .= "\n# FATFS library directory\nFAT_LIB = \$(LIBRARIES)/libfatfs\n";
          $fatfs_inc_path = array(
              "%root_dir_i%libraries/libfatfs",
              "%root_dir_i%libraries/libfatfs/inc",
              "%root_dir_i%libraries/libfatfs/src",
          );
          foreach ($fatfs_inc_path as $tmpPath) {
            $inc_path .= str_replace("%inc_path%", $tmpPath, $this->template_inc);
          }
          if( strstr($this->toolkit, "gcc") ) 
            $inc_path = str_replace("%root_dir_i%libraries/libfatfs", "\$(FAT_LIB)", $inc_path);
          $gcc_vpath .= "\nVPATH += \$(FAT_LIB)/src"."\nVPATH += \$(FAT_LIB)/src/option";
        }
     } elseif (strstr($path, "libstoragemedia")) {
        $tmp = str_replace("\n", "\n  ", $this->template_file);
        $libmemory = str_replace("%extra_file%",
            str_replace("%filename%", "%root_dir_c%".$file, $tmp)."%extra_file%",
            $libmemory);
        // add include path
        if(false == strstr($gcc_lib_dir, "libstoragemedia")) {
          $gcc_lib_dir .= "\n# storage library directory\nSTORAGE_LIB = \$(LIBRARIES)/libstoragemedia\n";
          $memory_inc_path = array(
              "%root_dir_i%libraries/libstoragemedia",
              "%root_dir_i%libraries/libstoragemedia/include",
              "%root_dir_i%libraries/libstoragemedia/include/sdmmc",
          );
          foreach ($memory_inc_path as $tmpPath) {
            $inc_path .= str_replace("%inc_path%", $tmpPath, $this->template_inc);
          }
          if( strstr($this->toolkit, "gcc") ) 
            $inc_path = str_replace("%root_dir_i%libraries/libstoragemedia", "\$(STORAGE_LIB)", $inc_path);
          $gcc_vpath .= "\nVPATH += \$(STORAGE_LIB)/source"."\nVPATH += \$(STORAGE_LIB)/source/sdmmc";
        }
     } elseif (strstr($path, "liblwip")) {
        $tmp = str_replace("\n", "\n  ", $this->template_file);
        $liblwip = str_replace("%extra_file%",
            str_replace("%filename%", "%root_dir_c%".$file, $tmp)."%extra_file%",
            $liblwip);
        if(false == strstr($gcc_lib_dir, "liblwip")) {
          $gcc_lib_dir .= "\n# LWIP library directory\nLWIP_LIB = \$(LIBRARIES)/liblwip\n";
          $lwip_inc_path = array(
              "%root_dir_i%libraries/liblwip/source/lwip_1.3.2/src/include",
              "%root_dir_i%libraries/liblwip/source/lwip_1.3.2/src/include/lwip",
              "%root_dir_i%libraries/liblwip/source/lwip_1.3.2/src/include/ipv4",
              "%root_dir_i%libraries/liblwip/source/lwip_1.3.2/src/include/netif",
              "%root_dir_i%libraries/liblwip/source/samv7-specific",
              "%root_dir_i%libraries/liblwip",
          );
          foreach ($lwip_inc_path as $tmpPath) {
            $inc_path .= str_replace("%inc_path%", $tmpPath, $this->template_inc);
          }
          if( strstr($this->toolkit, "gcc") )
            $inc_path = str_replace("%root_dir_i%libraries/liblwip", "\$(LWIP_LIB)", $inc_path);
          $gcc_vpath .= "\nVPATH += \$(LWIP_LIB)/source/lwip_1.3.2/src/api".
              "\nVPATH += \$(LWIP_LIB)/source/lwip_1.3.2/src/core/ipv4".
              "\nVPATH += \$(LWIP_LIB)/source/lwip_1.3.2/src/core/snmp".
              "\nVPATH += \$(LWIP_LIB)/source/lwip_1.3.2/src/core".
              "\nVPATH += \$(LWIP_LIB)/source/lwip_1.3.2/src/netif".
              "\nVPATH += \$(LWIP_LIB)/source/samv7-specific".
              "\nVPATH += \$(LWIP_LIB)/source/samv7-specific/arch";
        }
     } elseif (strstr($path, "libuip")) {
        $tmp = str_replace("\n", "\n  ", $this->template_file);
        $libuip = str_replace("%extra_file%",
            str_replace("%filename%", "%root_dir_c%".$file, $tmp)."%extra_file%",
            $libuip);
        if(false == strstr($gcc_lib_dir, "libuip")) {
          $gcc_lib_dir .= "\n# UIP library directory\nUIP_LIB = \$(LIBRARIES)/libuip\n";
          $uip_inc_path = array(
              "%root_dir_i%libraries/libuip/source/samv7-specific",
              "%root_dir_i%libraries/libuip/source/uip_1.0",
              "%root_dir_i%libraries/libuip/source/uip_1.0/apps/dhcpc",
              "%root_dir_i%libraries/libuip/source/uip_1.0/lib",
              "%root_dir_i%libraries/libuip/source/uip_1.0/uip",
              "%root_dir_i%libraries/libuip/source/uip_1.0/apps/%appInc%",
          );
          foreach ($uip_inc_path as $tmpPath) {
            $inc_path .= str_replace("%inc_path%", $tmpPath, $this->template_inc);
          }
          if( strstr($this->toolkit, "gcc") )
            $inc_path = str_replace("%root_dir_i%libraries/libuip", "\$(UIP_LIB)", $inc_path);
          $gcc_vpath .= "\nVPATH += \$(UIP_LIB)/source/samv7-specific".
              "\nVPATH += \$(UIP_LIB)/source/uip_1.0/uip".
              "\nVPATH += \$(UIP_LIB)/source/uip_1.0/apps/%appInc%".
              "\nVPATH += \$(UIP_LIB)/source/uip_1.0/lib".
              "\nVPATH += \$(UIP_LIB)/source/uip_1.0/uip";
        }
     } elseif (strstr($path, "libjpeg")) {
        $tmp = str_replace("\n", "\n  ", $this->template_file);
        $libjpeg = str_replace("%extra_file%",
            str_replace("%filename%", "%root_dir_c%".$file, $tmp)."%extra_file%",
            $libjpeg);
        if(false == strstr($gcc_lib_dir, "libjpeg")) {
          $gcc_lib_dir .= "\n# JEPG library directory\nJPEG_LIB = \$(LIBRARIES)/libjpeg\n";
          $uip_inc_path = array(
              "%root_dir_i%libraries/libjpeg",
              "%root_dir_i%libraries/libjpeg/include",
          );
          foreach ($uip_inc_path as $tmpPath) {
            $inc_path .= str_replace("%inc_path%", $tmpPath, $this->template_inc);
          }
          if( strstr($this->toolkit, "gcc") )
            $inc_path = str_replace("%root_dir_i%libraries/libjpeg", "\$(JPEG_LIB)", $inc_path);
          $gcc_vpath .= "\nVPATH += \$(JPEG_LIB)".
              "\nVPATH += \$(JPEG_LIB)/source";
        }
      } elseif(strstr($path, "benchmark")) {
        $szFilecontent=str_replace( '%extra_file%',
            str_replace("%filename%", "%root_dir_c%".$file, $this->template_file)."%extra_file%", 
            $szFilecontent ) ;
        if(false == strstr($gcc_vpath, "benchmark"))
          $gcc_vpath .= "\nVPATH += $(LIBRARIES)/benchmark/dhrystone";
      } elseif(strstr($path, "utils\\")) {
        $szFilecontent=str_replace( '%extra_file%',
            str_replace("%filename%", "%root_dir_c%".$file, $this->template_file)."%extra_file%", 
            $szFilecontent ) ;
        $gcc_vpath .= "\nVPATH += %root_dir_i%utils/md5";
      } else {
        $adjust_p = "..\\";
        if ( strstr($szPath, "examples_usb") ) $adjust_p = "";
        $tmp = $this->PRJ_PREFIX."\\..\\".$adjust_p.$file;
        if( strstr($this->toolkit, "mdk") ) $tmp = "..\\".$adjust_p.$file;
        if( strstr($this->toolkit, "gcc") ) $tmp = $file;
        if( strstr($this->toolkit, "studio") ) $tmp = "..\\..\\".$file;
        $szFilecontent=str_replace( '%extra_file%',
            str_replace("%filename%", $tmp, $this->template_file)."%extra_file%", 
            $szFilecontent ) ;
      }

      $szFilecontent = str_replace( "%filename_s%", $filename_s, $szFilecontent);
      $libusb    = str_replace( "%filename_s%", $filename_s, $libusb);
      $libfatfs  = str_replace( "%filename_s%", $filename_s, $libfatfs);
      $libmemory = str_replace( "%filename_s%", $filename_s, $libmemory);
      $liblwip   = str_replace( "%filename_s%", $filename_s, $liblwip);
      $libuip    = str_replace( "%filename_s%", $filename_s, $libuip);
      $libjpeg   = str_replace( "%filename_s%", $filename_s, $libjpeg);
    }

    if (strstr($this->toolkit, "iar")) {
      $inc_path = str_replace("\\", "/", $inc_path);
    } elseif( strstr($this->toolkit, "mdk") ) {
      $inc_path = str_replace("/", "\\", $inc_path);
    } elseif( strstr($this->toolkit, "gcc") ) {
      $inc_path = str_replace("\\", "/", $inc_path);
      $szFilecontent = str_replace( '%gcc_lib_dir%', $gcc_lib_dir, $szFilecontent );

      $gcc_vpath = str_replace("VPATH += ..\\..\\libraries\\libusb", "VPATH += \$(USB_LIB)", $gcc_vpath);
      $gcc_vpath = str_replace("%root_dir_c%libraries\\libusb", "\$(USB_LIB)", $gcc_vpath);
      $gcc_vpath = str_replace("\\", "/", $gcc_vpath);
      $gcc_vpath = str_replace("%end%", "", $gcc_vpath);
      $szFilecontent = str_replace( '%gcc_vpath%', $gcc_vpath, $szFilecontent );
    }

    if( strstr($this->toolkit, "gcc") ) 
      $inc_path = str_replace("%root_dir_i%libraries/libusb", "\$(USB_LIB)", $inc_path);
    $szFilecontent = str_replace( '%extra_include%', $inc_path, $szFilecontent );

    $arrayLib = array($libusb, $libfatfs, $libmemory, $liblwip, $libuip, $libjpeg);
    foreach ($arrayLib as $lib) {
      if (strstr($lib, $this->file_keyword)) {
        $lib = str_replace("%extra_file%", "", $lib);
        $szFilecontent = str_replace( '%extra_group%', $lib."%extra_group%", $szFilecontent );
      }
    }
    $szFilecontent = str_replace( "  C_OBJECTS += ", "C_OBJECTS += ", $szFilecontent );

    $szFilecontent = str_replace( '%libboard_file%', "", $szFilecontent );
    $szFilecontent = str_replace( '%libchip_file%', "", $szFilecontent );
    $szFilecontent = str_replace( '%extra_group%', "", $szFilecontent );
    $szFilecontent = str_replace( '%predefine%', "", $szFilecontent );
    $szFilecontent = str_replace( '%extra_file%', "", $szFilecontent );

    $szFilecontent = $this->proc_directory($szFilecontent, $szPath);
    return $szFilecontent;
  }

  /*****************************************************************************
    function: update configurations
  *****************************************************************************/
  function update_configurations($szFilecontent, $szConfiguration)
  {
    $pos = strpos($szFilecontent, $this->prj_target_s);
    $header = substr($szFilecontent, 0, $pos);
    $config1 = substr($szFilecontent, $pos);

    $pos = strrpos($config1, $this->prj_target_e);
    $tail = substr($config1, $pos+strlen($this->prj_target_e));
    $config1 = substr($config1, 0, $pos+strlen($this->prj_target_e));

    $szFilecontent  = $header;
    $szInsert = "";
    foreach ($this->targetList as $target) {
      if(strstr($szConfiguration, $target)) {
        $szFilecontent .= $szInsert;
        $szFilecontent .= str_replace("%configuration%", $target, $config1);
        $szInsert = $this->prj_target_ins;
      }
    }
    $szFilecontent .= $tail;

    return $szFilecontent;
  }

  function targets_post_proc($szFilecontent)
  {
    return $szFilecontent;
  }

  /*****************************************************************************
    function: add including paths for usb host examples
  *****************************************************************************/
  function add_inc_path_4_usb_host($inc_path, $szExamplePath)
  {
    if ( strstr($szExamplePath, "host") ) {
      if (false == strstr($inc_path, "../conf") ) {
        $inc_path .= str_replace("%inc_path%",
            "%prj_dir%../../conf",
            $this->template_inc);
        $host_inc_path = array(
            "%root_dir_i%libraries/libusb/host",
            "%root_dir_i%libraries/libusb/host/core",
          );
        if ( strstr($szExamplePath, "usb_host_cdc") ) {
          $host_inc_path = array_merge($host_inc_path, array(
              "%root_dir_i%libraries/libusb/host/hid",
              "%root_dir_i%libraries/libusb/host/cdc",
          ));
        } elseif ( strstr($szExamplePath, "usb_host_hid") ) {
          $host_inc_path = array_merge($host_inc_path, array(
              "%root_dir_i%libraries/libusb/host/hid",
              "%root_dir_i%libraries/libusb/host/hid/mouse",
          ));
        } elseif ( strstr($szExamplePath, "usb_host_msc") ) {
          $host_inc_path = array_merge($host_inc_path, array(
              "%root_dir_i%libraries/libusb/host/msc",
              "%root_dir_i%libraries/libusb/host/msc/storage/ctrl_access",
              "%root_dir_i%libraries/libusb/host/msc/storage/ctrl_access/module_config",
          ));
          if ( strstr($szExamplePath, "usb_host_msc_hid") ) {
            $host_inc_path = array_merge($host_inc_path, array(
                "%root_dir_i%libraries/libusb/host/hid",
                "%root_dir_i%libraries/libusb/host/hid/mouse",
            ));
          }
        } else {
          return $inc_path;
        }
        foreach ($host_inc_path as $tmpPath) {
          $inc_path .= str_replace("%inc_path%", $tmpPath, $this->template_inc);
        }
      }
    }
    return $inc_path;
  }

}

?>
