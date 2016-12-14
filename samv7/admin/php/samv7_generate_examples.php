<?php

include 'ClassProject.php';
include 'ClassIarEwp.php';
include 'ClassMdkUvprojx.php';
include 'ClassGccMakefile.php';
include 'ClassStudioCproj.php';

  $arrayBoard = array(
    array("v71", "SAMV71 Xplained ULTRA board", "BOARD_SAMV71_XULT", "SAMV71Q21"),
    array("e70", "SAME70 Xplained board",       "BOARD_SAME70_XPLD", "SAME70Q21"),
    );

  function help()
  {
    global $arrayBoard;
    $szHelp = "Parameter: v71/e70\n";
    foreach ( $arrayBoard as $b  => $value ) 
      $szHelp.= "   ".$arrayBoard[$b][0]."  -  update softpack for ".$arrayBoard[$b][1]."\n";
    die($szHelp);
  }

  function update_for_document($board)
  {
    $arrayTmp = array(
      array(
        "SAMV71 Xplained Ultra Software",
        "document_SAMV71_XULT",
        "BOARD_SAMV71_XULT",
        "__SAMV71Q21__",
        ),
      array(
        "SAME70 Xplained Software",
        "document_SAME70_XPLD",
        "BOARD_SAME70_XPLD",
        "__SAME70Q21__",
        ));

    // update doxyfile
    $file = "admin/doxygen/Doxyfile-samv7";
    $szFilecontent=file_get_contents($file);
    foreach ($arrayTmp[0] as $key  => $value) {
      if ($board == "v71") {
        $szFilecontent=str_replace( $arrayTmp[1][$key], $arrayTmp[0][$key], $szFilecontent ) ;
      } else if ($board == "e70") {
        $szFilecontent=str_replace( $arrayTmp[0][$key], $arrayTmp[1][$key], $szFilecontent ) ;
      }
    }
    file_put_contents( $file, $szFilecontent ) ;

    // update header-samv7.html
    $file = "admin/doxygen/header-samv7.html";
    $szFilecontent=file_get_contents($file);
    foreach ($arrayTmp[0] as $key  => $value) {
      if ($board == "v71") {
        $szFilecontent=str_replace( $arrayTmp[1][$key], $arrayTmp[0][$key], $szFilecontent ) ;
      } else if ($board == "e70") {
        $szFilecontent=str_replace( $arrayTmp[0][$key], $arrayTmp[1][$key], $szFilecontent ) ;
      }
    }
    file_put_contents( $file, $szFilecontent ) ;
  }

function string_replace($szContent, $szKeyWord, $search, $replace)
{
  $pos = strpos($szContent, $szKeyWord);
  $szContent1 = substr($szContent, 0, $pos);
  $szContent2 = substr($szContent, $pos);
  $pos = strpos($szContent2, $search) + strlen($search);
  $szContent3 = substr($szContent2, $pos);
  $szContent2 = str_replace($search, $replace, substr($szContent2, 0, $pos));
  return $szContent1.$szContent2.$szContent3;
}

//===========================================================================
  $szEwarmExamplesList = '';
  $szGnuExamplesList ='';
  
  $gxmlFile = "ExampleDirInfo.ENU.xml";
// Example list in 'examples' folder.
  $gaXmlList = array
  (
      //   = example name =                  = template path =
      array("./",                             "admin/templates/"        , ""),
      array("examples/",                      "admin/templates/example/", "Basic"),
      array("examples_crypto/",               "admin/templates/example/", "crypto"),
      array("examples_ethernet/",             "admin/templates/example/", "ethernet"),
      array("examples_storage/",              "admin/templates/example/", "storage"),
      array("examples_usb/device_examples/",  "admin/templates/example/", "USB device"),
      array("examples_usb/host_examples/",    "admin/templates/example/", "USB host"),
  );

  
  define( 'PARSE_STATE_NONE', 0 ) ;
  define( 'PARSE_STATE_PAGE', 1 ) ;
  define( 'PARSE_STATE_PURPOSE', 2 ) ;
  define( 'PARSE_STATE_DESCRIPTION', 3 ) ;

  /*
   * %example_name%
   * %example_page%
   * %example_label%
   * %example_purpose%
   * %example_description%
   *
   */


  /**
   *
   */
  class CExampleData
  {
    var $szPath ;
    var $szName ;
    var $szPage ;
    var $szLabel ;
    var $szPurpose ;
    var $szDescription ;

    function CExampleData()
    {
    }
  }

  class CExampleConfig
  {
    var $files;
    var $predefine;
  }

  /**
   * Obtain all examples (sub-folders) from a given path
   */
  function get_example_list( $szPath )
  {
    $aList=array() ;

    if ( is_dir( $szPath ) )
    {
      if ( $dh = opendir( $szPath ) )
      {
        while ( ($szFileName = readdir( $dh )) !== false )
        {
          // we check if file is a folder and not a specific folder like '.' '..' '.svn'
          if ( is_dir( $szPath.'/'.$szFileName ) && ($szFileName[0] != '.') )
          {
//            echo '*'.$szPath."/$szFileName\n" ;
            $aList[]=$szPath."/$szFileName" ;
          }
        }
        closedir( $dh ) ;
      }
    }

    // we return the whole list of subfolders for the given path
    return $aList ;
  }

  /**
   * Obtain the Doxygen header written by developper at beginning of each main.c
   */
  function get_doxygen_header( $szString )
  {
    $szHeader='' ;

    // we look for beginning of doxygen documentation
    $i=strpos( $szString, '/**' ) ;
    if ( $i !== false )
    {
      // we look for end of doxygen documentation
      $i2=strpos( $szString, '*/', $i ) ;

      if ( $i2 !== false )
      {
        // we extract the doxygen documentation
        $szHeader=substr( $szString, $i, $i2-$i ) ;
      }
    }

//    var_dump( $szHeader ) ;
    // we return the extracted header
    return $szHeader ;
  }

  /**
   * Parse main.c to obtain Doxygen header and then only the needed information to
   * create the build folder hierarchy
   */
  function parse_main( $szPath )
  {
    $iState=PARSE_STATE_NONE ;

    echo "-----------------------------------------------------------------------\r\n" ;
    echo "Parsing $szPath [".basename( $szPath )."]\n" ;
    $cExampleData=new CExampleData() ;
    $cExampleData->szPath=$szPath ;
    $cExampleData->szName=basename( $szPath ) ;
    // Obtain file content
    $szContent=file_get_contents( $szPath.'/main.c' ) ;
    // Look for doxygen intro
    $szHeader=get_doxygen_header( $szContent ) ;
    // var_dump( $szHeader ) ;
    $szContent='' ;

    $szHeader=str_replace( '/**', '', $szHeader ) ;
    $szHeader=str_replace( ' *  ', '', $szHeader ) ;
    $szHeader=str_replace( ' *', '', $szHeader ) ;
    $szHeader=str_replace( "\r\n", "\n", $szHeader ) ;
    // var_dump( $szHeader ) ;

    $aszLines=explode( "\n", $szHeader ) ;
    // var_dump( $aszLines ) ;
    foreach( $aszLines as $szLine )
    {
      $szLine=trim( $szLine ) ;
//      echo "--- $szLine\r\n" ;

      switch ( $iState )
      {
        case PARSE_STATE_NONE :
          // check \page
          if ( $a=strstr( $szLine, '\page' ) )
          {
            $iState=PARSE_STATE_PAGE ;
            $aInfos=explode( ' ', $szLine, 3 ) ;
            $cExampleData->szName=$aInfos[1] ;
            #$cExampleData->szPage=str_replace( '-', '__' , str_replace( '_', '__' ,$aInfos[1] ) ) ;
            $cExampleData->szPage= $aInfos[1];
            $cExampleData->szLabel=$aInfos[2] ;
            $iState=PARSE_STATE_NONE ;
          }

          // check \section Purpose
          if ( $a=strstr( $szLine, '\section Purpose' ) )
          {
            $iState=PARSE_STATE_PURPOSE ;
            $cExampleData->szPurpose='' ;
//            echo "Purpose\r\n" ;
          }

          // check \section Description
          if ( $a=strstr( $szLine, '\section Description' ) )
          {
            $iState=PARSE_STATE_DESCRIPTION ;
            $cExampleData->szDescription='' ;
//            echo "Description\r\n" ;
          }
        break ;

        // this case just for info, not used
        case PARSE_STATE_PAGE :
        break ;

        case PARSE_STATE_PURPOSE :
          // test end of block
//            echo "Purpose ($szLine)\r\n" ;
          if ( $a=strstr( $szLine, '\section' ) )
          {
            $iState=PARSE_STATE_NONE ;
          }
          else
          {
            // add line to purpose string
            $cExampleData->szPurpose.=" $szLine" ;
          }
        break ;

        case PARSE_STATE_DESCRIPTION :
          // test end of block
//          echo "Description ($szLine)\r\n" ;
          if ( $a=strstr( $szLine, '\section' ) )
          {
            $iState=PARSE_STATE_NONE ;
          }
          else
          {
            if ( strlen( $szLine ) == 0 )
            {
              $cExampleData->szDescription.="\n" ;
            }
            else
            {
              // add line to description string
              $cExampleData->szDescription.=" $szLine" ;
            }
          }
        break ;
      }
    }
    return $cExampleData ;
  }

  /**
   *
   */
  function getDirectoryTree( $szPath, $aszFilters = array() )
  {
    $aItems = array_diff( scandir( $szPath ), array_merge( Array( ".", ".." ), $aszFilters ) );
    $dir_array = Array() ;

    if ( count( $aItems ) == 0 )
    {
      return $dir_array ;
    }

    foreach( $aItems as $Item )
    {
      if ( is_dir( $szPath."/".$Item ) )
      {
        $aNewItems=getDirectoryTree( $szPath."/".$Item, $aszFilters ) ;

        if ( count( $aNewItems ) > 0 )
        {
          // We have subitems, so we add the folder into array
          $dir_array[]=$szPath.'/'.$Item.'/' ;

          // Adding subitems
          foreach ( $aNewItems as $NewItem )
          {
            $dir_array[] = $NewItem ;
          }
        }
      }
      else
      {
        $dir_array[] = $szPath."/".$Item ;
      }
    }

    return $dir_array ;
  }
  
  /**
   *
   */
  function create_template( $cExampleData, $szTemplatePath )
  {
    global $boardInfo;
    echo "- creating template for $cExampleData->szPath using $szTemplatePath\n" ;
    if (is_dir( $szTemplatePath."/build" ) )
    {
      $szTemplatePath.="/build" ;
      $aFiles=getDirectoryTree( $szTemplatePath, array( ".svn" ) ) ;

      if ( $aFiles > 0 )
      {
        $aReplacedFiles=Array() ;

        foreach( $aFiles as $File )
        {
          // Replace template path with example path
          $szTemp=str_replace( $szTemplatePath, $cExampleData->szPath.'/build', $File ) ;

          // Replace with real example name
          $aPath_parts = pathinfo( $szTemp ) ;
          // if file name to be replaced
          if ( $aPath_parts['filename'] == 'example' )
          {
            $szTemp=$aPath_parts['dirname'].'/'.$cExampleData->szName.'.'.$aPath_parts['extension'] ;
          }
          $aReplacedFiles[]=$szTemp ;
          
          // Copy template file hierarchy to example folder
          if ( is_dir( $File ) )
          {
            // create folder into example hierarchy
            echo "Creating folder $szTemp\n" ;
            if ( false == is_dir($szTemp) )   mkdir( $szTemp, 0777, true ) ;
          }
          else
          {
            // Load file
            $szFilecontent=file_get_contents( $File ) ;
            if ($cExampleData->szName == 'fft_demo') {
              if ( strstr( $szTemp, '.ewp' ) ) {
                // change FPU setting
                $szFilecontent = string_replace($szFilecontent, "<name>Variant</name>",
                  "20</version>", "21</version>");
                $szFilecontent = string_replace($szFilecontent, "<name>Variant</name>",
                  "40</state>", "41</state>");
                $szFilecontent = string_replace($szFilecontent, "<name>FPU</name>",
                  "2</version>", "4</version>");
                $szFilecontent = string_replace($szFilecontent, "<name>FPU</name>",
                  "0</state>", "10</state>");
                $szFilecontent = string_replace($szFilecontent, "<name>GFPUCoreSlave</name>",
                  "20</version>", "21</version>");
                $szFilecontent = string_replace($szFilecontent, "<name>GFPUCoreSlave</name>",
                  "40</state>", "41</state>");
                $szFilecontent = string_replace($szFilecontent, "<name>GBECoreSlave</name>",
                  "20</version>", "21</version>");
                $szFilecontent = string_replace($szFilecontent, "<name>GBECoreSlave</name>",
                  "40</state>", "41</state>");
                $szFilecontent = string_replace($szFilecontent, 
                  '<name>IlinkAdditionalLibs</name>',
                  '</state>',
                  "\$PROJ_DIR\$\..\..\..\..\libraries\libchip\include\cmsis\CMSIS\Lib\ARM\arm_cortexM7lfdp_math.lib</state>" );
              }
            }

            echo "\nprocess ----> $szTemp\n";
            if ( strstr( $szTemp, '.ewp' ) || strstr( $szTemp, '.ewd' ) ) {
              $IarEwp = new ClassIarEwp();
              $szFilecontent = $IarEwp->process($szFilecontent, $cExampleData->szPath);
            } elseif ( strstr( $szTemp, '.uvprojx') || strstr( $szTemp, '.uvoptx') ) {
              $uvprojx = new ClassMdkUvprojx();
              $szFilecontent = $uvprojx->process($szFilecontent, $cExampleData->szPath);
            } elseif ( strstr( $szTemp, 'Makefile' ) ) {
              $gcc = new ClassGccMakefile();
              $szFilecontent = $gcc->process($szFilecontent, $cExampleData->szPath);
            } elseif ( strstr( $szTemp, 'cproj' ) ) {
              $cproj = new ClassStudioCproj();
              $szFilecontent = $cproj->process($szFilecontent, $cExampleData->szPath);
            } else {
              // continue;
            }

            // Search and replace %tag% into file to corresponding value
            $szFilecontent=str_replace( '%example_name%', $cExampleData->szName, $szFilecontent ) ;
            $szFilecontent=str_replace( '%example_label%', $cExampleData->szLabel, $szFilecontent ) ;
            $szFilecontent=str_replace( '%example_page%', $cExampleData->szPage, $szFilecontent ) ;
            $szFilecontent=str_replace( '%example_purpose%', $cExampleData->szPurpose, $szFilecontent ) ;
            $szFilecontent=str_replace( '%example_description%', $cExampleData->szDescription, $szFilecontent ) ;
            if ($cExampleData->szName == 'gmac_uip_helloworld') {
                $szUipFolder = 'hello-world';
                $szFilecontent=str_replace( '%appInc%', $szUipFolder, $szFilecontent ) ;
            }
            if ($cExampleData->szName == 'gmac_uip_telnetd') {
                $szUipFolder = 'telnetd';
                $szFilecontent=str_replace( '%appInc%', $szUipFolder, $szFilecontent ) ;
            }
            if ($cExampleData->szName == 'gmac_uip_webserver') {
                $szUipFolder = 'webserver';
                $szFilecontent=str_replace( '%appInc%', $szUipFolder, $szFilecontent ) ;
            }

            // Different optimization -O0/Os
            if ($cExampleData->szName == 'twi_eeprom') {
                $szSearch = "</armgcc.compiler.directories.IncludePaths>";
                $szReplace=$szSearch."\r\n";
                $szReplace.= "  <armgcc.compiler.optimization.level>Optimize for size (-Os)</armgcc.compiler.optimization.level>\n" ;
                $szFilecontent=str_replace( $szSearch, $szReplace, $szFilecontent) ;
                $szFilecontent=str_replace( "OPTIMIZATION = -O0", "OPTIMIZATION = -Os", $szFilecontent) ;
            }

            echo "Copying file $szTemp\n". $File;
            if ( $a=strstr( $szTemp, 'Makefile' ) )
            {
              $szFilecontent = str_replace("\n", "\r\n", $szFilecontent);
            }

            $replaceMent = array(
                array("%resources%\\nocache_region\\iar\\%samv7%-flash.mac", "%resources%\\iar\\%samv7%-flash.mac"),
                array("%resources%\\nocache_region\\iar\\%samv7%-sram.mac",  "%resources%\\iar\\%samv7%-sram.mac"),
              );
            foreach ($replaceMent as $key) {
              $szFilecontent = str_replace($key[0], $key[1], $szFilecontent);
            }
            if (strstr($boardInfo[0], "v7")) {
              $szFilecontent = str_replace("%SAMV7x%", "SAMV7x", $szFilecontent);
              $szFilecontent = str_replace("%SAMV71%", "SAMV71", $szFilecontent);
            }
            elseif (strstr($boardInfo[0], "e7")) {
              $szFilecontent = str_replace("%SAMV7x%", "SAME7x", $szFilecontent);
              $szFilecontent = str_replace("%SAMV71%", "SAME70", $szFilecontent);
              $szFilecontent = str_replace("SAMV71_Xplained_Ultra", "SAME70_Xplained", $szFilecontent);
              $szFilecontent = str_replace("SAMV71 Software Package", "SAME70 Software Package", $szFilecontent);
            }
            $szFilecontent = str_replace("%resources%", "resources_".$boardInfo[0], $szFilecontent);
            $szFilecontent = str_replace("%samv7%", "sam".$boardInfo[0], $szFilecontent);
            $szFilecontent = str_replace("system_sam", "system_sam".$boardInfo[0], $szFilecontent);
            if ( !strstr( $szTemp, '.uvprojx' ) ) 
              $szFilecontent = str_replace("startup_sam", "startup_sam".$boardInfo[0], $szFilecontent);
            $szFilecontent = str_replace("%chip_lowercase%", strtolower($boardInfo[3]),  $szFilecontent);

            if (strstr($szTemp, ".atsln")) {
              file_put_contents( str_replace(".atsln", "_6_2.atsln", $szTemp), 
                                 str_replace(".cproj", "_6_2.cproj", $szFilecontent) );
              $szTemp = str_replace(".atsln", "_7_0.atsln", $szTemp);
              $szFilecontent = str_replace(".cproj", "_7_0.cproj", $szFilecontent);
            }
            else if (strstr($szTemp, ".cproj")) {
              file_put_contents( str_replace(".cproj", "_6_2.cproj", $szTemp), $szFilecontent ) ;
              $szTemp = str_replace(".cproj", "_7_0.cproj", $szTemp);
              $szFilecontent = str_replace("<ProjectVersion>6.2", "<ProjectVersion>7.0", $szFilecontent);
            }
            else if (strstr($szTemp, "StartupScreen.ewsample")) {
              if(strstr($cExampleData->szPath, "examples_usb")) {
                $szFilecontent = str_replace("libraries</copyDir>", "..\libraries</copyDir>", $szFilecontent);
                $szFilecontent = str_replace(    "utils</copyDir>",     "..\utils</copyDir>", $szFilecontent);
              }
            }
            file_put_contents( $szTemp, $szFilecontent ) ;
          }
        }
      }
    }
  }

  function get_configurations($szPath, $toolkit)
  {
    $exampleConfig = parse_dependency($szPath."/dependency.ini");
    
    $dir_adjust = "";
    if( strstr($szPath, "examples_usb") && strstr($toolkit, "studio") ){
      $dir_adjust = "..\\";
    }

      $dirHandle=@opendir($szPath);
      while(($file1=readdir($dirHandle))!==false){ 
         $arr=array('..' , ¡¯ .¡¯ , 'build','main.c','Send.txt', "HIDTest", "hidTest.exe");
         $objline=str_replace( '.c', '.o', $file1) ;
         $objline=str_replace( '.C', '.o', $objline) ;
         $objline=str_replace( '.asm', '.o', $objline) ;
         if(in_array($file1,$arr)){continue;}
         if( $file1 == 'dependency.ini'){continue;}
         if( $file1 == '.'){continue;}
         if( $file1 == 'conf'){
             $dirTemp=@opendir($szPath.'/conf');
             while(($file2=readdir($dirTemp))!==false){ 
             $arr1=array('..' , ¡¯ .¡¯);
            if(in_array($file2,$arr1)){continue;}
            if( $file2 == '.'){continue;}
              $exampleConfig->files[] = "..\\..\\..\\".$dir_adjust.$file2 ;
            }
           closedir($dirTemp); 
          }
         else{
            $exampleConfig->files[] = "..\\..\\".$dir_adjust.$file1;
          }
      }  

    if (strstr($szPath, "examples_storage")) {
    // files all needed to add into project files of examples under examples_storage\
    $arraySotrageFiles = array(
          "..\\..\\libraries\\libchip\\source\\mcid_dma.c",
          "..\\..\\libraries\\libchip\\source\\hsmci.c",
          "..\\..\\libraries\\libstoragemedia\\source\\sdmmc\\sdmmc.c",
          "..\\..\\libraries\\libstoragemedia\\source\\Media.c",
          "..\\..\\libraries\\libstoragemedia\\source\\MEDSdcard.c",
      );
      foreach ($arraySotrageFiles as $file) {
        $exampleConfig->files[] = $file ;
      }
    }
    return $exampleConfig;
  }

  function parse_dependency($szDependencyFile)
  {
    $exampleConfig = new CExampleConfig();
    $exampleConfig->files = Array() ;
    $exampleConfig->predefine = Array() ;

    $handle = fopen($szDependencyFile, "r");
    if (!$handle) {
      echo "-E- error while opening the dependency file.\n";
      return $exampleConfig;
    }

    while (($line = fgets($handle)) !== false) {
      // process the line read.
      if ( strstr($line, "PREDEFINE" ) ) {
        $line = str_replace("\r", "", $line);
        $line = str_replace("\n", "", $line);
        $exampleConfig->predefine[] = str_replace("PREDEFINE ", "", $line);
      }
      else if ( !strstr( $line, '#' ) ) {
        $line = str_replace("\r", "", $line);
        $line = str_replace("\n", "", $line);
        $line = str_replace(" ", "", $line);
        $line = str_replace("\t", "", $line);
        if ($line)  $exampleConfig->files[] = $line ;
      }
    }
    fclose($handle);

    return $exampleConfig;
  }
  
  function build_folder( $szPath, $szTemplatePath )
  {
    $aExamplesPath=get_example_list( $szPath ) ;
    print_r( $aExamplesPath ) ;

    foreach( $aExamplesPath as $szExamplePath )
    {
      $cExampleData=parse_main( $szExamplePath ) ;
      
      create_template( $cExampleData, $szTemplatePath ) ;
    }
  }
  
   /*
   * Build specific examples in the folder with specified build template.
   * The example list are defined by users.
   */
  function build_folder_specific( $szPath, $aExamplesList )
  {
    global $szEwarmExamplesList, $szGnuExamplesList;
    global $szExtraFiles;
    global $szExtraGccFiles;
    global $szExtraMdkFiles;
    global $szExtraStudioFiles;
    print_r($aExamplesList);
    
    foreach ($aExamplesList as $i => $value)
    {
      // $szExamplePath = $szPath.'/'.$aExamplesList[$i][0];
      $szExamplePath = $szPath.'/'.$aExamplesList[$i];
      $szTemplatePath = "admin/templates/example";
      $cExampleData=parse_main( $szExamplePath ) ;
      $szExtraFiles = '';
      $szExtraGccFiles = '';
      $szExtraMdkFiles = '';
      $szExtraStudioFiles = '';
      $dirHandle=@opendir($szExamplePath);
      while(($file1=readdir($dirHandle))!==false){ 
         $arr=array('..' , ¡¯ .¡¯ , 'build','main.c','Send.txt');
         $objline=str_replace( '.c', '.o', $file1) ;
         $objline=str_replace( '.C', '.o', $objline) ;
         $objline=str_replace( '.asm', '.o', $objline) ;
         if(in_array($file1,$arr)){continue;}
         if( $file1 == 'dependency.ini'){continue;}
         if( $file1 == '.'){continue;}
         if( $file1 == 'conf'){
             $dirTemp=@opendir($szExamplePath.'/conf');
             while(($file2=readdir($dirTemp))!==false){ 
             $arr1=array('..' , ¡¯ .¡¯);
            if(in_array($file2,$arr1)){continue;}
            if( $file2 == '.'){continue;}
             $szExtraStudioFiles .= "<Compile Include=\""."..\\..\\conf\\" .$file2. "\">\r<SubType>compile</SubType>\r<Link>$file2</Link>\r</Compile>\n\r";             
            }
           closedir($dirTemp); 
          }
         else{ 
             $szExtraFiles .= "<file><name>$" ."PROJ_DIR$". "\..\..\\" . $file1 . "</name></file>\n\r";
             $szExtraStudioFiles .= "<Compile Include=\""."..\\..\\" .$file1. "\">\r<SubType>compile</SubType>\r<Link>$file1</Link>\r</Compile>\n\r";
             echo $szExtraStudioFiles;
             if ( !strstr( $file1, '.h' ) )
             { 
               $szExtraGccFiles .= "C_OBJECTS += " . $objline . "\n"; 
               $szExtraMdkFiles .= "<File>\n\r <FileName>".$file1."</FileName>\n\r"."<FileType>1</FileType>\n\r"."<FilePath>..\..\\" .$file1 . "</FilePath>\n\r"." </File>";
             }
          }    
      }  
      closedir($dirHandle); 
      create_template( $cExampleData, $szTemplatePath ) ;

      {    
          $szEwarmItem  = "echo ============================================================= \n";
          $szEwarmItem .= "echo building %ExmapleFileName% example for sram configuration\n";
          $szEwarmItem .= "echo ============================================================= \n" ; 
          $szEwarmItem .="cd %Example_Path%/%ExmapleFileName%/build/ewarm\n";
          $szEwarmItem .=" iarbuild %ExmapleFileName%.ewp -clean sram\n iarbuild %ExmapleFileName%.ewp -build sram\n";
          $szEwarmItem .= "echo ============================================================= \n";
          $szEwarmItem .= "echo building %ExmapleFileName% example for flash configuration\n";
          $szEwarmItem .= "echo ============================================================= \n" ; 
          $szEwarmItem .=" iarbuild %ExmapleFileName%.ewp -clean flash\n iarbuild %ExmapleFileName%.ewp -build flash\n";
          $szEwarmItem .=" cd ../../../../\n";
      }
      $szEwarmItem=str_replace( '%ExmapleFileName%', $aExamplesList[$i] ,$szEwarmItem ) ;
      $szEwarmItem=str_replace( '%Example_Path%', $szPath ,$szEwarmItem ) ;
      $szEwarmExamplesList=$szEwarmExamplesList.$szEwarmItem ;

      /* Paraper GNU workspace for all examples */
      $szGnuItem="	(cd ../../%Example_Path%/%ExmapleFileName%/build/gcc && \$(MAKE) -f Makefile clean all) \n";
      $szGnuItem=str_replace( '%ExmapleFileName%', $aExamplesList[$i] ,$szGnuItem ) ;
      $szGnuItem=str_replace( '%Example_Path%', $szPath ,$szGnuItem ) ;
      $szGnuExamplesList=$szGnuExamplesList.$szGnuItem ;
    }
  }

  function generate_ewarm_xml( $aXmlList )
  {
      global $boardInfo;
      global $gxmlFile;
      foreach ($aXmlList as $i => $value)
      {
          $szExamplePath = $aXmlList[$i][0].$gxmlFile;
          $szTemplatePath = $aXmlList[$i][1].$gxmlFile;
          // Load file
          $szFilecontent=file_get_contents( $szTemplatePath) ;
          // Search and replace %tag% into file to corresponding value
          if (strstr($boardInfo[0], "v7"))
            $szFilecontent=str_replace( '%chip_name%', 'samv7-xplained', $szFilecontent ) ;
          elseif (strstr($boardInfo[0], "e7"))
            $szFilecontent=str_replace( '%chip_name%', 'same7-xplained', $szFilecontent ) ;
          $szFilecontent=str_replace( '%class%', $aXmlList[$i][2],  $szFilecontent ) ;
          echo "Copying file $szExamplePath\n" ;
          file_put_contents( $szExamplePath, $szFilecontent ) ;
      }

      $List = array (
          "ExampleDirInfo.ENU.xml",
          "examples_usb/ExampleDirInfo.ENU.xml",
          "examples_usb/device_examples/ExampleDirInfo.ENU.xml",
          "examples_usb/host_examples/ExampleDirInfo.ENU.xml",
      );
      foreach ($List as $szFile) {
        $szFilecontent=file_get_contents($szFile) ;
        // Search and replace %tag% into file to corresponding value
        if (strstr($boardInfo[0], "v7"))
          $szFilecontent=str_replace( 'same7-xplained', 'samv7-xplained', $szFilecontent ) ;
        elseif (strstr($boardInfo[0], "e7"))
          $szFilecontent=str_replace( 'samv7-xplained', 'same7-xplained', $szFilecontent ) ;
        file_put_contents( $szFile, $szFilecontent ) ;
      }
    }

  function build_workspace()
  {
      global $szEwarmExamplesList, $szGnuExamplesList;

      $szFilecontent=file_get_contents( "admin/templates/build/ewarm/examples_ewarm_build.bat");
      $szFilecontent=str_replace( '%examples_in_workspace%', $szEwarmExamplesList, $szFilecontent ) ;
      file_put_contents( "build/ewarm/examples_ewarm_build.bat", $szFilecontent ) ;

      $szFilecontent=file_get_contents( "admin/templates/build/gcc/makefile");
      $szFilecontent=str_replace( '%chip_name%', 'samv7', $szFilecontent ) ;
      $szFilecontent=str_replace( '%examples_in_workspace%', $szGnuExamplesList, $szFilecontent ) ;
      file_put_contents( "build/gcc/makefile", $szFilecontent ) ;
  }
  
  
  /*--------------
   * Main program
   */
  if ($argc < 2) {
    help();
  }

  $boardInfo = "";
  foreach ($arrayBoard as $b) {
    if($argv[1] === $b[0]){
      print("-I- update softpack for ".$b[1]."\n");
      $boardInfo = $b;
      break;
    }
  }
  if(!$boardInfo){
    print("\n-I- invalid parameter!\n");
    help();
  }

  chdir( '../..' ) ;
  echo "Working directory is at: ".getcwd()."\n" ;

  // update samv7\ExampleDirInfo.ENU.xml
  $szFilecontent=file_get_contents("ExampleDirInfo.ENU.xml");
  if (strstr($boardInfo[0], "v7"))
    $szFilecontent=str_replace( "same7-xplained", "samv7-xplained", $szFilecontent ) ;
  elseif (strstr($boardInfo[0], "e7"))
    $szFilecontent=str_replace( "samv7-xplained", "same7-xplained", $szFilecontent ) ;
  file_put_contents( $file, $szFilecontent ) ;

  update_for_document($boardInfo[0]);

  echo "\n>>>>>> Build Ewarm Information center xml ...\n" ;
  generate_ewarm_xml($gaXmlList);

  $handle = fopen("admin/php/examples_v71.ini", "r");
  if (!$handle) {
    echo "-E- error while opening examples list file.\n";
    return;
  }
  $path = "";
  $arrayExamples = array();
  while (($line = fgets($handle)) !== false) {
    if ( strstr( $line, '#' ) )   continue;

    $line = str_replace("\r", "", $line);
    $line = str_replace("\n", "", $line);
    $line = str_replace(" ", "", $line);
    $line = str_replace("\t", "", $line);
    if ( strstr( $line, ':' ) ) {
      $line = str_replace(":", "", $line);
      if($path) {
        echo "\n>>>>>> Build folder for examples in folder \"$path\" ...\n";
        build_folder_specific($path, $arrayExamples);
        $arrayExamples = array();
      }
      $path = $line;
      continue;
    }
    if ($line)  $arrayExamples[] = $line;
  }
  fclose($handle);
  echo "\n>>>>>> Build folder for examples in folder \"$path\" ...\n" ;
  build_folder_specific($path, $arrayExamples);

  echo "\n>>>>>> Build workspace for all examples...\n" ;
  build_workspace();

?>
