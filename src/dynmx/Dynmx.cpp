//
//  Dynmx.cpp
//  cinder_dynmx
//
//  Created by Thomas Buhrmann on 17/06/2011.
//  Copyright 2011 __MyCompanyName__. All rights reserved.
//

#include "Dynmx.h"
#include <time.h>
#include "boost/filesystem.hpp"
#include <mach-o/dyld.h>
#include <stdlib.h>
#include "rapidxml/rapidxml.hpp"
#include "rapidxml/rapidxml_print.hpp"

namespace dmx
{

Globals* Globals::pInstance = NULL;

//----------------------------------------------------------------------------------------------------------------------
Globals* Globals::Inst(const std::string& cfgFnm)
{
  if(pInstance == NULL)
  {
    pInstance = new Globals(cfgFnm);
    pInstance->saveToDataDir();
  }
  return pInstance;
}

//----------------------------------------------------------------------------------------------------------------------
Globals::Globals(const std::string& cfgFnm)
{ 
  initialise(cfgFnm);
}

//----------------------------------------------------------------------------------------------------------------------
void Globals::saveToDataDir()
{  
  m_settings->write(ci::writeFile(m_dataDir + m_fnm));
}

//----------------------------------------------------------------------------------------------------------------------
void Globals::initialise(const std::string& cfgFnm)
{
  // Get the base directory to save data to and read the config from  
  std::string baseDir;
#if DEPLOYING and defined DYNMX_MAC
  char path[1024];
  char path2[1024];
  uint32_t size = sizeof(path);
  _NSGetExecutablePath(path, &size);
  realpath(path, path2);
  std::string p(path2);
  boost::filesystem::path bp (p);
  baseDir = bp.parent_path().parent_path().parent_path().parent_path().string() + "/";
#else
  baseDir = std::string(DATA_BASE_DIR);
#endif  
  
  // Read in config xml file, so content will be available in memory, rather than on disk.
  m_settings = 0;
  if(cfgFnm.empty())
  {
    // No config file name given, so load first in base directory with xml ending.
    boost::filesystem::directory_iterator end_itr;  
    for (boost::filesystem::directory_iterator itr(baseDir); itr != end_itr; ++itr)
    {
      // If it's not a directory ...
      if (!is_directory(itr->status()))
      {
        // ... and the filename contains the xml ending      
        m_fnm = itr->path().filename().string();
        if(m_fnm.find(".xml") != std::string::npos)
        {
          // ... then load the file
          try
          {
            m_settings = new ci::XmlTree(ci::loadFile(baseDir + m_fnm));
            std::cout << "Loaded global settings implicitly from '" << m_fnm << "'." << std::endl;
          }
          catch (rapidxml::parse_error& e)
          {
            std::cout << ">> XML parse error: '" << e.what() << "'. Aborting!" << std::endl;
          }
          break;
        }               
      }
    }
  }
  else
  {
    // Specific config file name given, so load that
    try
    {    
      m_settings = new ci::XmlTree(ci::loadFile(baseDir + cfgFnm));
      m_fnm = cfgFnm;
      std::cout << "Loaded global settings explicitly from '" << cfgFnm << "'." << std::endl;
    }
    catch (rapidxml::parse_error& e)
    {
      std::cout << "XML parse error: '" << e.what() << "'. Aborting!" << std::endl;
    }    
  }
  
  // Choose subfolder to save output file
  std::string subDir = "Output/";
  if(m_settings->hasChild("Config/Globals/OutputFolder"))
  {
    std::string dir = (m_settings->getChild("Config/Globals/OutputFolder"))["Name"].as<std::string>();
    if(!dir.empty())
    {
      subDir = dir;
      subDir += "/";
    }
  }
  
  // Create a new directory based on current data and time and store the path for global access
  time_t now = time(NULL);
  static const int TimeMaxChar = 128; 
  char dateTime[TimeMaxChar];
  dateTime[0] = '\0';
  strftime(dateTime, TimeMaxChar, "%y_%m_%d__%H_%M_%S", localtime(&now));   
  
  // Actually create the data directory
  m_dataDir = baseDir + subDir + dateTime + "/";
  boost::filesystem::create_directories(boost::filesystem::path(m_dataDir));
  
}

} // namespace dmx