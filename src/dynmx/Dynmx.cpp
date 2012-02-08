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

namespace dmx
{

Globals* Globals::pInstance = NULL;

//----------------------------------------------------------------------------------------------------------------------
Globals* Globals::Inst()
{
  if(pInstance == NULL)
  {
    pInstance = new Globals();
    pInstance->saveToDataDir();
  }
  return pInstance;
}

//----------------------------------------------------------------------------------------------------------------------
Globals::Globals()
{ 
  initialise();
}

//----------------------------------------------------------------------------------------------------------------------
void Globals::saveToDataDir()
{  
  m_settings->write(ci::writeFile(m_dataDir + m_fnm));
}

//----------------------------------------------------------------------------------------------------------------------
void Globals::initialise()
{
  // Create a new directory based on current data and time and store the path for global access
  time_t now = time(NULL);
  static const int TimeMaxChar = 128; 
  char dateTime[TimeMaxChar];
  dateTime[0] = '\0';
  strftime(dateTime, TimeMaxChar, "%y_%m_%d__%H_%M_%S", localtime(&now));   
  std::string baseDir;
  
  // Get the base directory to save data to and read the config from
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
  
  // Actually create the data directory
  m_dataDir = baseDir + "Output/" + dateTime + "/";
  boost::filesystem::create_directory(boost::filesystem::path(m_dataDir));
  
  // Now load the xml file
  m_settings = 0;
  
  // Iterator over base directory's contents
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
        m_settings = new ci::XmlTree(ci::loadFile(baseDir + m_fnm));
        break;
      }               
    }
  }
}

} // namespace dmx