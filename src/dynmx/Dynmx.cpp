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
  
namespace bfs = boost::filesystem; 
  
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
  // Defaults
  m_settings = 0;
  m_fnm.clear();
  
  // Current directory of the executable
  bfs::path currentPath ( bfs::current_path() );    
  std::string currentPathStr = currentPath.string();
  
  // Look for config file in current exe's directory or in subfolder specified in argument
  std::string pathToConfig;
  if(cfgFnm.empty())
  {
#if DEBUGGING
    std::cout << "No config file specified. Searching in local directory '" << currentPathStr << "'." << std::endl;    
#endif    
    // No config file name given, so load first in base directory with xml ending.
    bfs::directory_iterator end_itr;  
    for (bfs::directory_iterator itr(currentPath); itr != end_itr; ++itr)
    {
      // If it's not a directory ...
      if (!is_directory(itr->status()))
      {
        // ... and the filename contains the xml ending      
        const std::string& fnm = itr->path().filename().string();
        if(fnm.find(".xml") != std::string::npos)
        {
          m_fnm = fnm;
          pathToConfig = itr->path().string();
#if DEBUGGING
          std::cout << "Found config file '" << m_fnm << "'." << std::endl;    
#endif              
          break;
        }               
      }
    }
  }
  else
  {
    // Global paths start with a forward slash '/', local paths don't.
    if (cfgFnm[0] == '/' && bfs::exists(cfgFnm))
    {
      m_fnm = cfgFnm.substr(cfgFnm.rfind('/') + 1);
      pathToConfig = cfgFnm;
#if DEBUGGING
      std::cout << "Found specified config file in ABSOLUTE path '" << cfgFnm << "'." << std::endl;    
#endif                
    }
    else 
    {
      const std::string localPath = currentPathStr + "/" + cfgFnm;      
      if(bfs::exists(localPath))
      {
        m_fnm = cfgFnm.substr(cfgFnm.rfind('/') + 1);;
        pathToConfig = localPath;
  #if DEBUGGING
        std::cout << "Found specified config file in LOCAL path '" << localPath << "'." << std::endl;    
  #endif
      }
    }
  }
  
  
  // If we have a valid config filename, load it
  if(!pathToConfig.empty())
  {
    try
    {    
      m_settings = new ci::XmlTree(ci::loadFile(pathToConfig));
    }
    catch (rapidxml::parse_error& e)
    {
      std::cout << "XML parse error trying to read config: '" << e.what() << "'. Aborting!" << std::endl;
    }    
  }
  else 
  {
#if DEBUGGING
    std::cout << "No config file specified. Not loading any settings! '" << std::endl;    
#endif    
  }
  
  // Choose folder to save output file. It's either a hard-coded subfolder of current directory,
  // or one specified in config file
  std::string outputDir(currentPathStr + "/Output/");
  if(m_settings && m_settings->hasChild("Config/Globals/OutputFolder"))
  {
    std::string dir = m_settings->getChild("Config/Globals/OutputFolder").getAttributeValue<std::string>("Name");
    if(!dir.empty())
    {
      // Expand home directory: replaces '~' if appropriate
      dir = pathExpandHome(dir);
      
      // Absolute or relative path?
      if(dir[0] != '/')
      {
        outputDir = currentPathStr + "/" + dir + "/";
      }
      else
      {
        outputDir = dir + "/";
      }
    }
  }


  // Create a new directory based on current data and time and store the path for global access
  time_t now = time(NULL);
  static const int TimeMaxChar = 128; 
  char dateTime[TimeMaxChar];
  dateTime[0] = '\0';
  strftime(dateTime, TimeMaxChar, "%y_%m_%d__%H_%M_%S", localtime(&now));   
  
  // Finally create the data directory
  m_dataDir = outputDir + dateTime + "/";
  bfs::create_directories(boost::filesystem::path(m_dataDir));
  
}

} // namespace dmx