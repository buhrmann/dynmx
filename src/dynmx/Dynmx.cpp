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

namespace dmx
{

Globals* Globals::pInstance = NULL;

//----------------------------------------------------------------------------------------------------------------------
Globals* Globals::Inst()
{
  if(pInstance == NULL)
  {
    pInstance = new Globals();
  }
  return pInstance;
}

//----------------------------------------------------------------------------------------------------------------------
Globals::Globals()
{ 
  initialise();
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
  m_dataDir = std::string(DATA_BASE_DIR) + dateTime + "/";
  boost::filesystem::create_directory(m_dataDir);
  
  m_settings = new ci::XmlTree(ci::loadFile(std::string(DATA_BASE_DIR) + "Config.xml"));
}

} // namespace dmx