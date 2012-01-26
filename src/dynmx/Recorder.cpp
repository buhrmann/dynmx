/*
 *  Recorder.cpp
 *  cinder_dynmx
 *
 *  Created by Thomas Buhrmann on 7/15/11.
 *  Copyright 2011 __MyCompanyName__. All rights reserved.
 *
 */

#include "Recorder.h"

#include <fstream>

namespace dmx
{

// Prints data in column format (each entry is a new row)
//----------------------------------------------------------------------------------------------------------------------    
void Recorder::saveTo(const std::string& fnm)
{
  std::ofstream file;
  file.open(fnm.c_str(), std::ios_base::app);
  
  std::map<std::string, std::vector<double> >::iterator iter = m_data.begin();
  
  // Print column headers
  for (; iter != m_data.end(); ++iter)
  {
    const std::string& name = iter->first;
    file << name << " ";
  }
  file << "\r\n"; // Don't flush yet
  
  // Print data, but not all vectors might be of same length
  bool hadValue = true;
  int i = 0;
  while(hadValue)
  {
    hadValue = false;
    iter = m_data.begin();
    for (; iter != m_data.end(); ++iter)
    {
      const std::vector<double>& values = iter->second;
      if(i < values.size())
      {
        file << values[i] << " ";
        hadValue = true;
      }
    }
    file << "\r\n";
    i++;
  }
  
  file.close();  
}
  
} // namespace dmx