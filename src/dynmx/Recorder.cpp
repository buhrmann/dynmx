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

#define RECORDER_PRECISION 6

namespace dmx
{

// Prints data in column format (each entry is a new row)
//----------------------------------------------------------------------------------------------------------------------    
void Recorder::saveTo(const std::string& fnm)
{
  if(m_data->empty())
  {
    return;
  }
     
  std::ofstream file;
  file.open(fnm.c_str(), std::ios_base::app);
  if(file.fail())
  {
    return;
  }
  
  // Print column headers
  std::map<std::string, std::vector<double> >::const_iterator iter;  
  if(m_printVarNames)
  {
    for (iter = m_data->begin(); iter != m_data->end(); ++iter)
    {
      std::string name = iter->first;
      file << name << " ";
    }
    file << "\r\n"; // Don't flush yet
  }
    
  // Print data, but not all vectors might be of same length
  bool hadValue = true;
  int i = 0;
  file.setf(std::ios_base::fixed);
  file.precision(RECORDER_PRECISION);
  while(hadValue)
  {
    hadValue = false;
    iter = m_data->begin();
    for (; iter != m_data->end(); ++iter)
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