/*
 *  Recorder.h
 *  cinder_dynmx
 *
 *  Created by Thomas Buhrmann on 7/15/11.
 *  Copyright 2011 __MyCompanyName__. All rights reserved.
 *
 */

#ifndef _DMX_RECORDER_
#define _DMX_RECORDER_

#include "Dynmx.h"

#include <string>
#include <map>


namespace dmx
{

//----------------------------------------------------------------------------------------------------------------------
// Base class for caching model data for analysis and later output 
// Todo: templatize on data type
//----------------------------------------------------------------------------------------------------------------------  
class Recorder
{
public:
  
  Recorder() : m_printVarNames(true) { m_data = new std::map<std::string, std::vector<double> >; m_data->clear(); };
  
  // Adds a new data point to the end of the corresponding variable's cache (vector)
  void push_back(std::string name, double value) { (*m_data)[name].push_back(value); };
  
  // Remove all data
  void clear() { m_data->clear(); };
  
  // Return the data vector for the named variable
  std::vector<double>& operator[] (std::string name) { return (*m_data)[name]; };
  
  // Save data to file
  void saveTo(const std::string& fnm);
  
protected:
  
  std::map<std::string, std::vector<double> >* m_data;
  bool m_printVarNames;
};
  
}; // namespace dmx

#endif