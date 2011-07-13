/*
 *  ofxModel.h
 *  openDynmx
 *
 *  Created by Thomas Buhrmann on 28/01/2010.
 *  Copyright 2010 __MyCompanyName__. All rights reserved.
 *
 */

#ifndef _DMX_MODEL_BASE_
#define _DMX_MODEL_BASE_

#include "Dynmx.h"

namespace dmx
{

//----------------------------------------------------------------------------------------------------------------------    
// Interface for a simulation that can run without visualization
// ---------------------------------------------------------------------------------------------------------------------
class Model
{

public:
  // Needs to be able to run as new after this function has been called.
  virtual void init() {};
  
  // Called every frame: do stuff here.
  virtual void update(float dt) {};
  
  // Signals that this model doesn't want/need to be updated anymore.
  virtual bool hasFinished() { return false; };
  
  // Getter and Setter for name, to be used by Factories e.g.
  const std::string& getName() const { return m_name; };
  void setName(const std::string& name) { m_name = name; };
  
protected:

  std::string m_name;
};

}

#endif