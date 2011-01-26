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

// Interface for a simulation that can run without visualization
// ---------------------------------------------------------------------------------------------------------------------
namespace dmx
{

class Model
{
public:
  virtual void init() {};
  virtual void update(float dt){};
};

}

#endif