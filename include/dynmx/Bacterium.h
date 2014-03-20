//
//  Bacterium.h
//  dynmx
//
//  Created by Thomas Buhrmann on 24/02/14.
//
//

#ifndef dynmx_Bacterium_h
#define dynmx_Bacterium_h

#include "SMCAgent.h"
#include "GradientSensor.h"

namespace dmx
{
  
//----------------------------------------------------------------------------------------------------------------------
// A minimal agent scanning the encironment with a distance sensor
//----------------------------------------------------------------------------------------------------------------------
class Bacterium : public SMCAgent
{
  
public:
  
  // Inherited from Model
  virtual void update(float dt);
  virtual void reset();
  virtual void init();
};
  
} // namespace

#endif
