//
//  GradientSensor.h
//  dynmx
//
//  Created by Thomas Buhrmann on 25/02/14.
//
//

#ifndef dynmx_TorusSensor_h
#define dynmx_TorusSensor_h

#include "Dynmx.h"
#include "Sensor.h"

namespace dmx
{
  
//----------------------------------------------------------------------------------------------------------------------
// A 2d torus gradient sensor
// ---------------------------------------------------------------------------------------------------------------------
class TorusSensor : public GradientSensor
{
  
public:
  
  virtual void updateActivation(SMCEnvironment& environment, float dt);
  
}; // class
  
  
// Inlines
//----------------------------------------------------------------------------------------------------------------------
inline void TorusSensor::updateActivation(SMCEnvironment& environment, float dt)
{
  m_activation = 0.0f;
  
  for(int i = 0; i < environment.getObjects().size(); i++)
  {
    const Positionable& obj = *environment.getObjects()[i];
    if(obj.getType() == Positionable::kObj_Torus && obj.isVisible())
    {
      const Torus& t = (Torus&)obj;
      float w = t.getWidth();
      float distance = t.relDistanceFromCentralLine(m_position);
      float c = t.getPeak() * exp(-(distance*distance) / (2*w*w));
      
      m_activation += c;
    }
  }
}

} // namespace
#endif
