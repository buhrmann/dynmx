//
//  GradientSensor.h
//  dynmx
//
//  Created by Thomas Buhrmann on 25/02/14.
//
//

#ifndef dynmx_GradientSensor_h
#define dynmx_GradientSensor_h

#include "Dynmx.h"
#include "Sensor.h"

namespace dmx
{
  
//----------------------------------------------------------------------------------------------------------------------
// A 2d gradient sensor
// ---------------------------------------------------------------------------------------------------------------------
class GradientSensor : public Sensor
{
  
public:
  

  GradientSensor() : m_angle(0) { init(); };
  GradientSensor(const ci::Vec2f& position, float angle) : Sensor(position), m_angle(angle) { init(); };
  ~GradientSensor() {};
  
  virtual void toXml(ci::XmlTree& xml);
  virtual void fromXml(const ci::XmlTree& xml);
  
  float getAngle() const { return m_angle; };
  
protected:
  
  virtual void updateActivation(SMCEnvironment& environment, float dt);
  
  float m_angle;
}; // class
  
} // namespace
#endif
