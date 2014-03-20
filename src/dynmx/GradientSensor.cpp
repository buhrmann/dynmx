/*
 *  GradientSensor.cpp
 *  cinder_dynmx
 *
 *  Created by Thomas Buhrmann on 12/16/11.
 *  Copyright 2011 __MyCompanyName__. All rights reserved.
 *
 */

#include "GradientSensor.h"
#include "Random.h"

namespace dmx
{
  
//----------------------------------------------------------------------------------------------------------------------
void GradientSensor::updateActivation(SMCEnvironment& environment, float dt)
{
  m_activation = 0.0f;
  float distance;
  
  for(int i = 0; i < environment.getObjects().size(); i++)
  {
    const Positionable& obj = *environment.getObjects()[i];
    if(obj.getType() == Positionable::kObj_Gradient && obj.isVisible())
    {
      const Gradient& g = (Gradient&)obj;
      distance = g.getPosition().distance(m_position);
      
      if(distance <= g.getRadius())
      {
        m_activation += 1.0f - (distance / g.getRadius());
      }
    }
  }
}
  
//----------------------------------------------------------------------------------------------------------------------
void GradientSensor::fromXml(const ci::XmlTree& xml)
{
  Sensor::fromXml(xml);
  m_angle = xml.getAttributeValue<float>("Angle", 0.0);
}
  
//----------------------------------------------------------------------------------------------------------------------
void GradientSensor::toXml(ci::XmlTree& xml)
{
  Sensor::toXml(xml);
  xml.setAttribute("Angle", m_angle);
}
  

} // namespace