/*
 *  DistanceSensor.cpp
 *  cinder_dynmx
 *
 *  Created by Thomas Buhrmann on 12/16/11.
 *  Copyright 2011 __MyCompanyName__. All rights reserved.
 *
 */

#include "DistanceSensor.h"
#include "Random.h"

namespace dmx
{
  
//----------------------------------------------------------------------------------------------------------------------        
void DistanceSensor::init() 
{
  Sensor::init();
  
  m_distance = m_maxDistance;
  m_activation = m_transferFunction(m_distance);
  m_activationDelayed = Delay(0.06, 0.02, m_activation);
}
  
//----------------------------------------------------------------------------------------------------------------------
void DistanceSensor::reset()
{
  Sensor::reset();
  m_distance = 0.0f;
}
  

//----------------------------------------------------------------------------------------------------------------------      
void DistanceSensor::updateActivation(SMCEnvironment& environment, float dt)
{
  ci::Vec2f end = m_position + m_direction * m_maxDistance;  
  m_distance = m_maxDistance;  
  ci::Vec2f collision;  

  for(int i = 0; i < environment.getObjects().size(); i++)
  {
    const Positionable& obj = *environment.getObjects()[i];
    if(obj.isVisible())
    {
      float distance = -1;
      
      // Object specific collision detection
      if(obj.getType() == Positionable::kObj_Line)
      {
        const Line& line = (Line&)obj;
        bool collided = lineSegmentIntersect(collision, m_position, end, line.getStart(), line.getEnd());
        distance = collided ? collision.distance(m_position) : -1;
      }
      else if (obj.getType() == Positionable::kObj_Triangle)
      {
        const Triangle& tri = (Triangle&)obj;        
        distance = raySegmentTriangleIntersect(collision, m_position, end, tri.p1, tri.p2, tri.p3); 
      }
      else if (obj.getType() == Positionable::kObj_Circle)
      {
        const Circle& circle = (Circle&)obj;        
        distance = lineSegmentCircleClosestIntersect(collision, m_position, end, circle.getPosition(), circle.getRadius());
      }
      else if(obj.getType() == Positionable::kObj_Gaussian)
      {
        const Gaussian& gaussian = (Gaussian&)obj;
        distance = pointToGaussianDist(collision, m_position, gaussian);  
      }
      
      // Check if closer than previously found collision
      if(distance > 0 && distance < m_distance)
      {
        m_distance = distance;
        m_collision = collision;
      }
    }
  }    

  // Set activation level
  m_activation = m_transferFunction(m_distance / m_maxDistance);
}
  
//----------------------------------------------------------------------------------------------------------------------        
void DistanceSensor::toXml(ci::XmlTree& xml)
{
  Sensor::toXml(xml);
  xml.setAttribute("MaxDist", m_maxDistance);
}
  
//----------------------------------------------------------------------------------------------------------------------
void DistanceSensor::fromXml(const ci::XmlTree& xml)
{
  Sensor::fromXml(xml);
  m_maxDistance = xml.getAttributeValue<double>("MaxDist", 1.0);
}
  
  
}