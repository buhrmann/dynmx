/*
 *  DistanceSensor.cpp
 *  cinder_dynmx
 *
 *  Created by Thomas Buhrmann on 12/16/11.
 *  Copyright 2011 __MyCompanyName__. All rights reserved.
 *
 */

#include "DistanceSensor.h"

namespace dmx
{

// Statics  
//----------------------------------------------------------------------------------------------------------------------          
std::string DistanceSensor::s_transFuncNames [kTF_NumFunctions] = {"Identity", "Inverse", "Binary"};  
  
//----------------------------------------------------------------------------------------------------------------------        
void DistanceSensor::init() 
{
  setTransferFunction(kTF_Inverse);
  m_distance = m_maxDistance;
  m_activation = m_transferFunction(m_distance);
}
  
//----------------------------------------------------------------------------------------------------------------------        
void DistanceSensor::setTransferFunction(int actFuncName) 
{ 
  switch (actFuncName) 
  {
    case kTF_Binary:
      m_transferFunction = &tfBinary;
      break;
    case kTF_Identity:
      m_transferFunction = &tfIdentity;
      break;      
    case kTF_Inverse:
    default:
      m_transferFunction = &tfInverse;
      break;      
  }
}

//----------------------------------------------------------------------------------------------------------------------        
void DistanceSensor::setTransferFunction(const std::string& name)
{
  for(int i = 0; i < kTF_NumFunctions; ++i)
  {
    if(name == s_transFuncNames[i])
      setTransferFunction(i);
  }
}

//----------------------------------------------------------------------------------------------------------------------        
const std::string& DistanceSensor::getTransferFunctionName() const
{
  if(m_transferFunction == &tfBinary)
    return s_transFuncNames[kTF_Binary];
  else if(m_transferFunction == &tfIdentity)
    return s_transFuncNames[kTF_Identity];
  else
    return s_transFuncNames[kTF_Inverse];
}

//----------------------------------------------------------------------------------------------------------------------      
float DistanceSensor::senseEnvironment(SMCEnvironment& environment)
{
  ci::Vec2f end = m_position + m_direction * m_maxDistance;  
  m_distance = m_maxDistance;  
  ci::Vec2f collision;  

  for(int i = 0; i < environment.getObjects().size(); i++)
  {
    const Positionable& obj = *environment.getObjects()[i];
    if(obj.isVisible())
    {
      float distance;
      
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
      
      // Check if closer than previously found collision
      if(distance > 0 && distance < m_distance)
      {
        m_distance = distance;
        m_collision = collision;
      }
    }
  }    

  m_activation = m_transferFunction(m_distance / m_maxDistance);
  return m_activation;
}
  
//----------------------------------------------------------------------------------------------------------------------        
void DistanceSensor::toXml(ci::XmlTree& xml)
{
  xml.push_back(ci::XmlTree("MaxSensorDist", toString(m_maxDistance)));
  xml.push_back(ci::XmlTree("SensorTransferFunc", getTransferFunctionName()));
}
  
}