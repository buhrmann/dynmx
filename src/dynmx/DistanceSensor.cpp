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

//----------------------------------------------------------------------------------------------------------------------      
float DistanceSensor::senseEnvironment(SMCEnvironment& environment)
{
  ci::Vec2f end = m_position + m_direction * m_maxDistance;  
  m_distance = m_maxDistance;  
  ci::Vec2f collision;  
  for(int i = 0; i < environment.getCircles().size(); i++)
  {
    const Circle& circle = environment.getCircles()[i];
    float distance = lineSegmentCircleClosestIntersect(collision, m_position, end, circle.getPosition(), circle.getRadius());
    if(distance > 0 && distance < m_distance)
    {
      m_distance = distance;
      m_collision = collision;
    }
  }
  
  for(int i = 0; i < environment.getTriangles().size(); i++)
  {
    const Triangle& tri = environment.getTriangles()[i];
    float distance = raySegmentTriangleIntersect(collision, m_position, end, tri.p1, tri.p2, tri.p3);
    if(distance > 0 && distance < m_distance)
    {
      m_distance = distance;
      m_collision = collision;
    }    
  }
  
  return m_distance;
}
  
}