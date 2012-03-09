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

  // Lines
  for(int i = 0; i < environment.getLines().size(); i++)
  {
    const Line& line = environment.getLines()[i];
    bool collided = lineSegmentIntersect(collision, m_position, end, line.getPosition(), line.getEnd());
    if(collided)
    {
      float distance = collision.distance(m_position);
      if(distance < m_distance)
      {
        m_distance = distance;
        m_collision = collision;
      }
    }
  }

  // Circles
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
  
  // Triangles
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