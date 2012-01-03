/*
 *  DistanceSensor.h
 *  cinder_dynmx
 *
 *  Created by Thomas Buhrmann on 12/16/11.
 *  Copyright 2011 __MyCompanyName__. All rights reserved.
 *
 */

#ifndef _DMX_DIST_SENSOR_
#define _DMX_DIST_SENSOR_

#include "Dynmx.h"
#include "CollisionDetection.h"
#include "SMCEnvironment.h"
#include "cinder/Vector.h"

namespace dmx
{
  
//----------------------------------------------------------------------------------------------------------------------    
// A 2d distance sensor that performs variois collision detection routines
// ---------------------------------------------------------------------------------------------------------------------
class DistanceSensor
{
public:
  DistanceSensor(float maxDist = 1.0f) : m_position(0.0f, 0.0f), m_direction(0.0f, 1.0f), m_maxDistance(maxDist) {};
  DistanceSensor(float maxDist, const ci::Vec2f& position, const ci::Vec2f& direction) : 
    m_position(position), m_direction(direction), m_maxDistance(maxDist) {};
  ~DistanceSensor() {};
  
  void setMaxDistance(float maxDist) { m_maxDistance = maxDist; };
  void setPosition(const ci::Vec2f& pos) { m_position = pos; };
  void setDirection(const ci::Vec2f& dir) { m_direction = dir; };
  
  float senseEnvironment(SMCEnvironment& environment);
  
  const ci::Vec2f& getPosition() { return m_position; };
  const ci::Vec2f& getDirection() { return m_direction; };
  const ci::Vec2f& getCollision() { return m_collision; };  
  float getDistance() { return m_distance; };
  float getDistanceProportional() { return m_distance / m_maxDistance; };
  float getMaxDistance() { return m_maxDistance; };

protected:
  
  ci::Vec2f m_position;
  ci::Vec2f m_direction;
  
  ci::Vec2f m_collision;
  float m_maxDistance;
  float m_distance;
  
}; // class
  
} // namespace
#endif