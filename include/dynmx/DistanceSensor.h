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
#include "Sensor.h"
#include "CollisionDetection.h"

namespace dmx
{
  
//----------------------------------------------------------------------------------------------------------------------    
// A 2d distance sensor that performs variois collision detection routines
// ---------------------------------------------------------------------------------------------------------------------
class DistanceSensor : public Sensor
{
  
public:
  
  DistanceSensor(float maxDist = 1.0f) : m_maxDistance(maxDist), m_direction(0.0f, 1.0f) { init(); };
  DistanceSensor(float maxDist, const ci::Vec2f& position, const ci::Vec2f& direction) : Sensor(position), m_direction(direction), m_maxDistance(maxDist) { init(); };
  virtual ~DistanceSensor() {};
  
  virtual void init();
  virtual void reset();
  
  void setMaxDistance(float maxDist) { m_maxDistance = maxDist; };
  void setDirection(const ci::Vec2f& dir) { m_direction = dir; };
  
  const ci::Vec2f& getDirection() const { return m_direction; };
  const ci::Vec2f& getCollision() const { return m_collision; };  
  float getDistance() const { return m_distance; };
  float getDistanceProportional() const { return m_distance / m_maxDistance; };
  float getMaxDistance() const { return m_maxDistance; };
  bool isColliding() const { return m_distance < m_maxDistance; };
  
  virtual void toXml(ci::XmlTree& xml);
  virtual void fromXml(const ci::XmlTree& xml);

protected:
  
  void updateActivation(SMCEnvironment& environment, float dt);
  
  // State
  ci::Vec2f m_direction;
  ci::Vec2f m_collision;
  float m_distance;
  
  // Parameters
  float m_maxDistance;
  
}; // class
  
} // namespace
#endif