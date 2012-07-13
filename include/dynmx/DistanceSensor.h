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
#include "Signal.h"
#include "cinder/Vector.h"

namespace dmx
{
  
//----------------------------------------------------------------------------------------------------------------------    
// A 2d distance sensor that performs variois collision detection routines
// ---------------------------------------------------------------------------------------------------------------------
class DistanceSensor
{
  
public:
  
  // Define a type of pointer to transfer function
  typedef double (*TransferFunction)(double);
  
  // Name the activation functions
  enum TransferFunctions
  {
    kTF_Identity,
    kTF_Inverse,
    kTF_Binary,
    kTF_NumFunctions
  };
  
  static std::string s_transFuncNames [kTF_NumFunctions];
  
  DistanceSensor(float maxDist = 1.0f) : m_position(0.0f, 0.0f), m_direction(0.0f, 1.0f), m_maxDistance(maxDist) { init(); };
  DistanceSensor(float maxDist, const ci::Vec2f& position, const ci::Vec2f& direction) : 
    m_position(position), m_direction(direction), m_maxDistance(maxDist) { init(); };
  ~DistanceSensor() {};
  
  void init();
  void reset() { m_activationDt.reset(); m_activationDelayed.reset(); };
  
  void setMaxDistance(float maxDist) { m_maxDistance = maxDist; };
  void setNoiseLevel(float level) { m_noiseLevel = level; };
  void setPosition(const ci::Vec2f& pos) { m_position = pos; };
  void setDirection(const ci::Vec2f& dir) { m_direction = dir; };
  void setTransferFunction(double (*pt2Func)(double)) { m_transferFunction = pt2Func; };
  void setTransferFunction(int actFuncName);
  void setTransferFunction(const std::string& name);
  
  float senseEnvironment(SMCEnvironment& environment, float dt);
  
  const ci::Vec2f& getPosition() const { return m_position; };
  const ci::Vec2f& getDirection() const { return m_direction; };
  const ci::Vec2f& getCollision() const { return m_collision; };  
  float getDistance() const { return m_distance; };
  float getDistanceProportional() const { return m_distance / m_maxDistance; };
  float getActivation() const { return m_activation; };
  float getDerivative() const { return m_activationDt.get(); };
  float getDelayed() const { return m_activationDelayed.get(); };
  float getMaxDistance() const { return m_maxDistance; };
  bool isColliding() const { return m_distance < m_maxDistance; };
  const std::string& getTransferFunctionName() const;
  
  void toXml(ci::XmlTree& xml);  

protected:
  
  static inline double tfIdentity(double distProp) { return distProp; };
  static inline double tfInverse(double distProp) { return 1.0 - distProp; };
  static inline double tfBinary(double distProp) { return  distProp < 1; };
  
  ci::Vec2f m_position;
  ci::Vec2f m_direction;
  
  ci::Vec2f m_collision;
  float m_maxDistance;
  float m_distance;
  float m_activation;
  Derivative m_activationDt; 
  Delay m_activationDelayed;
  
  float m_noiseLevel;
  
  TransferFunction m_transferFunction;
  
}; // class
  
} // namespace
#endif