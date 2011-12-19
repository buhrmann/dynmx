/*
 *  SMCAgent.h
 *  cinder_dynmx
 *
 *  Created by Thomas Buhrmann on 12/18/11.
 *  Copyright 2011 __MyCompanyName__. All rights reserved.
 *
 */

#ifndef _DMX_SMC_AGENT_
#define _DMX_SMC_AGENT_

#include "Dynmx.h"
#include "Model.h"
#include "DistanceSensor.h"
#include "SMCEnvironment.h"
#include "CTRNN.h"
#include "cinder/Vector.h"

namespace dmx
{

//----------------------------------------------------------------------------------------------------------------------
// A minimal agent scanning the encironment with a distance sensor
//----------------------------------------------------------------------------------------------------------------------
class SMCAgent : public Model
{
  
public:
  
  SMCAgent();
  ~SMCAgent() {};
  
  // Inherited from class Model
  virtual void init();
  virtual void reset();
  virtual void update(float dt);
  
  SMCEnvironment* getEnvironment() { return &m_environment; };
  CTRNN* getCTRNN() { return &m_ctrnn; };  
  DistanceSensor& getDistanceSensor() { return m_distanceSensor; };
  const ci::Vec2f& getPosition() { return m_position; };
  float getAngle() { return m_angle; };
  float getRadius() { return m_radius; };
  float getSensedValue() { return m_sensedValue; };
  
protected:
  
  void updateSensor(float dt);
  
  SMCEnvironment m_environment;
  CTRNN m_ctrnn;
  DistanceSensor m_distanceSensor;
  ci::Vec2f m_position;
  float m_angle;
  float m_maxSpeed;
  float m_radius;
  float m_sensedValue;
}; // class
  
} // namespace

#endif