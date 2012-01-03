/*
 *  SMCAgent.cpp
 *  cinder_dynmx
 *
 *  Created by Thomas Buhrmann on 12/18/11.
 *  Copyright 2011 __MyCompanyName__. All rights reserved.
 *
 */

#include "SMCAgent.h"

namespace dmx
{
  
//----------------------------------------------------------------------------------------------------------------------
SMCAgent::SMCAgent(int numNeurons) : 
  m_position(ci::Vec2f(0.0f, 0.0f)), 
  m_angle(0.0f), 
  m_maxSpeed(5.0f), 
  m_radius(0.03f),
  m_sensedValue(0.0f),
  m_time(0.0)
{
  m_ctrnn = new CTRNN(numNeurons);
  init();
}

//----------------------------------------------------------------------------------------------------------------------
void SMCAgent::init()
{
  m_distanceSensor.setMaxDistance(0.5f);
  
  reset();
}
  
//----------------------------------------------------------------------------------------------------------------------
void SMCAgent::reset()
{
  m_time = 0.0f;
  m_angle = 0.0f;
  m_position = ci::Vec2f(0.0f, 0.0f);
  updateSensor(0.0f);
}  
  
//----------------------------------------------------------------------------------------------------------------------
void SMCAgent::update(float dt)
{
  // Sense
  updateSensor(dt);
  m_sensedValue = 1.0f - m_distanceSensor.getDistanceProportional();
  
  // Think
  m_ctrnn->setExternalInput(0, m_sensedValue);
  m_ctrnn->update(dt);
  
  // Act
  float motorDiff = m_ctrnn->getOutput(1) - m_ctrnn->getOutput(2);
  float vel = motorDiff * m_maxSpeed;
  m_angle += vel * dt; 
  
  // Map angle into [-pi, pi]
  if (m_angle > PI)
  {
    m_angle = -(TWO_PI - m_angle);
  }
  else if (m_angle < -PI)
  {
    m_angle = TWO_PI + m_angle;
  }
  
  m_time += dt;
}

//----------------------------------------------------------------------------------------------------------------------
void SMCAgent::updateSensor(float dt)
{
  m_distanceSensor.setPosition(m_position);
  m_distanceSensor.setDirection(ci::Vec2f(cos(m_angle), sin(m_angle)));
  m_distanceSensor.senseEnvironment(m_environment);
}

//----------------------------------------------------------------------------------------------------------------------  
float SMCAgent::getAngleWithHeading(ci::Vec2f pos)
{
  ci::Vec2f agentRelPos  = pos - getPosition();
  return atan2(agentRelPos.y, agentRelPos.x);
}
                                
} // namespace