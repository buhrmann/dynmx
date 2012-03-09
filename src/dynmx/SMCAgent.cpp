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
  m_maxSpeed(5.0f), 
  m_maxAngularSpeed(TWO_PI), // rad/s
  m_radius(0.03f)
{
  m_ctrnn = new CTRNN(numNeurons);
  
  init();
}

//----------------------------------------------------------------------------------------------------------------------
SMCAgent::~SMCAgent()
{
  delete m_ctrnn;
}

//----------------------------------------------------------------------------------------------------------------------
void SMCAgent::init()
{
  reset();
}
  
//----------------------------------------------------------------------------------------------------------------------
void SMCAgent::reset()
{
  m_time = 0.0f;
  m_angle = 0.0f;
  m_angularVelocity = 0.0;
  m_position = ci::Vec2f(0.0f, 0.0f);
  m_velocity = ci::Vec2f(0.0f, 0.0f);
  
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
  m_angularVelocity = m_maxAngularSpeed * (m_ctrnn->getOutput(1) - m_ctrnn->getOutput(2));
  m_angle += m_angularVelocity * dt; 

  // Map angle into [-pi, pi]
  if (m_angle > PI)
  {
    m_angle = -(TWO_PI - m_angle);
  }
  else if (m_angle < -PI)
  {
    m_angle = TWO_PI + m_angle;
  }

  m_velocity = m_maxSpeed * (m_ctrnn->getOutput(1) - m_ctrnn->getOutput(2)) * ci::Vec2f(0, 1);
  m_position += m_velocity * dt;
  
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