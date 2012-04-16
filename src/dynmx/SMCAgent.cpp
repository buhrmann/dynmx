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
  static const int motTrans2 = m_ctrnn->getSize() - 1; // last one
  static const int motTrans1 = motTrans2 - 3;  
  static const int motRot1 = motTrans2 - 2;  
  static const int motRot2 = motTrans2 - 1;

  
  // Sense environment
  updateSensor(dt);
  m_sensedValue = m_distanceSensor.getActivation();  

  const bool angleDependentSensor = false;
  if(angleDependentSensor)
  {
    m_sensedValue = sign(m_angle) * m_distanceSensor.getActivation();
  }
  
  // Update neural network
  if(m_angle < 0)
  {
    m_ctrnn->setExternalInput(0, m_sensedValue);    
    m_ctrnn->setExternalInput(1, 0.0);    
  }
  else 
  {
    m_ctrnn->setExternalInput(0, 0.0);    
    m_ctrnn->setExternalInput(1, m_sensedValue);
  }

  m_ctrnn->updateDynamic(dt);
  
  // Update agent movement
  m_angularVelocity = m_maxAngularSpeed * (m_ctrnn->getOutput(motRot1) - m_ctrnn->getOutput(motRot2));
  m_angularVelocity += UniformRandom(-m_maxAngularSpeed / 100.0, m_maxAngularSpeed / 100.0);
  m_angle += m_angularVelocity * dt; 
  if(m_angleWraps)
    m_angle = wrap(m_angle, -m_maxAngle, m_maxAngle);
  else 
    m_angle = clamp(m_angle, -m_maxAngle, m_maxAngle);

  // Update positions 
  m_velocity = m_maxSpeed * (m_ctrnn->getOutput(motTrans1) - m_ctrnn->getOutput(motTrans2)) * ci::Vec2f(0, 1);
  m_position += m_velocity * dt;
  if(m_positionWraps)
    m_position.y = wrap(m_position.y, -m_maxPosition, m_maxPosition);
  else 
    m_position.y = clamp(m_position.y, -m_maxPosition, m_maxPosition);
  
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
  
//----------------------------------------------------------------------------------------------------------------------  
void SMCAgent::toXml(ci::XmlTree& xml)
{
  m_distanceSensor.toXml(xml);
  
  xml.push_back(ci::XmlTree("MaxSpeed", toString(m_maxSpeed)));
  xml.push_back(ci::XmlTree("MaxAngularSpeed", toString(radiansToDegrees(m_maxAngularSpeed))));
  xml.push_back(ci::XmlTree("MaxPosition", toString(m_maxPosition)));
  xml.push_back(ci::XmlTree("PositionWraps", toString(m_positionWraps)));
  xml.push_back(ci::XmlTree("MaxAngle", toString(radiansToDegrees(m_maxAngle))));
  xml.push_back(ci::XmlTree("AngleWraps", toString(m_angleWraps)));
    
  // ctrnn
  m_ctrnn->toXml(xml);
}

//----------------------------------------------------------------------------------------------------------------------  
void SMCAgent::record(Recorder& recorder)
{
  /*
  ci::Vec2f m_position;
  ci::Vec2f m_velocity;
  float m_angle;  
  float m_angularVelocity;  
  float m_sensedValue;
   */
  
  // sensor 
  // ctrnn
}  
                                
} // namespace