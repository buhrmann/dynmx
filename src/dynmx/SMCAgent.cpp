/*
 *  SMCAgent.cpp
 *  cinder_dynmx
 *
 *  Created by Thomas Buhrmann on 12/18/11.
 *  Copyright 2011 __MyCompanyName__. All rights reserved.
 *
 */

#include "SMCAgent.h"
#include "MathUtils.h"

namespace dmx
{
  
//----------------------------------------------------------------------------------------------------------------------
SMCAgent::SMCAgent(const Topology& top) : 
  m_topology(top),
  m_maxSpeed(5.0f), 
  m_maxAngularSpeed(TWO_PI), // rad/s
  m_radius(0.03f)
{
  m_ctrnn = new CTRNN(top.getSize());
  
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
  m_energy = 3.0f;
  m_food = 0.0f;
  setPosition(ci::Vec2f(0,0));
}  

//----------------------------------------------------------------------------------------------------------------------
void SMCAgent::setPosition(const ci::Vec2f& pos)
{
  m_position = pos;
  m_velocity = ci::Vec2f(0.0f, 0.0f);
  m_distanceSensor.reset();
  updateSensor(0.001f);
}
  
//----------------------------------------------------------------------------------------------------------------------
void SMCAgent::update(float dt)
{  
  // Sense environment
  updateSensor(dt);
  m_sensedValue = m_distanceSensor.getActivation();  
  m_sensedValueDerivative = m_distanceSensor.getDerivative();  

  switch(m_sensorMode)
  {
    case kSensorMode_Absolute:
    default:
      m_ctrnn->setExternalInput(0, m_sensedValue / dt);
      break;
    case kSensorMode_Derivative:
      m_ctrnn->setExternalInput(0, m_sensedValueDerivative);
      break;
  }
  
  // Metabolise
  m_food = m_sensedValue * 10;
  float dA = (-0.075*m_energy*m_energy*m_energy) + (0.5*m_food*m_energy*m_energy) - m_energy;
  m_energy += dA * dt;

  m_ctrnn->updateDynamic(dt);
  
#if 0  
  // Update agent movement
  static int motTrans1, motTrans2;  
  if(m_topology.getNumOutputs() >= 4)
  {
    motTrans1 = m_topology.getSize() - 4;
    motTrans2 = m_topology.getSize() - 1;
    static const int motRot1 = m_topology.getSize() - 3;  
    static const int motRot2 = m_topology.getSize() - 2;    
    m_angularVelocity = m_maxAngularSpeed * (m_ctrnn->getOutput(motRot1) - m_ctrnn->getOutput(motRot2));
    m_angularVelocity += UniformRandom(-m_maxAngularSpeed / 100.0, m_maxAngularSpeed / 100.0);
    m_angle += m_angularVelocity * dt; 
    if(m_angleWraps)
      m_angle = wrap(m_angle, -m_maxAngle, m_maxAngle);
    else 
      m_angle = clamp(m_angle, -m_maxAngle, m_maxAngle);
  }
#endif

  // Update positions 
  if(m_topology.getNumOutputs() == 2)
  {
    float motTrans1 = m_topology.getSize() - 2;
    float motTrans2 = m_topology.getSize() - 1;    
    m_velocity = m_maxSpeed * (m_ctrnn->getOutput(motTrans1) - m_ctrnn->getOutput(motTrans2)) * ci::Vec2f(0, 1);    
  }
  else if(m_topology.getNumOutputs() == 1)
  {
    float motTrans1 = m_topology.getSize() - 1;
    m_velocity = m_maxSpeed * ((-1.0 + 2.0 * m_ctrnn->getOutput(motTrans1)) * ci::Vec2f(0, 1));    
  }
  

  m_position += m_velocity * dt;
  if(m_positionWraps)
    m_position.y = wrap(m_position.y, -m_maxPosition, m_maxPosition);
  /*else 
    m_position.y = clamp(m_position.y, -m_maxPosition, m_maxPosition);
  */
  m_time += dt;
}

//----------------------------------------------------------------------------------------------------------------------
void SMCAgent::updateSensor(float dt)
{
  m_distanceSensor.setPosition(m_position);
  m_distanceSensor.setDirection(ci::Vec2f(cos(m_angle), sin(m_angle)));
  m_distanceSensor.senseEnvironment(m_environment, dt);
}

//----------------------------------------------------------------------------------------------------------------------  
float SMCAgent::getAngleWithHeading(ci::Vec2f pos)
{
  ci::Vec2f agentRelPos  = pos - getPosition();
  return atan2(agentRelPos.y, agentRelPos.x);
}

//----------------------------------------------------------------------------------------------------------------------  
void SMCAgent::setSensorMode(const std::string& mode)
{
  if(mode == "Absolute")
    m_sensorMode = kSensorMode_Absolute;
  else if (mode == "Derivative")
    m_sensorMode = kSensorMode_Derivative;
  else if (mode == "AbsAndDelayed")
    m_sensorMode = kSensorMode_AbsAndDelayed;
}
  
//----------------------------------------------------------------------------------------------------------------------  
void SMCAgent::toXml(ci::XmlTree& xml)
{
  m_distanceSensor.toXml(xml);
  
  xml.push_back(ci::XmlTree("MaxSpeed", toString(m_maxSpeed)));
  float maxAngSpeed = radiansToDegrees(m_maxAngularSpeed);
  xml.push_back(ci::XmlTree("MaxAngularSpeed", toString(maxAngSpeed)));
  xml.push_back(ci::XmlTree("MaxPosition", toString(m_maxPosition)));
  xml.push_back(ci::XmlTree("PositionWraps", toString(m_positionWraps)));
  float maxAngle = radiansToDegrees(m_maxAngle);
  xml.push_back(ci::XmlTree("MaxAngle", toString(maxAngle)));
  xml.push_back(ci::XmlTree("AngleWraps", toString(m_angleWraps)));
    
  // ctrnn
  m_ctrnn->toXml(xml);
}

//----------------------------------------------------------------------------------------------------------------------  
void SMCAgent::record(Recorder& recorder)
{
  recorder.push_back("PosX", m_position[0]);  
  recorder.push_back("PosY", m_position[1]);
  recorder.push_back("VelX", m_velocity[0]);  
  recorder.push_back("VelY", m_velocity[1]);  
  recorder.push_back("Angle", m_angle);  
  recorder.push_back("AngularVel", m_angularVelocity);  
  recorder.push_back("Sensor", m_sensedValue);  
  recorder.push_back("SensorDer", m_sensedValueDerivative);    
  
  // ctrnn
  m_ctrnn->record(recorder);
}  
                                
} // namespace