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
SMCAgent::SMCAgent()
{
  if (SETTINGS->hasChild("Config/GA/Evolvable/Topology"))
  {
    m_topology.fromXml(SETTINGS->getChild("Config/GA/Evolvable/Topology"));
    m_ctrnn = new CTRNN(m_topology.getSize());
  }
  init();
}
  
  
//----------------------------------------------------------------------------------------------------------------------
SMCAgent::SMCAgent(const Topology& top)
{
  m_topology = top;
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
  m_radius = 0.02;
  
  if (SETTINGS->hasChild("Config/GA/Evolvable"))
  {
    const ci::XmlTree& xml = SETTINGS->getChild("Config/GA/Evolvable");
    
    // Todo: move to respective sensor class!!!
    // Distance sensor
    if (xml.getChild("DistanceSensor").getValue<bool>(0))
    {
      m_distanceSensor = new DistanceSensor();
      m_distanceSensor->fromXml(xml.getChild("DistanceSensor"));
      setSensorMode(xml.getChild("DistanceSensor").getAttributeValue<std::string>("Mode", "Absolute"));
    }
    else
    {
      m_distanceSensor = NULL;
    }
    
    // Gradient sensor
    if (xml.getChild("GradientSensor").getValue<bool>(0))
    {
      m_gradientSensor = new GradientSensor();
      m_gradientSensor->fromXml(xml.getChild("GradientSensor"));
    }
    else
    {
      m_gradientSensor = NULL;
    }
    
    // Torus sensor
    if (xml.getChild("TorusSensor").getValue<bool>(0))
    {
      m_torusSensor = new TorusSensor();
      m_torusSensor->fromXml(xml.getChild("TorusSensor"));
    }
    else
    {
      m_torusSensor = NULL;
    }

    
    setMaxSpeed(xml.getChild("MaxSpeed").getValue<double>(1.0));
    setMaxAngularSpeed(degreesToRadians(xml.getChild("MaxAngularSpeed").getValue<double>(180)));
    setMaxAngle(degreesToRadians(xml.getChild("MaxAngle").getValue<double>(90)));
    setMaxPosition(xml.getChild("MaxPosition").getValue<double>(0.5));
    setAngleWraps(xml.getChild("AngleWraps").getValue<bool>(1));
    setPositionWraps(xml.getChild("PositionWraps").getValue<bool>(1));
    m_energyInitial = xml.getChild("Energy").getAttributeValue<float>("initial", 30.0);
    m_energySpeedTresh = xml.getChild("Energy").getAttributeValue<float>("threshForSpeed", -1);
    
    m_evMin = xml.getChild("Energy").getAttributeValue<float>("evMin", 0);
    m_evMax = xml.getChild("Energy").getAttributeValue<float>("evMax", 10);
    
    getEnvironment().fromXml(xml.getChild("Environment"));
  }
  
  reset();
}
  
//----------------------------------------------------------------------------------------------------------------------
void SMCAgent::reset()
{
  m_time = 0.0f;
  m_angle = 0.0f;
  m_angularSpeed = 0.0;
  m_energy = m_energyInitial;
  m_food = 0.0f;
  m_sensedValue = 0.0f;
  
  setPosition(ci::Vec2f(0,0));
}

//----------------------------------------------------------------------------------------------------------------------
void SMCAgent::setPosition(const ci::Vec2f& pos)
{
  m_position = pos;
  m_velocity = ci::Vec2f(0.0f, 0.0f);

  if(m_distanceSensor){
    m_distanceSensor->reset();
    updateDistanceSensor(0.001f);
  }
  
  if(m_gradientSensor){
    m_gradientSensor->reset();
    updateGradientSensor(0.001f);
  }
  
  if(m_torusSensor){
    m_torusSensor->reset();
    updateTorusSensor(0.001f);
  }

}
  
//----------------------------------------------------------------------------------------------------------------------
void SMCAgent::update(float dt)
{  
  // Sense environment
  if(hasDistanceSensor())
  {
    m_sensedValue = m_distanceSensor->getActivation();
    m_sensedValueDerivative = m_distanceSensor->getDerivative();

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
  }
   
  // Metabolise
  m_food = m_sensedValue * 10;
  float dA =  - m_energy -(0.075*m_energy*m_energy*m_energy) + (0.5*m_food*m_energy*m_energy);
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
void SMCAgent::updateSensors(float dt)
{
  if(hasDistanceSensor())
    updateDistanceSensor(dt);
  
  if(hasGradientSensor())
    updateGradientSensor(dt);
  
  if(hasTorusSensor())
    updateTorusSensor(dt);
}
  
//----------------------------------------------------------------------------------------------------------------------
float SMCAgent::updateDistanceSensor(float dt)
{
  m_distanceSensor->setPosition(m_position);
  m_distanceSensor->setDirection(ci::Vec2f(cos(m_angle), sin(m_angle)));
  return m_distanceSensor->sense(m_environment, dt);
}
  
//----------------------------------------------------------------------------------------------------------------------
float SMCAgent::updateGradientSensor(float dt)
{
  float a = m_gradientSensor->getAngle();
  ci::Vec2f spos = m_position + ci::Vec2f(m_radius*cos(m_angle + a), m_radius*sin(m_angle + a));
  m_gradientSensor->setPosition(spos);
  return m_gradientSensor->sense(m_environment, dt);
}
  
//----------------------------------------------------------------------------------------------------------------------
float SMCAgent::updateTorusSensor(float dt)
{
  float a = m_torusSensor->getAngle();
  ci::Vec2f spos = m_position + ci::Vec2f(m_radius*cos(m_angle + a), m_radius*sin(m_angle + a));
  m_torusSensor->setPosition(spos);
  return m_torusSensor->sense(m_environment, dt);
}

//----------------------------------------------------------------------------------------------------------------------
float SMCAgent::getSensedEnergy()
{
  if (m_energy > m_evMax)
    return 1;
  else if (m_energy > m_evMin)
    return 1.0f - ((m_evMax - m_energy) / (m_evMax - m_evMin));
  else
    return 0;
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
  if(m_distanceSensor != NULL)
    m_distanceSensor->toXml(xml);
  
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
  recorder.push_back("AngularSpeed", m_angularSpeed);
  recorder.push_back("Sensor", m_sensedValue);  
  recorder.push_back("SensorDer", m_sensedValueDerivative);    
  
  // ctrnn
  m_ctrnn->record(recorder);
}  
                                
} // namespace