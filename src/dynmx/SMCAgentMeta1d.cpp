/*
 *  SMCAgentMeta1d.cpp
 *  cinder_dynmx
 *
 *  Created by Thomas Buhrmann on 12/18/11.
 *  Copyright 2011 __MyCompanyName__. All rights reserved.
 *
 */

#include "SMCAgentMeta1d.h"
#include "MathUtils.h"
#include "cinder/Rand.h"

namespace dmx
{
  
//----------------------------------------------------------------------------------------------------------------------
SMCAgentMeta1d::SMCAgentMeta1d()
{
  init();
}

//----------------------------------------------------------------------------------------------------------------------
SMCAgentMeta1d::~SMCAgentMeta1d()
{
  delete m_ctrnn;
}

//----------------------------------------------------------------------------------------------------------------------
void SMCAgentMeta1d::init()
{
  const ci::XmlTree* settings = SETTINGS;
  if (settings->hasChild("Config/GA/Evolvable"))
  {
    const ci::XmlTree& xml = settings->getChild("Config/GA/Evolvable");
    
    m_topology.fromXml(xml.getChild("Topology"));
    m_netLimits.fromXml(xml.getChild("NetLimits"));
    m_ctrnn = new CTRNN(m_topology.getSize());
    
    m_distanceSensor.setMaxDistance(xml.getChild("DistanceSensor").getAttributeValue<double>("MaxDist", 1.0));
    m_distanceSensor.setTransferFunction(xml.getChild("DistanceSensor").getAttributeValue<std::string>("TransferFunc", "Binary"));
    m_distanceSensor.setNoiseLevel(xml.getChild("DistanceSensor").getAttributeValue<float>("NoiseLevel", 0.0));
    setSensorMode(xml.getChild("DistanceSensor").getAttributeValue<std::string>("Mode", "Absolute"));
    setMaxSpeed(xml.getChild("MaxSpeed").getValue<double>(1.0));
    setMaxPosition(xml.getChild("MaxPosition").getValue<double>(0.5));
    setPositionWraps(xml.getChild("PositionWraps").getValue<bool>(1));
    m_trialDuration = xml.getChild("TrialDuration").getValue<double>(10.0);
    m_fitnessEvalDelay = xml.getChild("FitnessEvalDelay").getValue<double>(8.0);
    
    // Load environment objects
    m_environment.fromXml(xml.getChild("Environment"));
  }
    
  reset();
}

//----------------------------------------------------------------------------------------------------------------------
void SMCAgentMeta1d::reset()
{
  m_time = 0.0f;
  m_energy = 10.0f;
  m_food = 0.0f;
  m_maxSpeed = 1.0f;
  setPosition(0);
  
  m_fitness = 0;
  m_ctrnn->zeroStates();
}

//----------------------------------------------------------------------------------------------------------------------
void SMCAgentMeta1d::setPosition(float pos)
{
  m_position = pos;
  m_velocity = 0.0f;
  m_distanceSensor.reset();
  updateSensor(0.001f);
}

//----------------------------------------------------------------------------------------------------------------------
void SMCAgentMeta1d::update(float dt)
{
  // Update position first, so that it stays in sync for drawing with sensor
  m_position += m_velocity * dt;
  if(m_positionWraps)
    m_position = wrap(m_position, -m_maxPosition, m_maxPosition);
  else
    m_position = clamp(m_position, -m_maxPosition, m_maxPosition);

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
  
  const int motorNeuronId = m_topology.getSize() - 1;
  m_maxSpeed = clamp(m_energy, 0.f, 20.f) / 20.0;
  m_velocity = m_maxSpeed * (-1.0 + 2.0 * m_ctrnn->getOutput(motorNeuronId));
  
  m_time += dt;
  
  updateFitness();
}

//----------------------------------------------------------------------------------------------------------------------
void SMCAgentMeta1d::updateSensor(float dt)
{
  m_distanceSensor.setPosition(ci::Vec2f(0, m_position));
  m_distanceSensor.setDirection(ci::Vec2f(1, 0));
  m_distanceSensor.sense(m_environment, dt);
}


//----------------------------------------------------------------------------------------------------------------------
void SMCAgentMeta1d::setSensorMode(const std::string& mode)
{
  if(mode == "Absolute")
    m_sensorMode = kSensorMode_Absolute;
  else if (mode == "Derivative")
    m_sensorMode = kSensorMode_Derivative;
  else if (mode == "AbsAndDelayed")
    m_sensorMode = kSensorMode_AbsAndDelayed;
}
  
//----------------------------------------------------------------------------------------------------------------------
void SMCAgentMeta1d::updateFitness()
{
  if (m_maxSpeed > 0.01)
    m_fitness = m_time;
}
  
//----------------------------------------------------------------------------------------------------------------------
float SMCAgentMeta1d::getFitness()
{
  return m_fitness / m_trialDuration;
}

//----------------------------------------------------------------------------------------------------------------------
void SMCAgentMeta1d::nextTrial(int trial)
{
  setPosition(ci::Rand::randFloat(-m_maxPosition, m_maxPosition));
}

//----------------------------------------------------------------------------------------------------------------------
bool SMCAgentMeta1d::hasFinished()
{
  return m_time >= m_trialDuration;
};
  
//----------------------------------------------------------------------------------------------------------------------
int SMCAgentMeta1d::getNumGenes()
{
  return m_topology.getNumParameters();
}
  
//----------------------------------------------------------------------------------------------------------------------
void SMCAgentMeta1d::decodeGenome(const double* genome)
{
  m_topology.decode(*m_ctrnn, genome);
}

//----------------------------------------------------------------------------------------------------------------------
void SMCAgentMeta1d::toXml(ci::XmlTree& xml)
{
  ci::XmlTree evolvable ("SMCAgent", "");
  evolvable.setAttribute("Fitness", getFitness());
  
  m_distanceSensor.toXml(evolvable);
  
  evolvable.push_back(ci::XmlTree("MaxSpeed", toString(m_maxSpeed)));
  evolvable.push_back(ci::XmlTree("MaxPosition", toString(m_maxPosition)));
  evolvable.push_back(ci::XmlTree("PositionWraps", toString(m_positionWraps)));
  
  evolvable.push_back(ci::XmlTree("TrialDuration", toString(m_trialDuration)));
  evolvable.push_back(ci::XmlTree("FitnessEvalDelay", toString(m_fitnessEvalDelay)));

  m_topology.toXml(evolvable);
  m_netLimits.toXml(evolvable);
  m_ctrnn->toXml(evolvable);
  
  xml.push_back(evolvable);
}
  
//----------------------------------------------------------------------------------------------------------------------
void SMCAgentMeta1d::record(Recorder& recorder)
{
  recorder.push_back("PosX", 0);
  recorder.push_back("PosY", m_position);
  recorder.push_back("VelX", 0);
  recorder.push_back("VelY", m_velocity);
  recorder.push_back("Sensor", m_sensedValue);  
  recorder.push_back("SensorDer", m_sensedValueDerivative);    
  
  // ctrnn
  m_ctrnn->record(recorder);
}  

} // namespace