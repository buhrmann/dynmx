//
//  SMCArm.cpp
//  dynmx
//
//  Created by Thomas Buhrmann on 8/9/13.
//
//

#include "SMCArm.h"
#include "MathUtils.h"
#include "cinder/Rand.h"

namespace dmx
{

//----------------------------------------------------------------------------------------------------------------------
SMCArm::SMCArm()
{
  init();
}

//----------------------------------------------------------------------------------------------------------------------
SMCArm::~SMCArm()
{
  delete m_ctrnn;
}

//----------------------------------------------------------------------------------------------------------------------
void SMCArm::init()
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
    m_trialDuration = xml.getChild("TrialDuration").getValue<double>(10.0);
    m_fitnessEvalDelay = xml.getChild("FitnessEvalDelay").getValue<double>(8.0);
    
    // Load environment objects
    m_environment.fromXml(xml.getChild("Environment"));
  }
  
  m_arm.init();
  m_arm.m_pd[0].m_P = 10.0f;
  m_arm.m_pd[0].m_D = 1.0f;
  
  m_arm.m_pd[1].m_P = 20.0f;
  m_arm.m_pd[1].m_D = 2.0f;
  
  reset();
}

//----------------------------------------------------------------------------------------------------------------------
void SMCArm::reset()
{
  m_time = 0.0f;
  
  m_fitness = 0;
  m_ctrnn->zeroStates();
  m_arm.reset();
}


//----------------------------------------------------------------------------------------------------------------------
void SMCArm::update(float dt)
{

  // Sense environment
  updateSensor(dt);
  m_sensedValue = m_distanceSensor.getActivation();
  
  // Update CTRNN
  m_ctrnn->setExternalInput(0, m_sensedValue);
  m_ctrnn->setExternalInput(1, m_arm.getJointAngle(JT_elbow));
  m_ctrnn->setExternalInput(2, m_arm.getJointAngle(JT_shoulder));
  m_ctrnn->updateDynamic(dt);
  
  const int mn1Id = m_topology.getSize() - 1;
  const int mn2Id = m_topology.getSize() - 2;
  
  float desAng1 = -PI_OVER_TWO + PI * m_ctrnn->getOutput(mn1Id);
  float desAng2 = -PI_OVER_TWO + PI * m_ctrnn->getOutput(mn2Id);
  
  //desAng1 = sin(m_time);
  //desAng2 = cos(m_time);
  
  // Move
  m_arm.updatePD(dt, desAng1, desAng2);
  
  m_time += dt;
  
  updateFitness();
}

//----------------------------------------------------------------------------------------------------------------------
void SMCArm::updateSensor(float dt)
{
  m_distanceSensor.setPosition(m_arm.getEffectorPos());
  ci::Vec2f dir = (m_arm.getEffectorPos() - m_arm.getElbowPos()).normalized();
  m_distanceSensor.setDirection(dir);
  m_distanceSensor.senseEnvironment(m_environment, dt);
}


//----------------------------------------------------------------------------------------------------------------------
void SMCArm::setSensorMode(const std::string& mode)
{
  if(mode == "Absolute")
    m_sensorMode = kSensorMode_Absolute;
  else if (mode == "Derivative")
    m_sensorMode = kSensorMode_Derivative;
  else if (mode == "AbsAndDelayed")
    m_sensorMode = kSensorMode_AbsAndDelayed;
}

//----------------------------------------------------------------------------------------------------------------------
void SMCArm::updateFitness()
{
  m_fitness = 0;
}

//----------------------------------------------------------------------------------------------------------------------
float SMCArm::getFitness()
{
  return m_fitness / m_trialDuration;
}

//----------------------------------------------------------------------------------------------------------------------
void SMCArm::nextTrial(int trial)
{
  reset();
}

//----------------------------------------------------------------------------------------------------------------------
bool SMCArm::hasFinished()
{
  return m_time >= m_trialDuration;
};

//----------------------------------------------------------------------------------------------------------------------
int SMCArm::getNumGenes()
{
  return m_topology.getNumParameters();
}

//----------------------------------------------------------------------------------------------------------------------
void SMCArm::decodeGenome(const double* genome)
{
  m_topology.decode(*m_ctrnn, genome, m_netLimits);
}

//----------------------------------------------------------------------------------------------------------------------
void SMCArm::toXml(ci::XmlTree& xml)
{
  ci::XmlTree evolvable ("SMCArm", "");
  evolvable.setAttribute("Fitness", getFitness());
  
  m_distanceSensor.toXml(evolvable);
  
  evolvable.push_back(ci::XmlTree("TrialDuration", toString(m_trialDuration)));
  evolvable.push_back(ci::XmlTree("FitnessEvalDelay", toString(m_fitnessEvalDelay)));
  
  m_topology.toXml(evolvable);
  m_netLimits.toXml(evolvable);
  m_ctrnn->toXml(evolvable);
  
  xml.push_back(evolvable);
}

//----------------------------------------------------------------------------------------------------------------------
void SMCArm::record(Recorder& recorder)
{
  m_arm.record(recorder);
  m_ctrnn->record(recorder);
}  
  
} // namespace