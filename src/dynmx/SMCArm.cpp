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
  
  // All limits are symmetric, equal between joints, and stored in rad
  m_maxJointAngle = abs(m_arm.getJointLimitUpper(JT_elbow));
  
  nextTrial(0);
  
  m_fitnessStage = 0;
  
  reset();
}

//----------------------------------------------------------------------------------------------------------------------
void SMCArm::reset()
{
  m_time = 0.0f;
  m_sensedValue = 0.0f;
  
  m_fitness = 0;
  m_fitHandVel = 0;
  m_fitHandDist = 0;
  m_fitAngleDist = 0;
  m_fitMaxSensor = 0;
  
  m_ctrnn->zeroStates();
  m_distanceSensor.reset();
  
  m_arm.reset();
  m_prevPos = m_arm.getEffectorPos();
  m_handTraj.clear();
  
  m_handSpeed = 0.0;
  m_handDist = 0.0;
  
  //m_fitnessStage = 0;
}


//----------------------------------------------------------------------------------------------------------------------
void SMCArm::update(float dt)
{

  // Sense environment
  updateSensor(dt);
  m_sensedValue = m_distanceSensor.getActivation();
  
  // Update CTRNN
  m_ctrnn->setExternalInput(0, m_time < m_fitnessEvalDelay ? m_sensedValue : 0.0f);
  m_ctrnn->setExternalInput(1, m_arm.getJointAngle(JT_elbow) / m_maxJointAngle);
  m_ctrnn->setExternalInput(2, m_arm.getJointAngle(JT_shoulder) / m_maxJointAngle);
  m_ctrnn->updateDynamic(dt);
  
  const int mn1Id = m_topology.getSize() - 1;
  const int mn2Id = m_topology.getSize() - 2;
  
  float desAng1 = -m_maxJointAngle + (2 * m_maxJointAngle * m_ctrnn->getOutput(mn1Id));
  float desAng2 = -m_maxJointAngle + (2 * m_maxJointAngle * m_ctrnn->getOutput(mn2Id));
  
  //desAng1 = 0.0;
  //desAng2 = 0.0;
  
  // Move
  m_prevPos = m_arm.getEffectorPos();
  m_arm.updatePD(dt, desAng1, desAng2);
  
  m_time += dt;
  
  // Store desired trajectory
  m_handTraj.push_back(m_arm.getEffectorPos());
  const int maxTrajPoints = 1000;
  if(m_handTraj.size() >= maxTrajPoints)
  {
    m_handTraj.pop_front();
  }

  
  updateFitness(dt);
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
void SMCArm::updateFitness(float dt)
{
  // Measure fitness components relative to line
  const std::vector<Positionable*>& objects = getEnvironment()->getObjects();
  const ci::Vec2f& s = ((Line*)objects[0])->getStart();
  const ci::Vec2f& e = ((Line*)objects[0])->getEnd();
  ci::Vec2f lineDir = (e - s).normalized();

  // Ensure a minimum averge hand speed
  const float minVel = 0.1f;
  ci::Vec2f handPos = m_arm.getEffectorPos();
  ci::Vec2f handVel = (handPos - m_prevPos) / dt;
  m_handSpeed = handVel.dot(lineDir);
  m_projVel = lineDir * m_handSpeed;
  m_fitHandVel = 1.0f - ((minVel - clamp(abs(m_handSpeed), 0.0f, minVel)) / minVel);
  
  // Punish distance from line
  const float maxDist = 0.1f;
  float handProj = (handPos - s).dot(lineDir);
  handProj = clamp(handProj, 0.0f, ((Line*)objects[0])->getLength());
  m_projPos = s + lineDir * handProj;
  m_fitHandDist = 1.0f - sqr(clamp(m_projPos.distance(handPos), 0.0f, maxDist) / maxDist);
  
  // Punish angular distance between line and hand velocity
  // Can only calculate angle if hand has a velocity > 0
  if (abs(m_handSpeed) > 0.0001f)
  {
    float dot = lineDir.dot(handVel.normalized());
    float angle = acos(clamp(dot, -1.f, 1.f));
    m_fitAngleDist = 1.0f - (angle/PI);
  }
  else
  {
    m_fitAngleDist = 0.0f;
  }
  
  // Encourage sensor use in the beginning
  m_instFit = 0;
  if(m_time < m_fitnessEvalDelay)
  {
    if (m_fitnessStage < 1)
    {
      m_instFit = m_sensedValue / m_fitnessEvalDelay;
    }
    else
    {
      //fit = min(min(m_sensedValue, m_fitHandVel), m_fitAngleDist);
      m_instFit = (m_fitHandDist * m_fitHandVel * m_fitAngleDist) / m_fitnessEvalDelay;
    }
  }
  
  // Only correct movement rewarded in final phase
  if(m_time >= m_fitnessEvalDelay)
  {
    if (m_fitnessStage < 1)
    {
      m_instFit += m_sensedValue / (m_trialDuration - m_fitnessEvalDelay);
    }
    else
    {
      m_instFit += (m_fitHandDist * m_fitHandVel * m_fitAngleDist) / (m_trialDuration - m_fitnessEvalDelay);
    }
  }

  m_fitness += dt * m_instFit;
}

//----------------------------------------------------------------------------------------------------------------------
float SMCArm::getFitness()
{
  return m_fitness / 2.0f;
}

//----------------------------------------------------------------------------------------------------------------------
void SMCArm::nextTrial(int trial)
{
  const int numTrials = SETTINGS->getChild("Config/GA/Trials").getAttributeValue<int>("Num",1);

  // Systematically vary distance of line
  if(numTrials > 1)
  {
    Positionable* obj = getEnvironment()->getObjects()[0];
    obj->setPosition(ci::Vec2f(0.4, 0) + trial * ci::Vec2f(0.2/numTrials, 0));
  }
  
#if 0
  // Systematically vary angle of line
  if(numTrials > 1)
  {
    const std::vector<Positionable*>& objects = getEnvironment()->getObjects();
    const float angularRange = 60.0f;
    int t = trial % numTrials;
    float angleDeg = 90.0f - angularRange/2.0f + (t * angularRange / (numTrials-1));
    objects[0]->setAngle(degreesToRadians(angleDeg));
  }
#endif
}

//----------------------------------------------------------------------------------------------------------------------
bool SMCArm::hasFinished()
{
  return m_time >= m_trialDuration;
};

//----------------------------------------------------------------------------------------------------------------------
void SMCArm::endOfEvaluation(float fit)
{
  nextTrial(0);
  
  if ((fit > 0.4f) && (m_fitnessStage == 0))
    m_fitnessStage = 1;
  
  //if ((fit > 0.5f) && (m_fitnessStage == 1))
  //  m_fitnessStage = 2;
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