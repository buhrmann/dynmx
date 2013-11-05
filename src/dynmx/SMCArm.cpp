//
//  SMCArm.cpp
//  dynmx
//
//  Created by Thomas Buhrmann on 8/9/13.
//
//

#include "SMCArm.h"
#include "MathUtils.h"
#include "Simulation.h"

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
    
    m_sensorDropTail = xml.getChild("SensorFilter").getAttributeValue<double>("dropTail", 0.0);
    m_sensorDropInterval = xml.getChild("SensorFilter").getAttributeValue<double>("dropInterval", 0.0);
    m_sensorDropStaged = xml.getChild("SensorFilter").getAttributeValue<bool>("staged", true);
    m_noise = xml.getChild("SensorFilter").getAttributeValue<double>("noise", 0.0);
    
    m_trialDuration = xml.getChild("TrialDuration").getValue<double>(14.0);
    m_fitnessEvalDelay = xml.getChild("FitnessEvalDelay").getValue<double>(2.0);
    m_probePhaseDuration = xml.getChild("ProbePhaseDuration").getValue<double>(4.0);
    m_evalPhaseDuration = m_trialDuration - m_fitnessEvalDelay - m_probePhaseDuration;
    m_fitnessStageThreshold = xml.getChild("FitnessStageThreshold").getValue<float>(0.5);
    m_minimiseConnections = xml.getChild("MinimiseConnections").getValue<bool>(false);
    if(m_minimiseConnections)
    {
      m_maxTotalWeight = xml.getChild("MinimiseConnections").getAttributeValue<float>("maxWeight");
      m_minFitness = xml.getChild("MinimiseConnections").getAttributeValue<float>("minFit");
    }
    else
    {
      m_maxTotalWeight = 0.0f;
      m_minFitness = 0.0f;
    }
    
    // Load environment objects
    m_environment.fromXml(xml.getChild("Environment"));
    
    m_posVar = getEnvironment()->getObjects()[0]->getPositionVar().x > 0;
  }
  
  m_arm.init();
  m_arm.m_pd[0].m_P = 10.0f;
  m_arm.m_pd[0].m_D = 1.0f;
  
  m_arm.m_pd[1].m_P = 20.0f;
  m_arm.m_pd[1].m_D = 2.0f;
  
  // All limits are symmetric, equal between joints, and stored in rad
  m_maxJointAngle = abs(m_arm.getJointLimitUpper(JT_elbow));
  
  // Which stage do we start with?
  m_fitnessMaxStages = 8;
  if (settings->getChild("Config/GA/Eval").getAttributeValue<bool>("Run", false))
  {
    // We're testing
    m_fitnessStage = settings->getChild("Config/GA/Evolvable/FitnessStageThreshold").getAttributeValue<float>("initialStage", m_fitnessMaxStages);
    m_numTrials = SETTINGS->getChild("Config/GA/Eval/NumTrials").getAttributeValue<int>("Value", 1);
  }
  else
  {
    if (settings->getChild("Config/GA/Incremental").getValue<bool>(false))
    {
      m_fitnessStage = settings->getChild("Config/GA/Evolvable/FitnessStageThreshold").getAttributeValue<float>("initialStage", m_fitnessMaxStages);
      m_numTrials = SETTINGS->getChild("Config/GA/Trials").getAttributeValue<int>("Num", 1);
    }
    else
    {
      m_fitnessStage = 1;
      m_numTrials = 1;
    }
  }
  
  nextTrial(0);
  
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
  m_vel.set(0,0);
  m_handTraj.clear();
  
  m_handSpeed = 0.0;
  m_handDist = 0.0;
  
  m_phase = 0;
}


//----------------------------------------------------------------------------------------------------------------------
void SMCArm::update(float dt)
{
  
#define CONTROL_JOINTS 1
  
  // Sense environment
  updateSensor(dt);
  m_sensedValue = m_distanceSensor.getActivation();
  
  // Manipulate sensor input
  float sensor = m_sensedValue;
  
  // Final blackout: starts earlier with increasing fitness stage
  float fitnessStageProp = m_sensorDropStaged ? ((float)m_fitnessStage) / m_fitnessMaxStages : 1.0f;
  float sensorDropTail = m_sensorDropTail * fitnessStageProp;
  bool blackout = m_time > (m_trialDuration - sensorDropTail);

  // Interval blackouts: duration increases with fitness stage
  float sensorDropInterval = m_sensorDropInterval * fitnessStageProp;
  if(sensorDropInterval > 0)
  {
    const int numDrops = 4;
    float drops [numDrops] = {8.0f, 9.0f, 10.0f, 11.0f};
    for (int i = 0; i < numDrops; ++i)
    {
      if ((m_time >= drops[i]) && (m_time < (drops[i] + sensorDropInterval)))
      {
        blackout = true;
      }
    }
  }
  
  if(blackout)
    sensor = 0;
  
  // Add noise
  if(m_noise > 0.0)
    sensor = sensor + UniformRandom(-m_noise, m_noise);

  
  // Update CTRNN
  m_ctrnn->setExternalInput(0, sensor);
  
#if CONTROL_JOINTS
  m_ctrnn->setExternalInput(1, m_arm.getJointAngle(JT_elbow) / m_maxJointAngle);
  m_ctrnn->setExternalInput(2, m_arm.getJointAngle(JT_shoulder) / m_maxJointAngle);
#else
  m_ctrnn->setExternalInput(1, m_vel.x);
  m_ctrnn->setExternalInput(2, m_vel.y);
#endif
  
  // Add blackout signal
  if (m_topology.getNumInputs() > 3)
  {
    if (blackout)
    {
      m_ctrnn->setExternalInput(3, 1.0f);
    }
    else
    {
      m_ctrnn->setExternalInput(3, 0.0f);
    }
    //m_ctrnn->setExternalInput(3, m_distanceSensor.getDerivative() / 100.0f);
  }
  
  m_ctrnn->updateDynamic(dt);
  
  // Move
  const int mn1Id = m_topology.getSize() - 1;
  const int mn2Id = m_topology.getSize() - 2;
  
  m_prevPos = m_arm.getEffectorPos();
  
#if CONTROL_JOINTS
  float desAng1 = -m_maxJointAngle + (2 * m_maxJointAngle * m_ctrnn->getOutput(mn1Id));
  float desAng2 = -m_maxJointAngle + (2 * m_maxJointAngle * m_ctrnn->getOutput(mn2Id));
  m_arm.updatePD(dt, desAng1, desAng2);
#else
  const float maxVel = 0.2f;
  const ci::Vec2f desVel(-1.0f + 2.0f * m_ctrnn->getOutput(mn1Id), -1.0f + 2.0f * m_ctrnn->getOutput(mn2Id));
  const ci::Vec2f desPos = m_arm.getEffectorPos() + maxVel * desVel;
  m_arm.updatePosition(dt, desPos.x, desPos.y);
#endif
  
  m_vel = (m_arm.getEffectorPos() - m_prevPos) / dt;
  
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
  Line* line = (Line*)(getEnvironment()->getObjects()[0]);
  const ci::Vec2f& s = line->getStart();
  const ci::Vec2f& e = line->getEnd();
  ci::Vec2f lineDir = (e - s).normalized();

  // Ensure a minimum averge hand speed
  const float minVel = 0.08f;
  ci::Vec2f handPos = m_arm.getEffectorPos();
  ci::Vec2f handVel = (handPos - m_prevPos) / dt;
  m_handSpeed = handVel.dot(lineDir);
  m_projVel = lineDir * m_handSpeed;
  m_fitHandVel = 1.0f - ((minVel - clamp(abs(m_handSpeed), 0.0f, minVel)) / minVel);
  
  // Punish distance from line
  const float maxDist = 0.1f;
  float handProj = (handPos - s).dot(lineDir);
  handProj = clamp(handProj, 0.0f, line->getLength());
  m_projPos = s + lineDir * handProj;
  m_fitHandDist = 1.0f - sqr(clamp(m_projPos.distance(handPos), 0.0f, maxDist) / maxDist);
  
  // Punish angular distance between line and hand velocity
  // Can only calculate angle if hand has a velocity > 0
  if (abs(m_handSpeed) > 0.0001f)
  {
    float dot = lineDir.dot(handVel.normalized());
    float angle = acos(clamp(dot, -1.f, 1.f)) / PI; // in [0,1]
    m_fitAngleDist = angle < 0.1f ? 1.0f - angle : 0.0f;
  }
  else
  {
    m_fitAngleDist = 0.0f;
  }
  
  // Determine phase of trial
  float secPhase = m_fitnessEvalDelay + m_probePhaseDuration;
  m_phase = 0;
  if(m_time > m_fitnessEvalDelay)
  {
    m_phase = m_time < secPhase ? 1 : 2;
  }
    
  m_instFit = 0;
  
  // Fitness depends on evaluation phase
  if(m_phase == 1)
  {
    // Encourage sensor use in the beginning
    float maxDist = 1.0f;
    m_instFit = 1.0f - sqr(clamp(m_projPos.distance(handPos), 0.0f, maxDist) / maxDist);
    m_instFit /= 2.0f * m_probePhaseDuration;
  }
  else if(m_phase == 2)
  {
    m_instFit += ((m_sensedValue) * m_fitHandVel * m_fitAngleDist) / (2.0f * m_evalPhaseDuration);
  }

  m_fitness += dt * m_instFit;
  
}

//----------------------------------------------------------------------------------------------------------------------
float SMCArm::getFitness()
{
  
  if(m_minimiseConnections)
  {
    float annWeightProp = m_ctrnn->getWeightSum() / m_maxTotalWeight;
    float weightFit = 1 - annWeightProp;
    
    //std::cout << "Fit without con. min.: " << fitness << std::endl;

    if(m_fitness >= m_minFitness)
      return weightFit;
    else
      return 0.0f;
  }
  
  return m_fitness;
}

//----------------------------------------------------------------------------------------------------------------------
void SMCArm::nextTrial(int trial)
{
  Positionable* obj = getEnvironment()->getObjects()[0];
  
  // Systematically vary angle of line
  if(m_numTrials > 1)
  {
    // In rad...
    const bool posVar = obj->getPositionVar().x > 0;
    int numAngleTrials = posVar ? m_numTrials / 2 : m_numTrials;
    const float maxRange = obj->getAngleVar();
    const float angularRange = m_fitnessStage * (maxRange / m_fitnessMaxStages);
    int t = trial % numAngleTrials;
    // PI/2 (90 deg) makes the line orthogonal to the arm when extended, so that's the middle of the range
    float angle = PI_OVER_TWO - angularRange/2.0f + (t * angularRange / (numAngleTrials - 1));
    obj->setAngle(angle);
    
#if 0
    std::cout << "Angle: " << radiansToDegrees(angle) << std::endl;
#endif
    
    // Systematicall vary distance of line
    if(posVar)
    {
      const int numPosTrials = 2;
      const float maxDistRange = obj->getPositionVar().x;
      int t = trial < m_numTrials/2 ? 0 : 1;
      float distRange = m_fitnessStage * (maxDistRange / m_fitnessMaxStages);
      float dist = 0.5 - distRange/2.0f + (t * distRange / (numPosTrials - 1));
      obj->setPosition(ci::Vec2f(dist, 0));
#if 0
      std::cout << "Distance: " << dist << std::endl;
#endif
    }
    
  }
}

//----------------------------------------------------------------------------------------------------------------------
bool SMCArm::hasFinished()
{
  return m_time >= m_trialDuration;
};

//----------------------------------------------------------------------------------------------------------------------
void SMCArm::endOfEvaluation(float fit)
{
  //nextTrial(0);
  
  if (fit > m_fitnessStageThreshold && m_fitnessStage < m_fitnessMaxStages)
  {
    m_fitnessStage++;
    
    if(m_fitnessStage == 2)
    {
      // 3 different angles (and 2 positions)
      m_numTrials = 3 * (m_posVar ? 2 : 1);
      ((GARunner*)Simulation::getInstance()->getModel())->setNumTrials(m_numTrials);
    }
    else if(m_fitnessStage == 5)
    {
      // 5 different angles (and 2 positions)
      m_numTrials = 5 * (m_posVar ? 2 : 1);
      ((GARunner*)Simulation::getInstance()->getModel())->setNumTrials(m_numTrials);
    }
    
    // Bad form, but what the heck: need to inform GA to reset previous evaluations
    ((GARunner*)Simulation::getInstance()->getModel())->fitnessFunctionChanged();
    
    std::cout <<  Globals::Inst()->getDataDirName() << " | Next fitness stage: " << m_fitnessStage;
    std::cout << " | numTrials: " << m_numTrials << std::endl;
  }
  
  m_fitness = fit;
  
  // Safe here, since we know we're being called by GARunner
#if 0
  std::cout << "Arm trials n: " << m_numTrials << " GA: " << ((GARunner*)Simulation::getInstance()->getModel())->getNumTrials() << std::endl;
  // m_numTrials = ((GARunner*)Simulation::getInstance()->getModel())->getNumTrials();
#endif
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
  
  // Hack: check if new encode method roundtrips
#if 0
  m_ctrnn
  
  const int numGenes = m_topology.getNumParameters();
  double new_genome [numGenes];
  m_topology.encode(*m_ctrnn, &new_genome[0], m_netLimits);
  bool equal = true;
  double eps = 0.000001;
  for(int i = 0; i < numGenes; i++)
  {
    if(fabs(new_genome[i] - genome[i]) > eps)
    {
      equal = false;
      std::cout << "Roundtrip o: " << genome[i] << " n: " << new_genome[i] << std::endl;
      break;
    }
  }
  std::cout << "Roundtrips: " << equal << std::endl;
#endif
}

//----------------------------------------------------------------------------------------------------------------------
void SMCArm::toXml(ci::XmlTree& xml)
{
  ci::XmlTree evolvable ("SMCArm", "");
  evolvable.setAttribute("Fitness", getFitness());
  evolvable.setAttribute("FitnessStage", m_fitnessStage);
  
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
  
  Line* line = ((Line*)getEnvironment()->getObjects()[0]);
  const ci::Vec2f& s = line->getStart();
  const ci::Vec2f& e = line->getEnd();
  recorder.push_back("lineX1", s.x);
  recorder.push_back("lineY1", s.y);
  recorder.push_back("lineX2", e.x);
  recorder.push_back("lineY2", e.y);
}
  
} // namespace