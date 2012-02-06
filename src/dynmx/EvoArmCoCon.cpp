/*
 *  EvoArmCoCon.cpp
 *  cinder_dynmx
 *
 *  Created by Thomas Buhrmann on 1/18/12.
 *  Copyright 2012 __MyCompanyName__. All rights reserved.
 *
 */

#include "EvoArmCoCon.h"

namespace dmx
{

//----------------------------------------------------------------------------------------------------------------------
EvoArmCoCon::EvoArmCoCon() : m_arm(0), m_time(0) 
{ 
  m_arm = new dmx::ArmReflex(); 
  init();
};


//----------------------------------------------------------------------------------------------------------------------
int EvoArmCoCon::getNumGenes() 
{ 
  int numGenes = 0;
  
  // Duration of commanded ramp as proportion of desired movement time
  numGenes += 1;
   
  // Delay between start of commanded movement and evaluation of minimum-jerk trajectory    
  if(m_evolveMinJerkDelay)
    numGenes += 1;  
  
  // Muscle params symmetric antagonists:
  // 2 X joint radii, 2 x friction, 5 muscle params (L0, Fmax, vmax, origin, insertion)
  numGenes += 2 + 2 + (2 * 5);
  
  // Muscle params symmetric antagonists:
  // 2 X 4 hill parameters (hSh, hLn, hMax)
  if(m_evolveHillParams)
    numGenes += 2 * 3;  
  
  // Reflex params
  // Spindle gains (p,v,d) and exponent
  if(m_evolveUniformSpindles)
    numGenes += 1 * 4; // All spindles behave the same way
  else
    numGenes += 2 * 4; // Spindles differ between elbow and shoulder
  
  // Input to motor neurons
  // Weights from spindle (stretch reflex)
  numGenes += 2 * 1;  
  
  // Min cocontraction throughout rest and movement periods
  if(m_enableCoconIncrease)
    numGenes += 1;
  
  // Parameters for IaIn neurons
  // Bias, tau, weights from ia, rn, sp, A
  // Plus input to MN
  if(m_evolveIAIN)
    numGenes += 2 * 7;
  
  // Renshaw neurons
  // Bias, tau, weights from mn, rn
  // Plus input to MN  
  if(m_evolveRenshaw)
    numGenes += 2 * 5;
  
  // IbIn neurons
  // Bias, tau, weights from golgi, ib
  // Plus input to MN  
  if(m_evolveIBIN)
    numGenes += 2 * 5;  
  
  // Parameters of ifv neurons
  if(m_evolveIFV)  
    numGenes += 2 * 2;
  
  // Open-loop activation (numMuscles x 3 target poses)
  if(m_evolveOpenLoop)
    numGenes += 2 * 3;  
  
  return numGenes;
};

//----------------------------------------------------------------------------------------------------------------------
void EvoArmCoCon::decodeGenome(const double* genome)
{
  // Joint parameters
  double elbRad = 0.02 + 0.03 * genome[0];
  double shdRad = 0.02 + 0.03 * genome[1];
  m_arm->setJointRadii(elbRad, shdRad);  
  
  m_arm->setFriction(0.01 + 0.19 * genome[2], 0.01 + 0.19 * genome[3]);
  
  int start = 4;
  for(int i = 0; i < m_arm->getNumMuscles() / 2; ++i)
  {
    // Muscle parameters
    double origin = 0.1 + 0.15 * genome[start + 0];
    double insertion =  0.075 + 0.125 * genome[start + 1];
    double maxIsoForce = 100.0 + 2400.0 * genome[start + 2]; 
    double optimalLength = 0.5 + 0.5 * genome[start + 3];          
    double maxVel = 5.0 + 10.0 * genome[start + 4];                  
    m_arm->setMuscleParams(i*2, origin, insertion, maxIsoForce, optimalLength, maxVel); 
    m_arm->setMuscleParams(i*2 + 1, origin, insertion, maxIsoForce, optimalLength, maxVel);     
    
    // Muscles need to be reinitialized so that correct values can be precalculated from new parameters
    if(m_arm->getMuscle(i)->isMonoArticulate())
    {
      ((MuscleMonoWrap*)m_arm->getMuscle(i*2))->init();      
      ((MuscleMonoWrap*)m_arm->getMuscle(i*2 + 1))->init();
    }
    else 
    {
      ((MuscleBiWrap*)m_arm->getMuscle(i*2))->init();        
      ((MuscleBiWrap*)m_arm->getMuscle(i*2 + 1))->init();        
    }
    
    // Need to set length again after initialisation of muscle to be able to use min, max lengths !
    double lOpt0 = m_arm->getMuscle(i*2)->unitLengthToLength(optimalLength);
    double lOpt1 = m_arm->getMuscle(i*2+1)->unitLengthToLength(optimalLength);
    m_arm->getMuscle(i*2)->setParameters(maxIsoForce, lOpt0, maxVel);
    m_arm->getMuscle(i*2 + 1)->setParameters(maxIsoForce, lOpt1, maxVel);
    
    start += 5;
    
    if(m_evolveHillParams)
    {
      double hSh = 0.1 + 0.4 * genome[start + 0];
      double hLn = 0.1 + 0.4 * genome[start + 1];
      double hMax = 1.1 + 0.7 * genome[start + 2];
      double hSlp = 2.0;
      m_arm->getMuscle(i*2)->setHillParameters(hSh, hLn, hMax, hSlp);
      m_arm->getMuscle(i*2 + 1)->setHillParameters(hSh, hLn, hMax, hSlp);
      start += 3;
    }

  }
  
  // Spindle parameters
  const float maxSpP = 20.0;
  const float maxSpV = 0.0;
  const float maxSpD = 5.0;
  const float minSpExp = 0.5;
  m_arm->getReflex(0)->setSpindleParameters(genome[start + 0]*maxSpP, genome[start + 0]*maxSpP, 
                                            genome[start + 1]*maxSpV, genome[start + 1]*maxSpV, 
                                            genome[start + 2]*maxSpD, genome[start + 2]*maxSpD, 
                                            minSpExp + (genome[start + 3] * (1 - minSpExp)), 
                                            minSpExp + (genome[start + 3] * (1 - minSpExp)));  
  if(m_evolveUniformSpindles)
  {
    m_arm->getReflex(1)->setSpindleParameters(genome[start + 0]*maxSpP, genome[start + 0]*maxSpP, 
                                              genome[start + 1]*maxSpV, genome[start + 1]*maxSpV, 
                                              genome[start + 2]*maxSpD, genome[start + 2]*maxSpD, 
                                              minSpExp + (genome[start + 3] * (1 - minSpExp)), 
                                              minSpExp + (genome[start + 3] * (1 - minSpExp)));  
    start += 4;
  }
  else
  {
    m_arm->getReflex(1)->setSpindleParameters(genome[start + 4]*maxSpP, genome[start + 4]*maxSpP, 
                                              genome[start + 5]*maxSpV, genome[start + 5]*maxSpV, 
                                              genome[start + 6]*maxSpD, genome[start + 6]*maxSpD, 
                                              minSpExp + (genome[start + 7] * (1 - minSpExp)), 
                                              minSpExp + (genome[start + 7] * (1 - minSpExp)));  
    start += 8;  
  }
  
  
  // Alpha MN params: input from spindles
  float maxW = 1.0;
  m_arm->getReflex(0)->setMotoNeuronParameters(genome[start+0] * maxW, genome[start+0] * maxW); 
  m_arm->getReflex(1)->setMotoNeuronParameters(genome[start+1] * maxW, genome[start+1] * maxW);
  start += 2;
  
  maxW = 10.0;
  float maxB = 10.0;
  float maxT = 100.0;  
  
  // IaIn parameters  
  if(m_evolveIAIN)
  {
    m_arm->getReflex(0)->setIaInParameters(genome[start+0] * maxW, genome[start+0] * maxW,        // spindle->ia
                                           genome[start+1] * maxW, genome[start+1] * maxW,        // ia->ia
                                           genome[start+2] * maxW, genome[start+2] * maxW,        // desired contr->ia
                                           genome[start+3] * maxW, genome[start+3] * maxW,        // rn->ia
                                           genome[start+4] * maxW, genome[start+4] * maxW,        // ia->mn
                                           10 + genome[start+5] * maxT, 10 + genome[start+5] * maxT,        // time constants
                                           -maxB + (genome[start+6] * 2 * maxB), -maxB + (genome[start+6] * 2 * maxB));       // biases
    
    m_arm->getReflex(1)->setIaInParameters(genome[start+7] * maxW, genome[start+7] * maxW,
                                           genome[start+8] * maxW, genome[start+8] * maxW,
                                           genome[start+9] * maxW, genome[start+9] * maxW,
                                           genome[start+10] * maxW, genome[start+10] * maxW,
                                           genome[start+11] * maxW, genome[start+11] * maxW,
                                           10 + genome[start+12] * maxT, 10 + genome[start+12] * maxT,
                                           -maxB + (genome[start+13] * 2 * maxB), -maxB + (genome[start+13] * 2 * maxB));  
    start += 14;
  }
  
  
  // Renshaw param
  if(m_evolveRenshaw)
  {
    m_arm->getReflex(0)->setRenshawParameters(genome[start+0], genome[start+0], 
                                              genome[start+1], genome[start+1], 
                                              genome[start+2], genome[start+2], 
                                              10 + genome[start+3] * maxT, 10 + genome[start+3] * maxT,
                                              -maxB + (genome[start+4] * 2 * maxB), -maxB + (genome[start+4] * 2 * maxB));  
    
    m_arm->getReflex(1)->setRenshawParameters(genome[start+5], genome[start+5], 
                                              genome[start+6], genome[start+6], 
                                              genome[start+7], genome[start+7], 
                                              10 + genome[start+8] * maxT, 10 + genome[start+8] * maxT,
                                              -maxB + (genome[start+9] * 2 * maxB), -maxB + (genome[start+9] * 2 * maxB));  
    start += 10;  
  }
  
  // IbIn params
  if(m_evolveIBIN)
  {
    m_arm->getReflex(0)->setIbInParameters(genome[start+0], genome[start+0], 
                                           genome[start+1], genome[start+1], 
                                           genome[start+2], genome[start+2], 
                                           10 + genome[start+3] * maxT, 10 + genome[start+3] * maxT,
                                           -maxB + (genome[start+4] * 2 * maxB), -maxB + (genome[start+4] * 2 * maxB));  
    
    m_arm->getReflex(1)->setIbInParameters(genome[start+5], genome[start+5], 
                                           genome[start+6], genome[start+6], 
                                           genome[start+7], genome[start+7], 
                                           10 + genome[start+8] * maxT, 10 + genome[start+8] * maxT,
                                           -maxB + (genome[start+9] * 2 * maxB), -maxB + (genome[start+9] * 2 * maxB)); 
    start += 10;    
  }
  
  // IFV gains
  if(m_evolveIFV)
  {
    const float maxIC = 10.0f;
    const float maxICb = 1.0f;
    m_arm->getReflex(0)->setInertiaCompensationParameters(genome[start+0] * maxIC, genome[start+0] * maxIC, 
                                                          genome[start+1] * maxICb, genome[start+1] * maxICb);
    
    m_arm->getReflex(1)->setInertiaCompensationParameters(genome[start+2] * maxIC, genome[start+2] * maxIC, 
                                                          genome[start+3] * maxICb, genome[start+3] * maxICb);  
    start +=4;
  }
  
  // Open loop
  if(m_evolveOpenLoop)
  {
    for (int i = 0; i < 6; ++i)
    {
      m_openLoop[i] = m_maxOpenLoop * genome[start + i];
    }
    start += 6;
  }
  
  // Trajectory related values
  if(m_evolveMinJerkDelay)
  {
    m_minJerkTrajDelay = 0.1f * genome[start];
    start += 1;
  }
  
  // Duration of commanded ramp as proportion of desired movement time
  m_rampDurationFactor = 0.4 + 0.6 * genome[start];
  start += 1;
  
  if(m_enableCoconIncrease)
  {
    m_minCocontraction = 0.05 + 0.15 * genome[start];
    start += 1;
  }
  
  createTrajectories();
};


  // Define distance measure for fitness function
//----------------------------------------------------------------------------------------------------------------------
struct VecDist
{
  double operator() (ci::Vec2f t1, ci::Vec2f t2) { return (t1-t2).lengthSquared(); };
};  

//----------------------------------------------------------------------------------------------------------------------
float EvoArmCoCon::getFitness() 
{ 
  const double dt = 1.0 / (double)SETTINGS->getChild("Config/Globals/FrameRate").getAttributeValue<int>("Value");
  
#define FITNESS_USE_CROSS_CORRELATION 1
#if !FITNESS_USE_CROSS_CORRELATION  
  float distFit = 1.0 - sqrt(m_fitness / (m_time / dt));
#else  
  int minDelay = -100;
  int maxDelay = 100;
  std::vector<double> cc = crossCorrelation(m_desiredPositions, m_actualPositions, VecDist(), minDelay, maxDelay);
  std::vector<double>::iterator optElem = min_element(cc.begin(), cc.end());
  double optVal = *optElem / (m_time / dt);
  //int optAt = minDelay + std::distance(cc.begin(), optElem);
  float distFit = 1.0 - sqrt(optVal);
#endif
  
  
  if(m_enableCoconIncrease)
  {
    float coconFit = 0;
    int N = m_coconIncrements.size();
    for(int i = 0; i < N; i++)
    {
      coconFit += std::max(m_coconIncrements[i], 0.0f);
    }
    coconFit /= N;
    distFit *= coconFit;
  }
  
  return distFit;
};

//----------------------------------------------------------------------------------------------------------------------
void EvoArmCoCon::init() 
{ 
  assert(m_arm); 
  m_arm->init(); 
  
  // Read settings
  if (SETTINGS->hasChild("Config/GA/Evolvable/EvoFlags")) 
  {
    ci::XmlTree& xml = SETTINGS->getChild("Config/GA/Evolvable/EvoFlags");    
    m_evolveIAIN = xml.getChild("iain").getValue<bool>();
    m_evolveRenshaw = xml.getChild("renshaw").getValue<bool>();
    m_evolveIBIN = xml.getChild("ibin").getValue<bool>();
    m_evolveIFV = xml.getChild("ifv").getValue<bool>();
    m_evolveSFV = xml.getChild("sfv").getValue<bool>();
    m_evolveOpenLoop = xml.getChild("openLoop").getValue<bool>();
    m_maxOpenLoop = xml.getChild("maxOpenLoop").getValue<float>();    
    m_evolveUniformSpindles = xml.getChild("uniformSpindles").getValue<bool>();
    m_evolveHillParams = xml.getChild("hillParams").getValue<bool>();
    m_evolveMinJerkDelay = xml.getChild("minJerkDelay").getValue<bool>();
    
    xml = SETTINGS->getChild("Config/GA/Evolvable/");
    m_enableCoconIncrease = xml.getChild("enableCoconInc").getValue<bool>();
    m_minCocontraction = xml.getChild("minCocon").getValue<float>();
    m_maxCocontraction = xml.getChild("maxCocon").getValue<float>();
    
    m_useMuscleCoords = xml.getChild("useMuscleCoords").getValue<bool>();
    
    // Read trajectory from settings file    
    // Requires arm to be setup to do FK, but that's already done in init() above, so should be fine.
    if(SETTINGS->hasChild("Config/GA/Evolvable/DesiredTrajectory"))
    {
      ci::Vec2f pos, tmp;
      float approachDuration;
      float maintainDuration;
      xml = SETTINGS->getChild("Config/GA/Evolvable/DesiredTrajectory/");
      bool convertToRadians = xml.getAttributeValue<bool>("InDegrees");
      for (ci::XmlTree::ConstIter viaPoint = xml.begin(); viaPoint != xml.end(); ++viaPoint)
      {
        float elbAngle = viaPoint->getChild("Position").getAttributeValue<float>("x");
        float shdAngle = viaPoint->getChild("Position").getAttributeValue<float>("y");
        if (convertToRadians)
        {
          elbAngle = degreesToRadians(elbAngle);
          shdAngle = degreesToRadians(shdAngle);
        }
        m_arm->forwardKinematics(elbAngle, shdAngle, tmp, pos);
        approachDuration = viaPoint->getChild("ApproachDuration").getValue<float>();
        maintainDuration = viaPoint->getChild("MaintainDuration").getValue<float>();

        // First one is special
        if(viaPoint == xml.begin())
        {
          m_desiredTrajectory.add(pos, maintainDuration);
        }
        else 
        {
          ci::Vec2f prevPos = m_desiredTrajectory.getTargets().back().position;
          m_desiredTrajectory.add(prevPos, approachDuration);
          m_desiredTrajectory.add(pos, maintainDuration);
        }
      } // loop viaPoints
    } // read trajectory
    m_desiredTrajectory.setBlend(Trajectory<ci::Vec2f>::kTr_BlendMinJerk);
  }
  else 
  {
    m_evolveIAIN = false;
    m_evolveRenshaw = false;
    m_evolveIBIN = false;
    m_evolveIFV = false;
    m_evolveSFV = false;
    m_evolveOpenLoop = false;
    m_maxOpenLoop = 0.25;
    m_evolveUniformSpindles = true;
    m_evolveHillParams = false;
    m_evolveMinJerkDelay = false;
    
    m_enableCoconIncrease = false;
    m_minCocontraction = 0.05;
    m_maxCocontraction = 0.25;
    
    m_useMuscleCoords = false;
  }
  
  reset();
}

//----------------------------------------------------------------------------------------------------------------------
void EvoArmCoCon::createTrajectories() 
{   
  // First clear existing trajectories, as below we will append
  m_commandedTrajectory.clear();
  m_targetJointAngles.clear();
  m_targetElbLengths.clear();
  m_targetShdLengths.clear();
  
  // Translate desired into commanded trajectory, based on ramp duration factor
  m_commandedTrajectory.setBlend(Trajectory<ci::Vec2f>::kTr_BlendLinear);
  m_commandedTrajectory.add(m_desiredTrajectory[0]);
  for(int i = 1; i < m_desiredTrajectory.size()-1; i+=2)
  {
    // approach
    ci::Vec2f desPos = m_desiredTrajectory[i].position;
    float desApproachDuration = m_desiredTrajectory[i].getDuration();
    float comApproachDuration = m_rampDurationFactor * desApproachDuration;
    m_commandedTrajectory.add(desPos, comApproachDuration);
    // maintain
    desPos = m_desiredTrajectory[i+1].position;
    float maintainDuration = m_desiredTrajectory[i+1].getDuration() + (desApproachDuration - comApproachDuration);
    m_commandedTrajectory.add(desPos, maintainDuration);
  }
   
  // Transform cartesian effector position into joint angle targets
  for(int i = 0; i < m_commandedTrajectory.size(); i++)
  {
    int elbDir = i < 8 ? 1 : -1;
    double desiredAngles[2];
    m_arm->inverseKinematics(m_commandedTrajectory[i].position, elbDir, desiredAngles[JT_elbow], desiredAngles[JT_shoulder]);  
    m_targetJointAngles.add(m_commandedTrajectory[i]);
    m_targetJointAngles[i].position = Pos(desiredAngles[JT_elbow], desiredAngles[JT_shoulder]);
  }
  m_targetJointAngles.setBlend(Trajectory<ci::Vec2f>::kTr_BlendLinear);
  
  
  // Transform joint angle targets into muscle length targets
  for(int i = 0; i < m_commandedTrajectory.size(); i++)
  {
    Target<Pos> jointTarget = m_targetJointAngles[i];
    
    // Elbow muscles
    double desLength0 = m_arm->getReflex(0)->getMuscle(0)->getLengthFromJointAngles(jointTarget.position.x, jointTarget.position.y);
    double desLength1 = m_arm->getReflex(0)->getMuscle(1)->getLengthFromJointAngles(jointTarget.position.x, jointTarget.position.y);
    m_targetElbLengths.add(jointTarget);
    m_targetElbLengths[i].position = Pos(desLength0, desLength1);
    
    // Shoulder muscles
    desLength0 = m_arm->getReflex(1)->getMuscle(0)->getLengthFromJointAngles(jointTarget.position.x, jointTarget.position.y);
    desLength1 = m_arm->getReflex(1)->getMuscle(1)->getLengthFromJointAngles(jointTarget.position.x, jointTarget.position.y);
    m_targetShdLengths.add(jointTarget);
    m_targetShdLengths[i].position = Pos(desLength0, desLength1);
  }
  m_targetElbLengths.setBlend(Trajectory<ci::Vec2f>::kTr_BlendLinear);  
  m_targetShdLengths.setBlend(Trajectory<ci::Vec2f>::kTr_BlendLinear);    
}

//----------------------------------------------------------------------------------------------------------------------
void EvoArmCoCon::reset()
{
  m_fitness = 0.0f;
  m_time = 0.0f;
  
  m_desiredPositions.clear();
  m_actualPositions.clear();
  m_coconIncrements.clear();
  
  m_coconStarted = false;
  m_arm->reset();
}

//----------------------------------------------------------------------------------------------------------------------
void EvoArmCoCon::update(float dt)
{ 
  // Blend linearly not in target position, as that would lead to asymmetric profiles in joint and muscle space,
  // but blend linearly in joint angle space
  // Get position first
  Target<Pos> desired = m_desiredTrajectory.atTime(m_time);
  Target<Pos> command = m_commandedTrajectory.atTime(m_time);
  updateCurrentCommand(command, desired);
  
  // Incremental increase in cocontraction at target position
  Target<Pos> coconTarget = desired;
  if(m_enableCoconIncrease)
  {
    m_cocontraction = m_minCocontraction;
    
    //if (coconTarget.id == 2 || coconTarget.id == 4 || coconTarget.id == 6)
    if (coconTarget.id % 2 == 0)
    {     
      // Determine how far proportionally we're through with this target
      m_cocontraction += m_maxCocontraction * ((m_time - coconTarget.start) / (coconTarget.stop - coconTarget.start));
    } 
    
    // Apply
    m_arm->getReflex(0)->setCocontraction(m_cocontraction, m_cocontraction);
    m_arm->getReflex(1)->setCocontraction(m_cocontraction, m_cocontraction);    
  }
  
  // Static activation added for each target
#if 0
  if(m_evolveOpenLoop)
  {
    if (command.id == 1 || command.id == 2)
    {
      m_arm->getReflex(0)->setCocontraction(m_openLoop[2], m_openLoop[3]);
      m_arm->getReflex(1)->setCocontraction(m_openLoop[4], m_openLoop[5]);        
    }
    else if (command.id == 5 || command.id == 6)
    {
      m_arm->getReflex(0)->setCocontraction(m_openLoop[3], m_openLoop[2]); // Mirror image of first set
      m_arm->getReflex(1)->setCocontraction(m_openLoop[5], m_openLoop[4]);        
    }
    else
    {
      m_arm->getReflex(0)->setCocontraction(m_openLoop[0], m_openLoop[0]);
      m_arm->getReflex(1)->setCocontraction(m_openLoop[1], m_openLoop[1]);    
    }   
  }
#endif  
  
  if(m_evolveOpenLoop)
  {
    if (command.id == 1 || command.id == 2)
    {
      // First movement      
      m_arm->getReflex(0)->setCocontraction(m_openLoop[0], m_openLoop[1]);
      m_arm->getReflex(1)->setCocontraction(m_openLoop[2], m_openLoop[3]);
    }
    else if (command.id == 5 || command.id == 6)
    {
      // Second: elbow same, shoulder mirrored      
      m_arm->getReflex(0)->setCocontraction(m_openLoop[0], m_openLoop[1]);
      m_arm->getReflex(1)->setCocontraction(m_openLoop[3], m_openLoop[2]);
    }
    else if (command.id == 9 || command.id == 10)
    {
      // Third: elbow mirrored shoulder same
      m_arm->getReflex(0)->setCocontraction(m_openLoop[1], m_openLoop[0]);
      m_arm->getReflex(1)->setCocontraction(m_openLoop[2], m_openLoop[3]);
    }   
    else if (command.id == 13 || command.id == 14)
    {
      // Fourth: both mirrored
      m_arm->getReflex(0)->setCocontraction(m_openLoop[1], m_openLoop[0]);
      m_arm->getReflex(1)->setCocontraction(m_openLoop[3], m_openLoop[2]);
    }       
    else 
    {
      // Neutral position
      m_arm->getReflex(0)->setCocontraction(m_openLoop[4], m_openLoop[4]);
      m_arm->getReflex(1)->setCocontraction(m_openLoop[5], m_openLoop[5]);      
    }

  }  
  
  if(m_useMuscleCoords)
  {
    Pos commandedElbLengths = m_targetElbLengths.atTime(m_time).position;
    Pos commandedShdLengths = m_targetShdLengths.atTime(m_time).position;  
    m_arm->update(commandedElbLengths.x, commandedElbLengths.y, commandedShdLengths.x, commandedShdLengths.y, dt);  
    //m_arm->update(desiredAngles.x, desiredAngles.y, dt);  
  }
  else
  {
    m_arm->update(command.position, dt, command.id < 8 ? 1 : -1);    
  }
  
  
  // Measure real co-contraction increment between beginning and end of commanded increase
  // First target
  //if(coconTarget.id == 2 || coconTarget.id == 4 || coconTarget.id == 6)
  if(coconTarget.id % 2 == 0)
  {
    // At start of increase
    if(!m_coconStarted)
    {
      // Cache average activation at start of increase
      m_coconAtStart[0] = 0.5 * (m_arm->getReflex(0)->getAlphaOutput(0) + m_arm->getReflex(0)->getAlphaOutput(1));
      m_coconAtStart[1] = 0.5 * (m_arm->getReflex(1)->getAlphaOutput(0) + m_arm->getReflex(1)->getAlphaOutput(1));
      assert(m_coconAtStart[0] == m_coconAtStart[0]);
      assert(m_coconAtStart[1] == m_coconAtStart[1]);
      m_coconStarted = true;
    }
    // End of increase
    if((coconTarget.stop - m_time) <= dt)
    {
      double coconAtEnd [2];
      coconAtEnd[0] = 0.5 * (m_arm->getReflex(0)->getAlphaOutput(0) + m_arm->getReflex(0)->getAlphaOutput(1));
      coconAtEnd[1] = 0.5 * (m_arm->getReflex(1)->getAlphaOutput(0) + m_arm->getReflex(1)->getAlphaOutput(1));
      double totalInc = 0.5 * ((coconAtEnd[0] - m_coconAtStart[0]) + (coconAtEnd[1] - m_coconAtStart[1]));
      assert(totalInc == totalInc);
      
      //m_coconIncrements[coconTarget.id/2-1] = totalInc; // Normalise to [0,1]. Can be 1 at max per reflex, so 2 in total.
      m_coconIncrements.push_back(totalInc); // Normalise to [0,1]. Can be 1 at max per reflex, so 2 in total.
      m_coconStarted = false;
    }
  }
  
  //std::cout << "C: " << command.id << " D: " << desired.id << " CoCon: " << m_cocontraction << std::endl;
  
  // Fitness evaluation
  ci::Vec2f actualPosition = m_arm->getEffectorPos();
  const float distSq = (actualPosition - desired.position).lengthSquared();
  m_fitness += distSq;
  
  m_desiredPositions.push_back(desired.position);  
  m_actualPositions.push_back(actualPosition); 
  
  m_time += dt;
};

// Calculates desired and commanded state in cartesian, joint angle, and muscle length coordinates
//----------------------------------------------------------------------------------------------------------------------    
void EvoArmCoCon::updateCurrentCommand(Target<Pos>& command, Target<Pos>& desired)
{
  m_currentDesiredPos = desired.position;
  m_currentCommandPos = command.position;
  
  // Transform desired position to joint angles (IK) and muscle lengths
  double currentDesiredAngles[2];
  m_arm->inverseKinematics(desired.position, command.id < 8 ? 1.0f : -1.0f, currentDesiredAngles[0], currentDesiredAngles[1]);
  m_currentDesiredAngles.x = clamp(currentDesiredAngles[0], m_arm->getJointLimitLower(JT_elbow), m_arm->getJointLimitUpper(JT_elbow));  
  m_currentDesiredAngles.y = clamp(currentDesiredAngles[1], m_arm->getJointLimitLower(JT_shoulder), m_arm->getJointLimitUpper(JT_shoulder));    
  
  double desLength0 = m_arm->getReflex(0)->getMuscle(0)->getLengthFromJointAngles(m_currentDesiredAngles.x, m_currentDesiredAngles.y);
  double desLength1 = m_arm->getReflex(0)->getMuscle(1)->getLengthFromJointAngles(m_currentDesiredAngles.x, m_currentDesiredAngles.y);
  m_currentDesiredElbLenghts = Pos(desLength0, desLength1);
  
  desLength0 = m_arm->getReflex(1)->getMuscle(0)->getLengthFromJointAngles(m_currentDesiredAngles.x, m_currentDesiredAngles.y);
  desLength1 = m_arm->getReflex(1)->getMuscle(1)->getLengthFromJointAngles(m_currentDesiredAngles.x, m_currentDesiredAngles.y);
  m_currentDesiredShdLenghts = Pos(desLength0, desLength1);  
  
#if 0 
  // if controlling in other coordinates
  Pos targetAngles = m_targetJointAngles.atTime(m_time).position;
  Pos targetElbLengths = m_targetElbLengths.atTime(m_time).position;
  Pos targetShdLengths = m_targetShdLengths.atTime(m_time).position;
#endif  
}

//----------------------------------------------------------------------------------------------------------------------  
bool EvoArmCoCon::hasFinished()
{
  return m_time >= m_desiredTrajectory.getDuration();
}

//----------------------------------------------------------------------------------------------------------------------  
void EvoArmCoCon::finish()
{
  std::cout << "Fitness: " << getFitness() << std::endl;
}  

//----------------------------------------------------------------------------------------------------------------------    
void EvoArmCoCon::toXml(ci::XmlTree& xml)
{
  ci::XmlTree evolvable ("EvoArmCoCon", "");
  evolvable.setAttribute("Fitness", getFitness());
  
  // Add ArmMuscled xml data
  m_arm->toXml(evolvable);  
  
  xml.push_back(evolvable);
}
  
//----------------------------------------------------------------------------------------------------------------------     
void EvoArmCoCon::record(Recorder& recorder) 
{ 
  // General arm and reflex state
  m_arm->record(recorder); 
  
  // Desired state (different from commanded state in ArmReflex)
  recorder.push_back("desX", m_currentDesiredPos.x);
  recorder.push_back("desY", m_currentDesiredPos.y);
  recorder.push_back("desElbow", m_currentDesiredAngles.x);
  recorder.push_back("desShoulder", m_currentDesiredAngles.y);
  recorder.push_back("desElbowM1", m_currentDesiredElbLenghts.x);
  recorder.push_back("desElbowM2", m_currentDesiredElbLenghts.y);
  recorder.push_back("desShoulderM1", m_currentDesiredShdLenghts.x);
  recorder.push_back("desShoulderM2", m_currentDesiredShdLenghts.y);
}
  
  
} // namespace
