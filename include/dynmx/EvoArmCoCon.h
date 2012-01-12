/*
 *  EvoArmCoCon.h
 *  cinder_dynmx
 *
 *  Created by Thomas Buhrmann on 9/23/11.
 *  Copyright 2011 __MyCompanyName__. All rights reserved.
 *
 */

#ifndef _DMX_EVO_ARM_COCON_
#define _DMX_EVO_ARM_COCON_

#include "Dynmx.h"
#include "Model.h"
#include "GARunner.h"
#include "Arm.h"
#include "ArmReflex.h"
#include "MuscleMonoWrap.h"
#include "MuscleBiWrap.h"
#include "Trajectory.h"

namespace dmx
{
  
#define ENABLE_IAIN 0
#define ENABLE_RENSHAW 0
#define ENABLE_IBIN 0
#define ENABLE_IFV 0
#define ENABLE_SFV 0
#define ENABLE_OPENLOOP 0
  
#define ENABLE_COCON_INCREASE 0
#define MAX_COCONTRACTION 0.25
  
#define USE_UNIFORM_SPINDLES 1
#define USE_HILL_PARAMS 1
  
#define MAX_OPENLOOP 0.2 

#define DESIRED_SETTLING_TIME 1.0
#define DESIRED_MOVEMENT_TIME 0.2
#define EVOLVE_MIN_JERK_DELAY 0
  
#define USE_MUSCLE_COORDS 0
  
//----------------------------------------------------------------------------------------------------------------------
// Evolves a simple PD-like reflex for a single joint, two movements
//----------------------------------------------------------------------------------------------------------------------
class EvoArmCoCon : public Evolvable
{
  
public:
  
  EvoArmCoCon();
  
  // Implementation of dmx::Evolvable functions
  virtual int getNumGenes();
  virtual void decodeGenome(const double* genome);
  virtual float getFitness();
  
  // Implementation of dmx::Model functions
  virtual void init(); 
  virtual void reset();
  virtual void update(float dt);
  virtual bool hasFinished(); 
  virtual void finish();
  
  float getTime() { return m_time; };
  
  // toXML
  void toXml(ci::XmlTree& xml);
  
  
  ArmReflex* m_arm;
  
  Trajectory<ci::Vec2f> m_desiredTrajectory;
  Trajectory<ci::Vec2f> m_commandedTrajectory;
  Trajectory<ci::Vec2f> m_targetJointAngles;     // same targets but in joint angle space
  Trajectory<ci::Vec2f> m_targetElbLengths;      // same targets but in muscle length space
  Trajectory<ci::Vec2f> m_targetShdLengths;  
  
  ci::Vec2f m_currentDesiredPos;
  ci::Vec2f m_currentCommandPos;
  ci::Vec2f m_currentDesiredAngles;
  ci::Vec2f m_currentDesiredElbLenghts;
  ci::Vec2f m_currentDesiredShdLenghts;
  
  std::vector<ci::Vec2f> m_desiredPositions;
  std::vector<ci::Vec2f> m_actualPositions;
  
protected:
  
  void createTrajectories();
  void updateCurrentCommand(Target<Pos>& command, Target<Pos>& desired);
  
  float m_time;
  float m_fitness;
  double m_cocontraction;
  
#if EVOLVE_MIN_JERK_DELAY  
  float m_minJerkTrajDelay;     // Time between start of commanded ramp and min jerk trajectory used for fitness
#endif
  
  float m_rampDurationFactor;   // Scales the duration of the commanded ramp as proportion of desired movement time
  
  double m_coconIncrements[3];  // At three different positions
  double m_coconAtStart[2];     // For two reflexes
  bool m_coconStarted;
  
#if ENABLE_OPENLOOP
  double m_openLoop[6];  
#endif
};

//----------------------------------------------------------------------------------------------------------------------
// Inline Implementation
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
  
#if EVOLVE_MIN_JERK_DELAY  
  // Delay between start of commanded movement and evaluation of minimum-jerk trajectory  
  numGenes += 1;  
#endif
  
  // Muscle params symmetric antagonists:
  // 2 X joint radii, 5 muscle params (L0, Fmax, vmax, origin, insertion)
  numGenes += 2 + (2 * 5);
  
#if USE_HILL_PARAMS  
  // Muscle params symmetric antagonists:
  // 2 X 4 hill parameters (hSh, hLn, hMax)
  numGenes += 2 * 3;  
#endif  
  
  // Reflex params
  // Spindle gains (p,v,d) and exponent
#if USE_UNIFORM_SPINDLES  
  numGenes += 1 * 4; // All spindles behave the same way
#else
  numGenes += 2 * 4; // Spindles differ between elbow and shoulder
#endif
  
  // Input to motor neurons
  // Weights from spindle (stretch reflex)
  numGenes += 2 * 1;  
    
  // Parameters for IaIn neurons
  // Bias, tau, weights from ia, rn, sp, A
  // Plus input to MN
#if ENABLE_IAIN
  numGenes += 2 * 7;
#endif
  
  // Renshaw neurons
  // Bias, tau, weights from mn, rn
  // Plus input to MN  
#if ENABLE_RENSHAW  
  numGenes += 2 * 5;
#endif
  
  // IbIn neurons
  // Bias, tau, weights from golgi, ib
  // Plus input to MN  
#if ENABLE_IBIN
  numGenes += 2 * 5;  
#endif
  
  // Parameters of ifv neurons
#if ENABLE_IFV
  numGenes += 2 * 2;
#endif
  
  // Open-loop activation (numMuscles x 3 target poses)
#if ENABLE_OPENLOOP
  numGenes += 2 * 3;  
#endif
  
  return numGenes;
};

//----------------------------------------------------------------------------------------------------------------------
void EvoArmCoCon::decodeGenome(const double* genome)
{
  double elbRad = 0.02 + 0.03 * genome[0];
  double shdRad = 0.02 + 0.03 * genome[1];
  m_arm->setJointRadii(elbRad, shdRad);
  
  int start = 2;
  for(int i = 0; i < m_arm->getNumMuscles() / 2; ++i)
  {
    // Muscle parameters
    double origin = 0.1 + 0.15 * genome[start + 0];
    double insertion =  0.075 + 0.125 * genome[start + 1];
    double maxIsoForce = 100.0 + 2000.0 * genome[start + 2]; 
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
    
#if USE_HILL_PARAMS
    double hSh = 0.1 + 0.4 * genome[start + 0];
    double hLn = 0.1 + 0.4 * genome[start + 1];
    double hMax = 1.1 + 0.7 * genome[start + 2];
    double hSlp = 2.0;
    m_arm->getMuscle(i*2)->setHillParameters(hSh, hLn, hMax, hSlp);
    m_arm->getMuscle(i*2 + 1)->setHillParameters(hSh, hLn, hMax, hSlp);
    start += 3;
#endif
  }
  
  // Spindle parameters
  const float maxSpP = 20.0;
  const float maxSpV = 0.0;
  const float maxSpD = 20.0;
  const float minSpExp = 0.8;
  m_arm->getReflex(0)->setSpindleParameters(genome[start + 0]*maxSpP, genome[start + 0]*maxSpP, 
                                            genome[start + 1]*maxSpV, genome[start + 1]*maxSpV, 
                                            genome[start + 2]*maxSpD, genome[start + 2]*maxSpD, 
                                            minSpExp + (genome[start + 3] * (1 - minSpExp)), 
                                            minSpExp + (genome[start + 3] * (1 - minSpExp)));  
#if USE_UNIFORM_SPINDLES  
  m_arm->getReflex(1)->setSpindleParameters(genome[start + 0]*maxSpP, genome[start + 0]*maxSpP, 
                                            genome[start + 1]*maxSpV, genome[start + 1]*maxSpV, 
                                            genome[start + 2]*maxSpD, genome[start + 2]*maxSpD, 
                                            minSpExp + (genome[start + 3] * (1 - minSpExp)), 
                                            minSpExp + (genome[start + 3] * (1 - minSpExp)));  
  start += 4;
#else
  m_arm->getReflex(1)->setSpindleParameters(genome[start + 4]*maxSpP, genome[start + 4]*maxSpP, 
                                            genome[start + 5]*maxSpV, genome[start + 5]*maxSpV, 
                                            genome[start + 6]*maxSpD, genome[start + 6]*maxSpD, 
                                            minSpExp + (genome[start + 7] * (1 - minSpExp)), 
                                            minSpExp + (genome[start + 7] * (1 - minSpExp)));  
  start += 8;  
#endif

  
  // Alpha MN params: input from spindles
  float maxW = 10.0;
  m_arm->getReflex(0)->setMotoNeuronParameters(genome[start+0] * maxW, genome[start+0] * maxW); 
  m_arm->getReflex(1)->setMotoNeuronParameters(genome[start+1] * maxW, genome[start+1] * maxW);
  start += 2;

#if ENABLE_IAIN
  maxW = 10.0;
  float maxB = 10.0;
  float maxT = 100.0;  
  
  // IaIn parameters
  m_arm->getReflex(0)->setIaInParameters(genome[start+0] * maxW, genome[start+0] * maxW,        // spindle->ia
                                         genome[start+1] * maxW, genome[start+1] * maxW,        // ia->ia
                                         genome[start+2] * maxW, genome[start+2] * maxW,        // desired contr->ia
                                         genome[start+3] * maxW, genome[start+3] * maxW,        // rn->ia
                                         genome[start+4] * maxW, genome[start+4] * maxW,        // ia->mn
                                         10 + genome[start+5] * maxT, 10 + genome[start+5] * maxT,        // time constants
                                         -maxB + (genome[start+6] * 2 * maxB), -maxB + (genome[start+6] * 2 * maxB));       // biases
                                         //0.0, 0.0); // bias
  
  m_arm->getReflex(1)->setIaInParameters(genome[start+7] * maxW, genome[start+7] * maxW,
                                         genome[start+8] * maxW, genome[start+8] * maxW,
                                         genome[start+9] * maxW, genome[start+9] * maxW,
                                         genome[start+10] * maxW, genome[start+10] * maxW,
                                         genome[start+11] * maxW, genome[start+11] * maxW,
                                         10 + genome[start+12] * maxT, 10 + genome[start+12] * maxT,
                                         -maxB + (genome[start+13] * 2 * maxB), -maxB + (genome[start+13] * 2 * maxB));  
                                         //0.0, 0.0);
  start += 14;
#endif  
  
#if ENABLE_RENSHAW
  // Renshaw param
  m_arm->getReflex(0)->setRenshawParameters(genome[start+0], genome[start+0], 
                                            genome[start+1], genome[start+1], 
                                            genome[start+2], genome[start+2], 
                                            10 + genome[start+3] * maxT, 10 + genome[start+3] * maxT,
                                            -maxB + (genome[start+4] * 2 * maxB), -maxB + (genome[start+4] * 2 * maxB));  
                                            //0.0, 0.0);
  
  m_arm->getReflex(1)->setRenshawParameters(genome[start+5], genome[start+5], 
                                            genome[start+6], genome[start+6], 
                                            genome[start+7], genome[start+7], 
                                            10 + genome[start+8] * maxT, 10 + genome[start+8] * maxT,
                                            -maxB + (genome[start+9] * 2 * maxB), -maxB + (genome[start+9] * 2 * maxB));  
                                            //0.0, 0.0);  
  start += 10;  
#endif
  
#if ENABLE_IBIN  
  // IbIn params
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
#endif   
  
  // IFV gains
#if ENABLE_IFV  
  const float maxIC = 10.0f;
  const float maxICb = 1.0f;
  m_arm->getReflex(0)->setInertiaCompensationParameters(genome[start+0] * maxIC, genome[start+0] * maxIC, 
                                                        genome[start+1] * maxICb, genome[start+1] * maxICb);
  
  m_arm->getReflex(1)->setInertiaCompensationParameters(genome[start+2] * maxIC, genome[start+2] * maxIC, 
                                                        genome[start+3] * maxICb, genome[start+3] * maxICb);  
  start +=4;
#endif  

#if ENABLE_OPENLOOP
  for (int i = 0; i < 6; ++i)
  {
    m_openLoop[i] = MAX_OPENLOOP * genome[start + i];
  }
  start += 6;
#endif
  
  // Trajectory related values
#if EVOLVE_MIN_JERK_DELAY  
  m_minJerkTrajDelay = 0.1f * genome[start];
  start += 1;
#endif
  
  m_rampDurationFactor = genome[start + 1];
  start += 1;
  
  createTrajectories();
};

//----------------------------------------------------------------------------------------------------------------------
// TODO: Remove this test code
struct VecDist
{
  double operator() (ci::Vec2f t1, ci::Vec2f t2) { return (t1-t2).lengthSquared(); };
};  
  
float EvoArmCoCon::getFitness() 
{ 
  const double dt = 1.0 / 200.0;
  float distFit = 1.0 - sqrt(m_fitness / (m_time / dt));
#if ENABLE_COCON_INCREASE  
  float coconFit = m_coconIncrements[0] * m_coconIncrements[1] * m_coconIncrements[2]; 
  distFit *= coconFit;
#endif
  
  int minDelay = -100;
  int maxDelay = 100;
  std::vector<double> cc = crossCorrelation(m_desiredPositions, m_actualPositions, VecDist(), minDelay, maxDelay);
  std::vector<double>::iterator optElem = min_element(cc.begin(), cc.end());
  double optVal = *optElem / (m_time / dt);
  int optAt = minDelay + std::distance(cc.begin(), optElem);
  float distFit2 = 1.0 - sqrt(optVal);
  
  return distFit2;
};

//----------------------------------------------------------------------------------------------------------------------
void EvoArmCoCon::init() 
{ 
  assert(m_arm); 
  m_arm->init(); 
  
  reset();
}
  
//----------------------------------------------------------------------------------------------------------------------
void EvoArmCoCon::createTrajectories() 
{   
  // First clear existing trajectories, as below we will append
  m_desiredTrajectory.clear();
  m_commandedTrajectory.clear();
  m_targetJointAngles.clear();
  m_targetElbLengths.clear();
  m_targetShdLengths.clear();
  
  Pos startPos, target1Pos, target2Pos, tmp;
  
  // Starting position: symmetric
  const float startPosElb = 0;
  const float startPosShd = 0;
  m_arm->forwardKinematics(startPosElb, startPosShd, tmp, startPos);
  
  // Target positions
  const float target1PosElb = PI_OVER_TWO;
  const float target2PosElb = -PI_OVER_TWO;  
  const float target1PosShd = 0.0f;
  const float target2PosShd = 0.0f;  
  m_arm->forwardKinematics(target1PosElb, target1PosShd, tmp, target1Pos);     
  m_arm->forwardKinematics(target2PosElb, target2PosShd, tmp, target2Pos);     
  
  // Desired trajectory is minimum jerk of given length

#if EVOLVE_MIN_JERK_DELAY  
  float desiredOffset = m_minJerkTrajDelay;
#else
  float desiredOffset = 0;   
#endif
  m_desiredTrajectory.add(startPos, DESIRED_SETTLING_TIME + desiredOffset);   
  m_desiredTrajectory.add(startPos, DESIRED_MOVEMENT_TIME);  
  m_desiredTrajectory.add(target1Pos, DESIRED_SETTLING_TIME); 
  m_desiredTrajectory.add(target1Pos, DESIRED_MOVEMENT_TIME);
  m_desiredTrajectory.add(startPos, DESIRED_SETTLING_TIME);   
  m_desiredTrajectory.add(startPos, DESIRED_MOVEMENT_TIME);  
  m_desiredTrajectory.add(target2Pos, DESIRED_SETTLING_TIME); 
  m_desiredTrajectory.add(target2Pos, DESIRED_MOVEMENT_TIME);
  m_desiredTrajectory.add(startPos, DESIRED_SETTLING_TIME);  
  m_desiredTrajectory.add(startPos, DESIRED_SETTLING_TIME);  
  m_desiredTrajectory.setBlend(Trajectory<ci::Vec2f>::kTr_BlendMinJerk);
  m_desiredTrajectory.setLoop(false);  
  
  // Commanded trajectory moves faster potentially, and settles correspondingly longer
  float commandDurationFactor = m_rampDurationFactor;
  float commandMovementTime = DESIRED_MOVEMENT_TIME * commandDurationFactor;
  float commandSettlingTime = DESIRED_SETTLING_TIME + (DESIRED_MOVEMENT_TIME - commandMovementTime);
  m_commandedTrajectory.add(startPos, DESIRED_SETTLING_TIME);   
  m_commandedTrajectory.add(startPos, commandMovementTime);  
  m_commandedTrajectory.add(target1Pos, commandSettlingTime); 
  m_commandedTrajectory.add(target1Pos, commandMovementTime);
  m_commandedTrajectory.add(startPos, commandSettlingTime);   
  m_commandedTrajectory.add(startPos, commandMovementTime);  
  m_commandedTrajectory.add(target2Pos, commandSettlingTime); 
  m_commandedTrajectory.add(target2Pos, commandMovementTime);
  m_commandedTrajectory.add(startPos, commandSettlingTime);  
  // Make sure they two trajectories are equally long
  m_commandedTrajectory.add(startPos, commandSettlingTime + desiredOffset);
  m_commandedTrajectory.setBlend(Trajectory<ci::Vec2f>::kTr_BlendLinear);
  m_commandedTrajectory.setLoop(false);    
  
  // Transform cartesian effector position into joint angle targets
  for(int i = 0; i < m_commandedTrajectory.size(); i++)
  {
    int elbDir = i < 5 ? 1 : -1;
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

#if ENABLE_COCON_INCREASE
  m_cocontraction = 0.0;
  
  if (command.id == 2 || command.id == 4 || command.id == 6)
  {     
    // Determine how far proportionally we're through with this target
    m_cocontraction = (m_time - command.start) / (command.stop - command.start);
    // Limit range
    m_cocontraction *= MAX_COCONTRACTION;
  } 
  
  // Apply
  m_arm->getReflex(0)->setCocontraction(m_cocontraction, m_cocontraction);
  m_arm->getReflex(1)->setCocontraction(m_cocontraction, m_cocontraction);    
#endif  

#if ENABLE_OPENLOOP
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
#endif  

#if USE_MUSCLE_COORDS
  Pos commandedElbLengths = m_targetElbLengths.atTime(m_time).position;
  Pos commandedShdLengths = m_targetShdLengths.atTime(m_time).position;  
  m_arm->update(commandedElbLengths.x, commandedElbLengths.y, commandedShdLengths.x, commandedShdLengths.y, dt);  
  //m_arm->update(desiredAngles.x, desiredAngles.y, dt);  
#else
  m_arm->update(command.position, dt, command.id < 5 ? 1 : -1);    
#endif

  
  
  // Measure real co-contraction increment between beginning and end of commanded increase
  // First target
  if(command.id == 2 || command.id == 4 || command.id == 6)
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
    if((command.stop - m_time) <= dt)
    {
      double coconAtEnd [2];
      coconAtEnd[0] = 0.5 * (m_arm->getReflex(0)->getAlphaOutput(0) + m_arm->getReflex(0)->getAlphaOutput(1));
      coconAtEnd[1] = 0.5 * (m_arm->getReflex(1)->getAlphaOutput(0) + m_arm->getReflex(1)->getAlphaOutput(1));
      double totalInc = (coconAtEnd[0] - m_coconAtStart[0]) + (coconAtEnd[1] - m_coconAtStart[1]);
      assert(totalInc == totalInc);
      
      m_coconIncrements[desired.id/2-1] = 0.5 * totalInc; // Normalise to [0,1]. Can be 1 at max per reflex, so 2 in total.
      m_coconStarted = false;
    }
  }
  
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
  // Transform desired position to joint angles (IK) and muscle lengths
  m_currentDesiredPos = desired.position;
  m_currentCommandPos = command.position;
  
  double currentDesiredAngles[2];
  m_arm->inverseKinematics(m_currentDesiredPos, command.id < 5 ? 1.0f : -1.0f, currentDesiredAngles[0], currentDesiredAngles[1]);
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
  
} // namespace dmx
#endif