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
#include "ArmMuscled.h"
#include "MuscleMonoWrap.h"
#include "MuscleBiWrap.h"
#include "Trajectory.h"

namespace dmx
{
  
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
  
  
  ArmMuscled* m_arm;
  
protected:
  
  float m_time;
  float m_fitness;
  double m_cocontraction;
  
  double m_coconIncrements[3];  // At three different positions
  double m_coconAtStart[2];     // For two reflexes
  bool m_coconStarted;
  
  
  Trajectory<ci::Vec2f> m_targets;  
  MinJerkTrajectory m_minJerkTraj;
};

//----------------------------------------------------------------------------------------------------------------------
// Inline Implementation
//----------------------------------------------------------------------------------------------------------------------
EvoArmCoCon::EvoArmCoCon() : 
m_arm(0), 
m_time(0) 
{ 
  m_arm = new dmx::ArmMuscled(); 
  init();
};


//----------------------------------------------------------------------------------------------------------------------
int EvoArmCoCon::getNumGenes() 
{ 
  int numGenes = 0;
  
  // Muscle params symmetric antagonists:
  // 2 X joint radii, 5 muscle params (L0, Fmax, vmax, origin, insertion)
  numGenes += 2 + ((m_arm->getNumMuscles() / 2) * 5);
  
  // Reflex params:
  // Static and dynamic spindle gains x 2 muscles
  numGenes += (m_arm->getNumMuscles() / 2) * 3;
    
  // Parameters for IaIn neurons
  //numGenes += (m_arm->getNumMuscles() / 2) * 4;
  
  // Input to motor neurons: weight from IaIn
  //numGenes += (m_arm->getNumMuscles() / 2) * 1;  
  
  // Parameters of ifv neurons
  numGenes += (m_arm->getNumMuscles() / 2) * 2;    
  
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
    double maxIsoForce = 1500 * genome[start + 2]; 
    double optimalLength = 0.5 + 0.5 * genome[start + 3];          
    double maxVel = 8 + 4 * genome[start + 4];                  
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
  }
  
  // Spindle parameters
  const float maxSpP = 1.0;
  const float maxSpV = 1.0;
  const float minSpExp = 0.5;
  m_arm->getReflex(0)->setSpindleParameters(genome[start + 0]*maxSpP, genome[start + 0]*maxSpP, 
                                            genome[start + 1]*maxSpV, genome[start + 1]*maxSpV, 
                                            minSpExp + (genome[start + 2] * (1 - minSpExp)), 
                                            minSpExp + (genome[start + 2] * (1 - minSpExp)));  
  
  m_arm->getReflex(1)->setSpindleParameters(genome[start + 3]*maxSpP, genome[start + 3]*maxSpP, 
                                            genome[start + 4]*maxSpV, genome[start + 4]*maxSpV, 
                                            minSpExp + (genome[start + 5] * (1 - minSpExp)), 
                                            minSpExp + (genome[start + 5] * (1 - minSpExp)));  
  start += 6;    
    
  // IaIn parameters
  /*
  const float maxW = 10.0;
  const float maxB = 1.0;
  const float maxT = 100.0;
  m_arm->getReflex(0)->setIaInParameters(genome[start+0] * maxW, genome[start+0] * maxW,        // spindle->ia
                                         genome[start+1] * maxW, genome[start+1] * maxW,        // ia->ia
                                         10 + genome[start+2] * maxT, 10 + genome[start+2] * maxT,        // time constants
                                         //-maxB + (genome[start+3] * 2 * maxB), -maxB + (genome[start+3] * 2 * maxB));       // biases
                                         0.0, 0.0);
  
  m_arm->getReflex(1)->setIaInParameters(genome[start+4] * maxW, genome[start+4] * maxW,
                                         genome[start+5] * maxW, genome[start+5] * maxW,
                                         10 + genome[start+6] * maxT, 10 + genome[start+6] * maxT,
                                         //-maxB + (genome[start+7] * 2 * maxB), -maxB + (genome[start+7] * 2 * maxB));  
                                         0.0, 0.0);
  start += 8;
  
  // Alpha MN params
  m_arm->getReflex(0)->setMotoNeuronParameters(genome[start+0] * maxW, genome[start+0] * maxW); // ia->mn
  m_arm->getReflex(1)->setMotoNeuronParameters(genome[start+1] * maxW, genome[start+1] * maxW);    
  start += 2;
   */
  
  // IFV gains
  const float maxIC = 10.0f;
  const float maxICb = 1.0f;
  m_arm->getReflex(0)->setInertiaCompensationParameters(genome[start+0] * maxIC, genome[start+0] * maxIC, 
                                                        genome[start+1] * maxICb, genome[start+1] * maxICb);
  
  m_arm->getReflex(1)->setInertiaCompensationParameters(genome[start+2] * maxIC, genome[start+2] * maxIC, 
                                                        genome[start+3] * maxICb, genome[start+3] * maxICb);  
  
};

//----------------------------------------------------------------------------------------------------------------------
float EvoArmCoCon::getFitness() 
{ 
  const double dt = 1.0 / 100.0;
  float distFit = 1.0 - sqrt(m_fitness / (m_time / dt));
  float coconFit = m_coconIncrements[0] * m_coconIncrements[1] * m_coconIncrements[2]; 
  return distFit;// * coconFit;
};

//----------------------------------------------------------------------------------------------------------------------
void EvoArmCoCon::init() 
{ 
  assert(m_arm); 
  m_arm->init(); 
  
  // Starting position: symmetric
  const float startPosElb = 0;
  const float startPosShd = 0;
  Pos startPos, target1Pos, target2Pos, tmp;
  m_arm->forwardKinematics(startPosElb, startPosShd, tmp, startPos);
  
  // Target positionss
  const float target1PosElb = PI_OVER_TWO;
  const float target2PosElb = -PI_OVER_TWO;  
  const float target1PosShd = 0.0f;
  const float target2PosShd = 0.0f;  
  m_arm->forwardKinematics(target1PosElb, target1PosShd, tmp, target1Pos);     
  m_arm->forwardKinematics(target2PosElb, target2PosShd, tmp, target2Pos);     
  
  m_targets.add(startPos, 1.0f);    // time to get to start
  m_targets.add(startPos, 0.25f);    // time to stay at start
  m_targets.add(target1Pos, 1.0f);  // time to get to target pos
  m_targets.add(target1Pos, 0.25f);  // time to stay at target pos
  m_targets.add(startPos, 1.0);     // time to return to start
  m_targets.add(startPos, 0.25);     // time to stay at start  
  m_targets.add(target2Pos, 1.0f);  // time to get to target pos
  m_targets.add(target2Pos, 0.25f);  // time to stay at target pos
  m_targets.add(startPos, 1.0);     // time to return to start
  m_targets.add(startPos, 1.0);     // time to return to start
  
  m_targets.setBlend(Trajectory<ci::Vec2f>::kTr_BlendMinJerk);
  m_targets.setLoop(false);  
  
  reset();    
}

//----------------------------------------------------------------------------------------------------------------------
void EvoArmCoCon::reset()
{
  m_fitness = 0.0f;
  m_time = 0.0f;
  m_coconStarted = false;
  m_arm->reset();
}

//----------------------------------------------------------------------------------------------------------------------
void EvoArmCoCon::update(float dt)
{ 
  Target<Pos> desired = m_targets.at(m_time);
  
  // Control co-contraction
  const float maxCoContraction = 0.5f;
  m_cocontraction = 0.0;
  /*
  if (desired.id == 2 || desired.id == 4 || desired.id == 6)
  {
    // Determine how far proportionally we're through with this target
    m_cocontraction = (m_time - desired.start) / (desired.stop - desired.start);
    // Limit range
    m_cocontraction *= maxCoContraction;
  } 
   */
  
  // Apply
  m_arm->getReflex(0)->setCocontraction(m_cocontraction, m_cocontraction);
  m_arm->getReflex(1)->setCocontraction(m_cocontraction, m_cocontraction);    

  
  // Control elbow direction of inverse kinematics solution
  if(desired.id < 5)
  {
    // "Normal" elbow flexion
    m_arm->updatePosition(desired.position, dt);    
  }
  else 
  {
    // Inverse elbow flexion
    m_arm->updatePosition(desired.position, dt, -1);      
  }
  
  
  // Measure real co-contraction increment between beginning and end of commanded increase
  // First target
  if(desired.id == 2 || desired.id == 4 || desired.id == 6)
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
    if((desired.stop - m_time) <= dt)
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
  Pos diff = m_arm->getEffectorPos() - desired.position;
  float dist = diff.length();
  m_fitness += sqr(dist);
  
  m_time += dt;
};

//----------------------------------------------------------------------------------------------------------------------  
bool EvoArmCoCon::hasFinished()
{
  return m_time >= m_targets.getDuration();
}

//----------------------------------------------------------------------------------------------------------------------  
void EvoArmCoCon::finish()
{
  std::cout << "Fitness: " << getFitness() << std::endl;
}  
  
} // namespace dmx
#endif