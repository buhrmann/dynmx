/*
 *  EvoArmSimpleReflex.h
 *  cinder_dynmx
 *
 *  Created by Thomas Buhrmann on 9/23/11.
 *  Copyright 2011 __MyCompanyName__. All rights reserved.
 *
 */

#ifndef _DMX_EVO_ARM_SIMPLE_REFLEX
#define _DMX_EVO_ARM_SIMPLE_REFLEX

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
class EvoArmSimpleReflex : public Evolvable
{
  
public:
  
  EvoArmSimpleReflex();
  
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
  double m_cocontraction[12];
  
  Trajectory<ci::Vec2f> m_targets;  
};

//----------------------------------------------------------------------------------------------------------------------
// Inline Implementation
//----------------------------------------------------------------------------------------------------------------------
EvoArmSimpleReflex::EvoArmSimpleReflex() : 
  m_arm(0), 
  m_time(0) 
{ 
  m_arm = new dmx::ArmMuscled(); 
  init();
};


//----------------------------------------------------------------------------------------------------------------------
int EvoArmSimpleReflex::getNumGenes() 
{ 
  int numGenes = 0;
  
  // Muscle params:
  // 2 X joint radii, 5 muscle params (L0, Fmax, vmax, origin, insertion)
  numGenes += 2 + ((m_arm->getNumMuscles() / 2) * 5);
  
  // Reflex params:
  // Static and dynamic spindle gains x 2 muscles
  numGenes += (m_arm->getNumMuscles() / 2) * 3;
  
  // Plus 12 cocontraction signals (numMuscles x 3 target poses)
  numGenes += m_arm->getNumMuscles() * 3;
  
  // Gain and inhibition of sfv neurons
  numGenes += (m_arm->getNumMuscles() / 2) * 2;
  
  // Parameters of ifv neurons
  numGenes += (m_arm->getNumMuscles() / 2) * 2;  
  
  // Parameters for IaIn neurons
  numGenes += (m_arm->getNumMuscles() / 2) * 3;
  
  return numGenes;
};

//----------------------------------------------------------------------------------------------------------------------
void EvoArmSimpleReflex::decodeGenome(const double* genome)
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
  const float maxSpP = 0.5;
  const float maxSpV = 0.1;
  const float minSpExp = 0.5;
  m_arm->getReflex(0)->setSpindleParameters(genome[start + 0]*maxSpP, genome[start + 0]*maxSpP, 
                                            genome[start + 1]*maxSpV, genome[start + 1]*maxSpV, 
                                            minSpExp + (genome[start + 2] * (1 - minSpExp)), 
                                            minSpExp + (genome[start + 2] * (1 - minSpExp)));  

  m_arm->getReflex(1)->setSpindleParameters(genome[start + 3]*maxSpP, genome[start + 3]*maxSpP, 
                                            genome[start + 4]*maxSpV, genome[start + 4]*maxSpV, 
                                            minSpExp + (genome[start + 5] * (1 - minSpExp)), 
                                            minSpExp + (genome[start + 5] * (1 - minSpExp)));  
  
  // Concontraction values
  start += 6;  
  const float maxCoContraction = 0.01f;
  for (int i = 0; i < 12; ++i)
  {
    m_cocontraction[i] = maxCoContraction * genome[start + i];
  }
  
  // SFV gains
  start += 12;
  const float maxLC = 10.0f;
  m_arm->getReflex(0)->setLoadCompensationParameters(genome[start+0] * maxLC, genome[start+0] * maxLC, 
                                                     genome[start+1] * maxLC, genome[start+1] * maxLC);
  
  m_arm->getReflex(1)->setLoadCompensationParameters(genome[start+2] * maxLC, genome[start+2] * maxLC, 
                                                     genome[start+3] * maxLC, genome[start+3] * maxLC);  
  
  // IFV gains
  start += 4;
  const float maxIC = 10.0f;
  const float maxICb = 1.0f;
  m_arm->getReflex(0)->setInertiaCompensationParameters(genome[start+0] * maxIC, genome[start+0] * maxIC, 
                                                        genome[start+1] * maxICb, genome[start+1] * maxICb);
  
  m_arm->getReflex(1)->setInertiaCompensationParameters(genome[start+2] * maxIC, genome[start+2] * maxIC, 
                                                        genome[start+3] * maxICb, genome[start+3] * maxICb);
  
  // IaIn parameters
  start += 4;
  const float maxIaExc = 10.0;
  const float maxIaIn = 10.0;
  const float maxIaT = 100.0;
  m_arm->getReflex(0)->setIaInParameters(genome[start+0] * maxIaExc, genome[start+0] * maxIaExc,
                                         genome[start+1] * maxIaIn, genome[start+1] * maxIaIn,
                                         genome[start+2] * maxIaT, genome[start+2] * maxIaT);

  m_arm->getReflex(1)->setIaInParameters(genome[start+3] * maxIaExc, genome[start+3] * maxIaExc,
                                         genome[start+4] * maxIaIn, genome[start+4] * maxIaIn,
                                         genome[start+5] * maxIaT, genome[start+5] * maxIaT);
  
  
                                                                                   
};

//----------------------------------------------------------------------------------------------------------------------
float EvoArmSimpleReflex::getFitness() 
{ 
  const double dt = 1.0 / 100.0;
  return -/*sqrt*/(m_fitness / (m_time / dt)); 
};

//----------------------------------------------------------------------------------------------------------------------
void EvoArmSimpleReflex::init() 
{ 
  assert(m_arm); 
  m_arm->init(); 
  
  // Starting position: asymmetric
  /*
  const float startPosElb = PI_OVER_FOUR;
  const float startPosShd = 0;
  Pos startPos, target1Pos, tmp;
  m_arm->forwardKinematics(startPosElb, startPosShd, tmp, startPos);
  
  // Synergistic movement
  const float target1PosElb = PI_OVER_TWO;
  const float target1PosShd = 0.0f;
  m_arm->forwardKinematics(target1PosElb, target1PosShd, tmp, target1Pos);     
  
  m_targets.add(startPos, 1.5f);    // time to get to start
  m_targets.add(startPos, 0.5f);    // time to stay at start
  m_targets.add(target1Pos, 1.0f);  // time to get to target pos
  m_targets.add(target1Pos, 0.5f);  // time to stay at target pos
  m_targets.add(startPos, 1.0);     // time to return to start
  m_targets.add(startPos, 0.5);     // time to stay at start
  */
  
  // Starting position: symmetric
  const float startPosElb = 0;
  const float startPosShd = 0;
  Pos startPos, target1Pos, target2Pos, tmp;
  m_arm->forwardKinematics(startPosElb, startPosShd, tmp, startPos);
  
  // Synergistic movement
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
  
  m_targets.setBlend(true);
  m_targets.setLoop(false);  
  
  reset();    
}

//----------------------------------------------------------------------------------------------------------------------
void EvoArmSimpleReflex::reset()
{
  m_fitness = 0.0f;
  m_time = 0.0f;
  m_arm->reset();
}

//----------------------------------------------------------------------------------------------------------------------
void EvoArmSimpleReflex::update(float dt)
{ 
  Target<Pos> desired = m_targets.at(m_time);
  desired.id++; // With blending enabled the current target is moving towards the next target
  
  if (desired.id == 2 || desired.id == 3)
  {
    m_arm->getReflex(0)->setCocontraction(m_cocontraction[4], m_cocontraction[5]);
    m_arm->getReflex(1)->setCocontraction(m_cocontraction[6], m_cocontraction[7]);        
  }
  else if (desired.id == 6 || desired.id == 7)
  {
    m_arm->getReflex(0)->setCocontraction(m_cocontraction[8], m_cocontraction[9]);
    m_arm->getReflex(1)->setCocontraction(m_cocontraction[10], m_cocontraction[11]);        
  }
  else
  {
    m_arm->getReflex(0)->setCocontraction(m_cocontraction[0], m_cocontraction[0]);
    m_arm->getReflex(1)->setCocontraction(m_cocontraction[1], m_cocontraction[1]);    
  }  
  
  if(desired.id < 6)
  {
    m_arm->updatePosition(desired.position, dt);    
  }
  else 
  {
    m_arm->updatePosition(desired.position, dt, -1);      
  }


  
  // Fitness evaluation
  Pos diff = m_arm->getEffectorPos() - desired.position;
  float dist = diff.length();
  m_fitness += /*sqr*/(dist);
  
  m_time += dt;
};
  
//----------------------------------------------------------------------------------------------------------------------  
bool EvoArmSimpleReflex::hasFinished()
{
  return m_time >= m_targets.getDuration();
}
  
//----------------------------------------------------------------------------------------------------------------------  
void EvoArmSimpleReflex::finish()
{
  std::cout << "Fitness: " << getFitness() << std::endl;
}  

} // namespace dmx
#endif