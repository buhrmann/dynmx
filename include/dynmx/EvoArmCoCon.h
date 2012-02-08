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
  void record(Recorder& recorder);
  
  
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
  
  float m_minJerkTrajDelay;     // Time between start of commanded ramp and min jerk trajectory used for fitness
  float m_rampDurationFactor;   // Scales the duration of the commanded ramp as proportion of desired movement time
  
  std::vector<float> m_coconIncrements;  // At three different positions
  double m_coconAtStart[2];       // For two reflexes
  bool m_coconStarted;
  
  double m_openLoop[6];  
  
  // Flags and params
  bool m_evolveIAIN;
  bool m_evolveRenshaw;
  bool m_evolveIBIN;
  bool m_evolveIFV;
  bool m_evolveSFV;
  bool m_evolveOpenLoop;
  bool m_evolveIntersegmentInputs;
  bool m_evolveUniformSpindles;
  bool m_evolveHillParams;
  bool m_evolveMinJerkDelay;
  
  bool m_enableCoconIncrease;
  bool m_useMuscleCoords;
  
  float m_maxOpenLoop;
  float m_minCocontraction;
  float m_maxCocontraction;
};
  
} // namespace dmx

#endif