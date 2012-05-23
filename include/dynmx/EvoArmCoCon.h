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
  
  enum MovePhase
  {
    kMvPh_invalid = -1,
    kMvPh_leadIn,
    kMvPh_move,    
    kMvPh_leadOut,
    kMvPh_end,
    kMvPh_numPhases
  };
  
  static const std::string PhaseNames [kMvPh_numPhases];
  
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
  std::vector<double> m_actualCoactivationsElb;
  std::vector<double> m_actualCoactivationsShd;
  
protected:
  
  void createTrajectories();
  void updateCurrentCommand(Target<Pos>& command, Target<Pos>& desired);
  int decodeMuscle(int mId, const double* genome, int start);
  void readDecodeLimits();
  void readRange(Range& range, const std::string& xmlElem);
  float getFitness(float t0, float tmax);
  
  // Holds data constraining the decoding of a genome into this model
  struct DecodeLimits 
  {
    struct ArmLimits
    {
      Range jointRadius;
      Range jointFriction;
    } arm;
    
    struct MuscleLimits
    { 
      Range attach, force, optLength, maxVel;
    } muscle;
    
    struct SpindleLimits
    {
      Range pos, vel, dmp, exp, weight;
    } spindle;
  };
  
  DecodeLimits m_decodeLimits;
  
  float m_time;
  float m_bestFitDelay;
  double m_cocontraction;
  
  float m_minJerkTrajDelay;     // Time between start of commanded ramp and min jerk trajectory used for fitness
  float m_rampDurationFactor;   // Scales the duration of the commanded ramp as proportion of desired movement time
  
  std::vector<float> m_coconIncrements;  // At three different positions
  double m_coconAtStart[2];              // For two reflexes
  bool m_coconStarted;
  
  // Size of these depend on the number of moves specified in config file
  std::vector<double> m_openLoopParams;  
  std::vector<double> m_intersegParams;
  
  // Fitness related
  bool m_resetEachMove;
  bool m_fitnessEndPointOnly;
  bool m_fitnessEvalVel;
  bool m_fitnessEvalCoact;
  double m_trialFitness;
  double m_fitness;
  
  // Trajectory related
  int m_numMoves;
  int m_currentMove;
  int m_currentPhase;
  
  // Flags and params
  bool m_evolveMuscles;
  bool m_evolveFriction;
  bool m_evolveRampDuration;
  bool m_symmetricMuscles;
  bool m_evolveSpindles;
  bool m_evolveIAIN;
  bool m_evolveIAINsimple;
  bool m_evolveIAINsym;  
  bool m_evolveRenshaw;
  bool m_evolveIBIN;
  bool m_evolveIFV;
  bool m_evolveSFV;
  bool m_evolveOpenLoop;
  int m_openLoopSymmetry;
  bool m_evolveIntersegmentInputs;
  int m_evolveUniformSpindles;
  int m_evolveUniformSpindleWeights;
  bool m_evolveHillParams;
  bool m_evolveMinJerkDelay;
  bool m_evolveVelRef;
  
  bool m_commandTrajSmooth;
  bool m_enableCoconIncrease;
  bool m_useMuscleCoords;
  
  float m_maxOpenLoop;
  float m_maxInterseg;
  float m_minCocontraction;
  float m_maxCocontraction;
};
  
} // namespace dmx

#endif