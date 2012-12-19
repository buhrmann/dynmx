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
  Trajectory<ci::Vec2f> m_commandedJointAngles;     // same targets but in joint angle space
  
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
  void decodeSpindles(int move=0);
  void readDecodeLimits();
  void readRange(Range& range, const std::string& xmlElem);
  float getFitness(float t0, float tmax);
  
  // Holds data constraining the decoding of a genome into this model
  struct ArmLimits
  {
    Range jointRadius;
    Range jointFriction;
  };
  
  struct MuscleLimits
  { 
    Range attach, force, optLength, maxVel;
  };
  
  struct DecodeLimits 
  {
    ArmLimits arm;
    MuscleLimits muscle;
    Reflex::SpindleLimits spindle;
  };
  
  DecodeLimits m_decodeLimits;
  
  float m_time;
  float m_bestFitDelay;
  double m_cocontraction;
  
  float m_minJerkTrajDelay;     // Time between start of commanded ramp and min jerk trajectory used for fitness
  
  // Size of these depend on the number of moves specified in config file
  std::vector<double> m_openLoopParams;  
  std::vector<double> m_intersegParams;
  std::vector<double> m_distalDelays;
  std::vector<double> m_rampDurations;
  std::vector<double> m_spindleParams;
  
  // Fitness related
  bool m_resetEachMove;
  bool m_fitnessEndPointOnly;
  bool m_fitnessEvalVel;
  bool m_fitnessEvalCoact;
  float m_fitnessEvalCoactMax;
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
  bool m_evolveRampDurationPerMove;
  bool m_symmetricMuscles;
  bool m_evolveSpindles;
  bool m_evolveSpindlesPerMove;  
  int m_evolveSymmetricSpindles;
  int m_evolveSymmetricSpindleWeights;  
  bool m_evolveIAIN;
  bool m_evolveIAINsimple;
  bool m_evolveIAINsym;  
  bool m_evolveRenshaw;
  bool m_evolveRenshawSym;
  bool m_evolveIBIN;
  bool m_evolveIBINsym;
  bool m_evolveIBINderiv;
  bool m_evolveIBInterSeg;
  bool m_evolveIBRecExcIa;
  bool m_evolveIFV;
  bool m_evolveSFV;
  bool m_evolveOpenLoop;
  int m_openLoopSymmetry;
  bool m_evolveIntersegmentInputs;
  bool m_evolveIntersegPerMove;
  bool m_evolveIntersegSym;
  bool m_evolveSigmoidSlope;
  bool m_evolveMNAsNeuron;
  bool m_evolveMNsym;

  bool m_evolveHillParams;
  bool m_evolveMinJerkDelay;
  bool m_evolveVelRef;
  bool m_evolveDistalCommandDelay;
  
  int m_commandTrajSmooth;
  bool m_useMuscleCoords;
  
  float m_maxOpenLoop;
  float m_openLoopTauMaxAct;
  float m_openLoopTauMaxDeact;    
  float m_maxInterseg;
  float m_maxDistalDelay;
  
  bool m_hasHitLimit;
};
  
} // namespace dmx

#endif