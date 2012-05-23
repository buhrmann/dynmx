/*
 *  Arm.h
 *  FastCTRNN
 *
 *  Created by Thomas Buhrmann on 16/01/2010.
 *  Copyright 2010 __MyCompanyName__. All rights reserved.
 *
 */

#ifndef _DMX_ARM_
#define _DMX_ARM_

#include "Dynmx.h"
#include "Model.h"
#include "Recorder.h"

#include "cinder/Vector.h"

#include <deque>

namespace dmx
{

typedef ci::Vec2f Pos;

static const int MaxTrajPoints = 400;

enum Joint
{
  JT_invalid = -1,
  JT_elbow = 0,
  JT_shoulder = 1,
  JT_maxNumber
};
  
enum Integrator
{
  kInteg_euler = 0,
  kInteg_heun,
  kInteg_maxNumber
};

//----------------------------------------------------------------------------------------------------------------------
/// Implements equations of motion for a planar two-jointed arm
//----------------------------------------------------------------------------------------------------------------------
class Arm : public Model
{
    
public:
  
  struct State
  {
    double angles[2];
    double velocities[2];
    double accelerations[2];
    double coriolisAcc[2];
    double gravityAcc[2];
    double inertiaAcc[2];
    double interactionAcc[2];
    double dampingAcc[2];
    double torques[2]; 
  };  
  
public:  
  
  Arm();
  virtual ~Arm();
  
  // Inherited from class Model
  virtual void init();
  virtual inline void reset() { resetTo(0.0f, 0.0f); };
  virtual void update(float dt) { update(dt, 0.0, 0.0); };
  
  void resetTo(double elbAngle, double shdAngle);  
  void update(float dt, double torque1, double torque2);
  
  // Setters
  void setParameters(double mass1, double mass2, double length1, double length2);
  void setFriction(double elbF, double shdF) { m_frictions[JT_elbow] = elbF; m_frictions[JT_shoulder] = shdF; };
  void setLimit(Joint jt, double upper, double lower) { assert(jt >=0 && jt < 2); m_limits[jt][0] = upper; m_limits[jt][1] = lower; };
  void setGravity(double g) { m_gravity = g; };
  void setJointLocked(Joint j, bool l) { m_jointLocked[j] = l; };
  void setTarget(Pos p) { m_target = p; };    
  
  // Getters
  double getGravity() const { return m_gravity; };
  Pos getPointOnUpperArm(double distanceFromJoint) const;
  Pos getPointOnLowerArm(double distanceFromJoint) const;  
  const State& getState() const { return m_state; };
  double getJointAngle(Joint joint) { return m_state.angles[joint]; };
  double getLength(Joint joint) const { return m_lengths[joint]; } ;  
  double getTotalLength() const { return m_lengths[0] + m_lengths[1]; };
  const Pos& getEffectorPos() const { return m_effectorPos; };
  Pos& getEffectorPos() { return m_effectorPos; };
  const Pos& getElbowPos() const { return m_elbowPos; };
  Pos& getElbowPos() { return m_elbowPos; };
  double getJointLimitUpper(Joint j) const { return m_limits[j][0]; };
  double getJointLimitLower(Joint j) const { return m_limits[j][1]; };
  Pos getTarget() { return m_target; };
  const std::deque<Pos>& getTrajectory() { return m_trajectory; };
  
  // Helpers
  void forwardKinematics();
  void forwardKinematics(double elbAngle, double shdAngle, Pos& elbPos, Pos& effPos);
  void inverseKinematics(const Pos& pos, float elbDir, double& elbAngle, double& shdAngle);  

  // Store description in human readable format
  virtual void toXml(ci::XmlTree& xml);
  
  // Store output in recorder
  virtual void record(Recorder& recorder);  
  
  
protected:
  
  void preCompute();
  void solveEuler(double dt);
  void solveImprovedEuler(double dt); 
  void computeAccelerations(State& state);  
  
  bool m_jointLocked[2];
  bool m_interactionTorques;
  
  // States
  State m_state;  
  
  Pos
    m_elbowPos,
    m_effectorPos,
    m_target;
  
  std::deque<Pos> m_trajectory;
  
  // Parameters
  double
    m_gravity,
    m_lengths[2],
    m_masses[2],
    m_inertias[2],
    m_frictions[2],
    m_limits[2][2];
  
  // Internal pre-computed values
  double
    m_lengthsSq[2],
    m_massElbLSq4,
    m_massElbLLHalf,
    m_massElbLL;
  
  int m_integrator;
};

} // namespace dmx

#endif