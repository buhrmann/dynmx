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
  Jt_maxNumber
};

//----------------------------------------------------------------------------------------------------------------------
/// Implements equations of motion for a planar two-jointed arm
//----------------------------------------------------------------------------------------------------------------------
class Arm : public Model
{
  
public:
  
  Arm();
  ~Arm();
  
  // Inherited from class Model
  virtual void init();
  virtual void reset() { reset(0.0, 0.0); };
  virtual void update(float dt) { update(dt, 0.0, 0.0); };
  
  void reset(float elbAngle, float shdAngle);  
  void update(float dt, double torque1, double torque2);
  
  // Setters
  void setParameters(float mass1, float mass2, float length1, float length2);
  void setFriction(float elbF, float shdF) { m_frictions[JT_elbow] = elbF; m_frictions[JT_shoulder] = shdF; };
  void setLimit(Joint jt, float upper, float lower) { assert(jt >=0 && jt < 2); m_limits[jt][0] = upper; m_limits[jt][1] = lower; };
  void setGravity(float g) { m_gravity = g; };
  void setJointLocked(Joint j, bool l) { m_jointLocked[j] = l; };
  void setTarget(Pos p) { m_target = p; };    
  
  // Getters
  double getGravity() const { return m_gravity; };
  Pos getPointOnUpperArm(float distanceFromJoint) const;
  Pos getPointOnLowerArm(float distanceFromJoint) const;  
  double getJointAngle(Joint joint) { return m_angles[joint]; };
  double getJointVelocity(Joint joint) { return m_velocities[joint]; };
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

  // Store output in human readable format
  virtual void toXml(ci::XmlTree& xml);
  
protected:

  void preCompute();
  void solveEuler(const double* torques, float dt);
  void solveImprovedEuler(const double* torques, float dt);
  void computeAccelerations(const double* angles, const double* velocities, const double* accel, const double* torques,
                            double* newAccel);
  
  bool m_jointLocked[2];
  
  // states
  double 
    m_angles[2],
    m_velocities[2],
    m_accelerations[2];
  
  Pos
    m_elbowPos,
    m_effectorPos,
    m_target;
  
  std::deque<Pos> m_trajectory;
  
  // parameters
  double
    m_gravity,
    m_lengths[2],
    m_masses[2],
    m_inertias[2],
    m_frictions[2],
    m_limits[2][2];
  
  // internal pre-computed values
  double
    m_lengthsSq[2],
    m_massElbLSq4,
    m_massElbLLHalf,
    m_massElbLL;
};

} // namespace dmx

#endif