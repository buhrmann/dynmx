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

namespace dmx
{

typedef ci::Vec2f Pos;

static const int MaxTrajPoints = 100;

enum Joint
{
  JT_invalid = -1,
  JT_elbow = 0,
  JT_shoulder = 1,
  Jt_maxNumber
};

//----------------------------------------------------------------------------------------------------------------------
/// Smoothest motion over given duration (2d)
//----------------------------------------------------------------------------------------------------------------------
struct MinJerkTrajectory
{
  MinJerkTrajectory() : duration(1.0f), time(0.0f) {};
  void setNew(const Pos& initPos, const Pos& finalPos, float duration); 
  void update(float dt); 
  
  Pos 
    current,
    initial,
    target;
  float duration;
  float time;
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
  
  void setParameters(float mass1, float mass2, float length1, float length2);
  void setFriction(float elbF, float shdF) { m_frictions[JT_elbow] = elbF; m_frictions[JT_shoulder] = shdF; };
  void setLimit(Joint jt, float upper, float lower) { assert(jt >=0 && jt < 2); m_limits[jt][0] = upper; m_limits[jt][1] = lower; };
  void setGravity(float g) { m_gravity = g; };
  
  Pos getPointOnUpperArm(float distanceFromJoint) const;
  Pos getPointOnLowerArm(float distanceFromJoint) const;  
  double getJointAngle(Joint joint) { return m_angles[joint]; };
  float getLength(Joint joint) const { return m_lengths[joint]; } ;
  const Pos& getEffectorPos() const { return m_effectorPos; };
  Pos& getEffectorPos() { return m_effectorPos; };
  const Pos& getElbowPos() const { return m_elbowPos; };
  Pos& getElbowPos() { return m_elbowPos; };
  void forwardKinematics();
  void forwardKinematics(double elbAngle, double shdAngle, Pos& elbPos, Pos& effPos);
  void inverseKinematics(const Pos& pos, float elbDir, double& elbAngle, double& shdAngle);  
  const std::vector<Pos>& getTrajectory() { return m_trajectory; };
  
protected:

  void preCompute();
  
  // states
  double 
    m_angles[2],
    m_velocities[2],
    m_accelerations[2];
  
  Pos
    m_elbowPos,
    m_effectorPos;
  
  std::vector<Pos> m_trajectory;
  
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