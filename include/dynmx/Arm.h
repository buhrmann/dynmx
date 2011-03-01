/*
 *  Arm.h
 *  FastCTRNN
 *
 *  Created by Thomas Buhrmann on 16/01/2010.
 *  Copyright 2010 __MyCompanyName__. All rights reserved.
 *
 */

#ifndef _ARM2D_
#define _ARM2D_

#include <queue>

enum Joint
{
  JT_invalid = -1,
  JT_elbow = 0,
  JT_shoulder = 1,
  Jt_maxNumber
};

//----------------------------------------------------------------------------------------------------------------------
/// Helper for independence from other math libraries
//----------------------------------------------------------------------------------------------------------------------
struct Pos
{
  Pos() : x(0), y(0) {};
  Pos(float newX, float newY) : x(newX), y(newY) {};
  float x, y;
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
/// Simple controller
//----------------------------------------------------------------------------------------------------------------------
class PD
{
  
  enum NonLinearity
  {
    kLinear = 0,
    kExponential,
    kAsinh
  };
  
public:

  PD() : m_target(0.0f), m_targetPrev(0), m_P(10.0f), m_D(1.0f), m_posFunction(kLinear), m_velFunction(kLinear),
    m_numFramesDelay(4) {};
  
  float update(float target, float pos, float vel, float dt);
  
  std::queue<float> m_prevPositions;
  std::queue<float> m_prevVelocities;
  
  int 
    m_numFramesDelay,
    m_posFunction,
    m_velFunction;
  
  float
    m_target,
    m_targetPrev,
    m_P,
    m_D;
};

//----------------------------------------------------------------------------------------------------------------------
/// Implements equations of motion for a planar two-jointed arm
//----------------------------------------------------------------------------------------------------------------------
class Arm2d
{
  
public:
  
  Arm2d();
  ~Arm2d();
  
  void init(float elbAngle = 0.0f, float shdAngle = 0.0f);
  void setParameters(float mass1, float mass2, float length1, float length2, float inertia1, float inertia2);
  void setGravity(float g) { m_gravity = g; };
  
  void update(float torque1, float torque2, float timeStep);
  void updatePD(float angle1, float angle2, float timeStep); 
  void updatePosition(float x, float y, float timeStep); 
  void getPointOnUpperArm(float distanceFromJoint, float&x, float& y) const;
  void getPointOnLowerArm(float distanceFromJoint, float&x, float& y) const;  
  float getJointAngle(Joint joint) const { return m_angles[joint]; } ;
  float getLength(Joint joint) const { return m_lengths[joint]; } ;
  void getEffectorPos(float& x, float& y) const { x = m_effectorPos[0]; y = m_effectorPos[1];  };
  void getElbowPos(float& x, float& y) const { x = m_elbowPos[0]; y = m_elbowPos[1];  };
  void forwardKinematics();
  void inverseKinematics(float posX, float posY, float elbDir, float& elbAngle, float& shdAngle);
  
  PD m_pd[2];
  
protected:

  void preCompute();
  
  // states
  float 
    m_angles[2],
    m_velocities[2],
    m_accelerations[2],
    m_elbowPos[2],
    m_effectorPos[2];
  
  // parameters
  float
    m_gravity,
    m_lengths[2],
    m_masses[2],
    m_inertias[2],
    m_frictions[2],
    m_limits[2];
  
  // internal pre-computed values
  float
    m_lengthsSq[2],
    m_massElbLSq4,
    m_massElbLLHalf,
    m_massElbLL;
  
};

#endif