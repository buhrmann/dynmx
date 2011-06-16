/*
 *  Arm.cpp
 *  FastCTRNN
 *
 *  Created by Thomas Buhrmann on 16/01/2010.
 *  Copyright 2010 __MyCompanyName__. All rights reserved.
 *
 */

#include "Arm.h"
#include "MathUtils.h"

#ifndef PI
#define PI 3.14159265
#define PI_OVER_TWO 1.5707963
#define PI_OVER_FOUR 0.7853981
#endif

// Helper needed, as function is not implemented on windows
//----------------------------------------------------------------------------------------------------------------------


//----------------------------------------------------------------------------------------------------------------------
// Implementation of MinJerkTrajectory
//----------------------------------------------------------------------------------------------------------------------
void MinJerkTrajectory::update(float dt)
{
  time += dt;
  float t = time / duration;
  t = t > duration ? duration : t;
  float amplitude = 15*t*t*t*t - 6*t*t*t*t*t - 10*t*t*t;
  current.x = initial.x + (initial.x - target.x) * amplitude;
  current.y = initial.y + (initial.y - target.y) * amplitude;
}

//----------------------------------------------------------------------------------------------------------------------
void MinJerkTrajectory::setNew(const Pos& initPos, const Pos& finalPos, float duration)
{
  initial = initPos;
  target = finalPos;
  duration = duration;
  time = 0.0f;
}

//----------------------------------------------------------------------------------------------------------------------
// Implementation of PD
//----------------------------------------------------------------------------------------------------------------------
float PD::update(float target, float pos, float vel, float dt)
{
  if(m_numFramesDelay > 0)
  {
    // push new state on to delay line (pushes to the back)
    m_prevPositions.push(pos);
    m_prevVelocities.push(vel); 
    
    // if we haven't got enought frames yet, do nothing
    if(m_prevPositions.size() < m_numFramesDelay)
    {
      return 0.0f;
    }

    // otherwise we've had at least the required number of delay frames, so use the stored information
    pos = m_prevPositions.front(); 
    vel = m_prevVelocities.front();
    // remove the oldest information from queue (pops from front)
    m_prevPositions.pop();
    m_prevVelocities.pop();
  }
  
  m_target = target;
  
  // calculate target velocity here if needed
  float targetVel = (m_target - m_targetPrev) / dt;
  m_targetPrev = m_target;
  
  // position term
  float posErr = m_target - pos; 
  if(m_posFunction == kExponential)
  {
    posErr = pos > m_target ? - exp(-posErr) - 1.0f : exp(posErr) - 1.0f;
  }
  
  // velocity term
  float velErr = targetVel - vel;
  if(m_velFunction == kAsinh)
  {
    velErr = asinh(velErr);
  }

  return m_P * posErr + m_D * velErr;
}

//----------------------------------------------------------------------------------------------------------------------
// Typical dimensions from Arnon(1990), referenced in 
// Karniel and Inbar (1997), "A model for learning human reaching movements"
//----------------------------------------------------------------------------------------------------------------------
Arm2d::Arm2d()
{
  //init();
}

//----------------------------------------------------------------------------------------------------------------------
Arm2d::~Arm2d()
{
  
}

//----------------------------------------------------------------------------------------------------------------------
void Arm2d::init(float elbAngle, float shdAngle)
{
  m_angles[JT_elbow] = elbAngle;
  m_angles[JT_shoulder] = shdAngle;
  
  for(int i = 0; i < 2; i++)
  {
    m_anglesDes[i] = 0.0f;
    m_velocities[i] = 0.0f;
    m_accelerations[i] = 0.0f;
  }
  
  // setup limits
  m_limits[JT_elbow] = m_limits[JT_shoulder] = PI * 0.75f;
  
  forwardKinematics();
}

// takes proportional distance from joint along bone d
//----------------------------------------------------------------------------------------------------------------------
Pos Arm2d::getPointOnUpperArm(float d) const
{
  return Pos(d * m_elbowPos.x, d * m_elbowPos.y); 
}

// takes proportional distance from joint along bone d
//----------------------------------------------------------------------------------------------------------------------
Pos Arm2d::getPointOnLowerArm(float d) const
{
  return Pos(
    m_elbowPos.x + d * (m_effectorPos.x - m_elbowPos.x), 
    m_elbowPos.y + d * (m_effectorPos.y - m_elbowPos.y)
  );
}

//----------------------------------------------------------------------------------------------------------------------
void Arm2d::forwardKinematics()
{
  forwardKinematics(m_angles[JT_elbow], m_angles[JT_shoulder], m_elbowPos, m_effectorPos);
}

// Make externally useful
//----------------------------------------------------------------------------------------------------------------------
void Arm2d::forwardKinematics(float elbAngle, float shdAngle, Pos& elbPos, Pos& effPos)
{
  elbPos.x = m_lengths[JT_shoulder] * cos(shdAngle);
  elbPos.y = m_lengths[JT_shoulder] * sin(shdAngle);

  effPos.x = elbPos.x + (m_lengths[JT_elbow] * cos(elbAngle + shdAngle));
  effPos.y = elbPos.y + (m_lengths[JT_elbow] * sin(elbAngle + shdAngle));
}

//----------------------------------------------------------------------------------------------------------------------
void Arm2d::inverseKinematics(const Pos& pos, float elbDir, float& elbAngle, float& shdAngle)
{
  float eAngle = pos.x*pos.x + pos.y*pos.y - m_lengthsSq[JT_elbow] - m_lengthsSq[JT_shoulder];
  eAngle /= 2 * m_lengths[JT_elbow] * m_lengths[JT_shoulder];
  // clamp to [-1, 1]
  eAngle = eAngle < -1 ? -1 : eAngle > 1 ? 1 : eAngle;
  elbAngle = elbDir * fabs(acosf(eAngle));
  
  float t1 = m_lengths[JT_shoulder] +  m_lengths[JT_elbow] * cos(elbAngle);
  float t2 = m_lengths[JT_elbow] * sin(elbAngle);
  float xx = pos.x * t1 + pos.y * t2;
  float yy = pos.y * t1 - pos.x * t2;
  shdAngle = atan2(yy, xx);
}

//----------------------------------------------------------------------------------------------------------------------
void Arm2d::preCompute()
{
  m_lengthsSq[JT_elbow] = m_lengths[JT_elbow] * m_lengths[JT_elbow];
  m_lengthsSq[JT_shoulder] = m_lengths[JT_shoulder] * m_lengths[JT_shoulder];  
  m_massElbLL = m_masses[JT_elbow] * m_lengths[JT_elbow] * m_lengths[JT_shoulder]; 
  m_massElbLLHalf = m_massElbLL / 2.0f;
  m_massElbLSq4 = (m_masses[JT_elbow] * m_lengthsSq[JT_elbow]) / 4.0f;
}

//----------------------------------------------------------------------------------------------------------------------
void Arm2d::setParameters(float mass1, float mass2, float length1, float length2, float inertia1, float inertia2)
{
  m_masses[JT_elbow] = mass1;
  m_masses[JT_shoulder] = mass2;
  m_lengths[JT_elbow] = length1;
  m_lengths[JT_shoulder] = length2;
  m_inertias[JT_elbow] = inertia1;
  m_inertias[JT_shoulder] = inertia2;
  m_frictions[JT_elbow] = 0.12f;
  m_frictions[JT_shoulder] = 0.12f;
  m_gravity = 9.81f;
  preCompute();
}

//----------------------------------------------------------------------------------------------------------------------
void Arm2d::updatePosition(float x, float y, float timeStep)
{
  // check not out of range
  float reachDist = m_lengths[0] + m_lengths[1];
  float magSq = (x * x) + (y * y);
  if(magSq > reachDist*reachDist)
  {
    float mag = sqrtf(magSq);
    x = x / mag * (reachDist - 0.0001f);
    y = y / mag * (reachDist - 0.0001f);
  }
  
  inverseKinematics(Pos(x, y), 1.0f, m_anglesDes[JT_elbow], m_anglesDes[JT_shoulder]);
  updatePD(m_anglesDes[JT_elbow], m_anglesDes[JT_shoulder], timeStep);
}

//----------------------------------------------------------------------------------------------------------------------
void Arm2d::updatePD(float angle1, float angle2, float dt)
{
  float elbTorque = m_pd[JT_elbow].update(angle1, m_angles[JT_elbow], m_velocities[JT_elbow], dt);
  float shdTorque = m_pd[JT_shoulder].update(angle2, m_angles[JT_shoulder], m_velocities[JT_shoulder], dt);
  update(elbTorque*100*dt, shdTorque*100*dt, dt);
}

//----------------------------------------------------------------------------------------------------------------------
void Arm2d::update(float torqElb, float torqShd, float timeStep)
{
  // calculate these only once:
  float cosElbAngle = cosf(m_angles[JT_elbow]);
  float sinElbAngle = sinf(m_angles[JT_elbow]);
  float sinElbShd = sinf(m_angles[JT_elbow] + m_angles[JT_shoulder]);
  
  // computer elbow acceleration
  float coriolis = m_massElbLLHalf * m_velocities[JT_shoulder] * m_velocities[JT_shoulder] * sinElbAngle;
  float normalInertia = m_inertias[JT_elbow] + m_massElbLSq4;
  float interInertia = m_accelerations[JT_shoulder] * (m_inertias[JT_elbow] + m_massElbLLHalf * cosElbAngle + m_massElbLSq4); 
  float gravity = m_gravity * m_masses[JT_elbow] * m_lengths[JT_elbow] * sinElbShd;
  float damping = m_frictions[JT_elbow] * m_velocities[JT_elbow];
  float elbAcceleration = (torqElb - interInertia - coriolis - gravity - damping) / normalInertia;
  
  // computer shoulder acceleration
  coriolis = - m_massElbLLHalf * m_velocities[JT_elbow] * m_velocities[JT_elbow] * sinElbAngle
              - m_massElbLL * m_velocities[JT_elbow] * m_velocities[JT_shoulder] * sinElbAngle;
  
  normalInertia = (m_inertias[JT_elbow] + m_inertias[JT_shoulder] 
                   + m_massElbLL * cosElbAngle 
                   + (m_masses[JT_shoulder] * m_lengthsSq[JT_shoulder]) / 4.0f
                   + m_massElbLSq4 
                   + (m_masses[JT_elbow] * m_lengthsSq[JT_shoulder]) );
  
  interInertia = m_accelerations[JT_elbow] * (m_inertias[JT_elbow] 
                                              + m_massElbLSq4 
                                              + (m_massElbLLHalf * cosElbAngle));
  
  // shoulder gravity is elbow gravity + shoulder
  gravity += m_gravity * (m_masses[JT_elbow] + m_masses[JT_shoulder]) * m_lengths[JT_shoulder] * sinf(m_angles[JT_shoulder]);
  damping = m_frictions[JT_shoulder] * m_velocities[JT_shoulder];
  float shdAcceleration = (torqShd - interInertia - coriolis - gravity - damping) / normalInertia;
  
  // synchronous update
  m_accelerations[JT_elbow] = elbAcceleration;
  m_accelerations[JT_shoulder] = shdAcceleration; 
  
  // integrate to get velocities and positions
  m_velocities[JT_elbow] += m_accelerations[JT_elbow] * timeStep;
  m_angles[JT_elbow] += m_velocities[JT_elbow] * timeStep;
  
  m_velocities[JT_shoulder] += m_accelerations[JT_shoulder] * timeStep;
  m_angles[ JT_shoulder] += m_velocities[JT_shoulder] * timeStep;
  
  // limit clamping
  if(fabs(m_angles[JT_elbow]) > m_limits[JT_elbow])
  {
    if(m_angles[JT_elbow] > 0)
      m_angles[JT_elbow] = m_limits[JT_elbow];
    else
      m_angles[JT_elbow] = -m_limits[JT_elbow];
      
    m_velocities[JT_elbow] = 0.0f;
    m_accelerations[JT_elbow] = 0.0f;
  }
  
  if(fabs(m_angles[JT_shoulder]) > m_limits[JT_shoulder])
  {
    if(m_angles[JT_shoulder] > 0)
      m_angles[JT_shoulder] = m_limits[JT_shoulder];
    else
      m_angles[JT_shoulder] = -m_limits[JT_shoulder];
      
    m_velocities[JT_shoulder] = 0.0f;
    m_accelerations[JT_shoulder] = 0.0f;
  }
  
  
  // get new end effector position
  forwardKinematics();
  
  // store trajectory
  m_trajectory.push_back(m_effectorPos);
  if(m_trajectory.size() > MaxTrajPoints)
  {
    m_trajectory.erase(m_trajectory.begin());
  }
  
#define CHECK_ROUND_TRIP 0
#if CHECK_ROUND_TRIP
  float elbAngle, shdAngle;
  float elbDir = m_angles[JT_elbow] > 0 ? 1 : -1;
  inverseKinematics(m_effectorPos[0], m_effectorPos[1], elbDir, elbAngle, shdAngle);
#endif
}

