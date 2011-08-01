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

namespace dmx
{

#ifndef PI
#define PI 3.14159265
#define PI_OVER_TWO 1.5707963
#define PI_OVER_FOUR 0.7853981
#endif

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
// Typical dimensions from Arnon(1990), referenced in 
// Karniel and Inbar (1997), "A model for learning human reaching movements", and (1996).
// Also see Gribble (1998), or Uno and Kawato (1989).
//----------------------------------------------------------------------------------------------------------------------
Arm::Arm()
{
}

//----------------------------------------------------------------------------------------------------------------------
Arm::~Arm()
{
  
}

//----------------------------------------------------------------------------------------------------------------------  
void Arm::init()
{  
  m_gravity = 9.81;
  
  const ci::XmlTree* settings = SETTINGS;
  if (settings->hasChild("Config/Arm"))
  {
    // Use setting from globals file
    const ci::XmlTree& xml = settings->getChild("Config/Arm");
    double upperArmLength = xml.getChild("UpperArm").getAttributeValue<double>("Length");
    double lowerArmLength = xml.getChild("LowerArm").getAttributeValue<double>("Length");
    double upperArmMass = xml.getChild("UpperArm").getAttributeValue<double>("Mass");
    double lowerArmMass = xml.getChild("LowerArm").getAttributeValue<double>("Mass");
    setParameters(lowerArmMass, upperArmMass, lowerArmLength, upperArmLength);
    
    double shoulderFriction = xml.getChild("Shoulder").getAttributeValue<double>("Friction");    
    double elbowFriction = xml.getChild("Elbow").getAttributeValue<double>("Friction");
    setFriction(elbowFriction, shoulderFriction);
    
    double shoulderUpperLimit = PI * xml.getChild("Shoulder").getAttributeValue<double>("UpperLimit");
    double shoulderLowerLimit = PI * xml.getChild("Shoulder").getAttributeValue<double>("LowerLimit");
    setLimit(JT_shoulder, shoulderUpperLimit, shoulderLowerLimit);    
    
    double elbowUpperLimit = PI * xml.getChild("Elbow").getAttributeValue<double>("UpperLimit");
    double elbowLowerLimit = PI * xml.getChild("Elbow").getAttributeValue<double>("LowerLimit");
    setLimit(JT_elbow, elbowUpperLimit, elbowLowerLimit);
  }
  else
  {
  
#define ARM_USE_KARNIEL_PARAMS 1
#if ARM_USE_KARNIEL_PARAMS
    // Parameters from Karniel and Inbar
    const float lowerArmMass = 1.3f;
    const float upperArmMass = 2.52f;
    const float lowerArmLength = 0.32f;
    const float upperArmLength = 0.33f;
#else
    // Parameters from Gribble
    const float lowerArmMass = 1.65f;
    const float upperArmMass = 2.1f;
    const float lowerArmLength = 0.34f;
    const float upperArmLength = 0.46f;    
#endif
    
    setParameters(lowerArmMass, upperArmMass, lowerArmLength, upperArmLength);
    
    // Todo: should be possible to make 0, but currently unstable
    m_frictions[0] = m_frictions[1] = 0.12;

    // setup limits
    const float limit = PI * 0.75f;
    m_limits[JT_shoulder][0] = limit;
    m_limits[JT_shoulder][1] = -limit;  
    m_limits[JT_elbow][0] = limit;
    m_limits[JT_elbow][1] = -limit; //-0.01f;    
  }    
    
  reset(0,0);
}  

//----------------------------------------------------------------------------------------------------------------------
void Arm::reset(float elbAngle, float shdAngle)
{
  m_angles[JT_elbow] = elbAngle;
  m_angles[JT_shoulder] = shdAngle;
  
  for(int i = 0; i < 2; i++)
  {
    m_velocities[i] = 0.0;
    m_accelerations[i] = 0.0;
  }
  
  forwardKinematics();
}

// takes proportional distance from joint along bone d
//----------------------------------------------------------------------------------------------------------------------
Pos Arm::getPointOnUpperArm(float d) const
{
  return Pos(d * m_elbowPos.x, d * m_elbowPos.y); 
}

// takes proportional distance from joint along bone d
//----------------------------------------------------------------------------------------------------------------------
Pos Arm::getPointOnLowerArm(float d) const
{
  return Pos(
    m_elbowPos.x + d * (m_effectorPos.x - m_elbowPos.x), 
    m_elbowPos.y + d * (m_effectorPos.y - m_elbowPos.y)
  );
}

//----------------------------------------------------------------------------------------------------------------------
void Arm::forwardKinematics()
{
  forwardKinematics(m_angles[JT_elbow], m_angles[JT_shoulder], m_elbowPos, m_effectorPos);
}

// Make externally useful
//----------------------------------------------------------------------------------------------------------------------
void Arm::forwardKinematics(double elbAngle, double shdAngle, Pos& elbPos, Pos& effPos)
{
  elbPos.x = m_lengths[JT_shoulder] * cos(shdAngle);
  elbPos.y = m_lengths[JT_shoulder] * sin(shdAngle);

  effPos.x = elbPos.x + (m_lengths[JT_elbow] * cos(elbAngle + shdAngle));
  effPos.y = elbPos.y + (m_lengths[JT_elbow] * sin(elbAngle + shdAngle));
}

//----------------------------------------------------------------------------------------------------------------------
void Arm::inverseKinematics(const Pos& pos, float elbDir, double& elbAngle, double& shdAngle)
{
  float eAngle = pos.x*pos.x + pos.y*pos.y - m_lengthsSq[JT_elbow] - m_lengthsSq[JT_shoulder];
  eAngle /= 2.0 * m_lengths[JT_elbow] * m_lengths[JT_shoulder];
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
void Arm::preCompute()
{
  m_lengthsSq[JT_elbow] = m_lengths[JT_elbow] * m_lengths[JT_elbow];
  m_lengthsSq[JT_shoulder] = m_lengths[JT_shoulder] * m_lengths[JT_shoulder];  
  m_massElbLL = m_masses[JT_elbow] * m_lengths[JT_elbow] * m_lengths[JT_shoulder]; 
  m_massElbLLHalf = m_massElbLL / 2.0;
  m_massElbLSq4 = (m_masses[JT_elbow] * m_lengthsSq[JT_elbow]) / 4.0;
}

//----------------------------------------------------------------------------------------------------------------------
void Arm::setParameters(float mass1, float mass2, float length1, float length2)
{
  m_masses[JT_elbow] = mass1;
  m_masses[JT_shoulder] = mass2;
  m_lengths[JT_elbow] = length1;
  m_lengths[JT_shoulder] = length2;
  m_inertias[JT_elbow] = (m_masses[JT_elbow] * sqr(m_lengths[JT_elbow])) / 12.0;
  m_inertias[JT_shoulder] = (m_masses[JT_shoulder] * sqr(m_lengths[JT_shoulder])) / 12.0;
  preCompute();
}

// Equations from Hollerbach and Flash 1981.   
//----------------------------------------------------------------------------------------------------------------------
void Arm::update(float timeStep, double torqElb, double torqShd)
{
  // calculate these only once:
  double cosElbAngle = cosf(m_angles[JT_elbow]);
  double sinElbAngle = sinf(m_angles[JT_elbow]);
  double sinElbShd = sinf(m_angles[JT_elbow] + m_angles[JT_shoulder]);
  
  // compute elbow acceleration
  double coriolis = m_massElbLLHalf * sqr(m_velocities[JT_shoulder]) * sinElbAngle;
  double normalInertia = /*m_acceleration[JT_elbow] **/ (m_inertias[JT_elbow] + m_massElbLSq4);
  double interInertia = m_accelerations[JT_shoulder] * (m_inertias[JT_elbow] + m_massElbLLHalf * cosElbAngle + m_massElbLSq4); 
  double gravity = m_gravity * m_masses[JT_elbow] * m_lengths[JT_elbow] * sinElbShd;
  double damping = m_frictions[JT_elbow] * m_velocities[JT_elbow];
  double elbAcceleration = (torqElb - interInertia - coriolis - gravity - damping) / normalInertia;
  
  // compute shoulder acceleration
  coriolis = - m_massElbLLHalf * sqr(m_velocities[JT_elbow]) * sinElbAngle
              - m_massElbLL * m_velocities[JT_elbow] * m_velocities[JT_shoulder] * sinElbAngle;
  
  normalInertia = /*m_accelerations[JT_shoulder] **/ (m_inertias[JT_elbow] + m_inertias[JT_shoulder] 
                   + m_massElbLL * cosElbAngle 
                   + (m_masses[JT_shoulder] * m_lengthsSq[JT_shoulder]) / 4.0f
                   + m_massElbLSq4 
                   + (m_masses[JT_elbow] * m_lengthsSq[JT_shoulder]) );
  
  interInertia = m_accelerations[JT_elbow] * (m_inertias[JT_elbow] + m_massElbLSq4 + (m_massElbLLHalf * cosElbAngle));
  
  // shoulder gravity is elbow gravity + shoulder
  gravity += m_gravity * (m_masses[JT_elbow] + m_masses[JT_shoulder]) * m_lengths[JT_shoulder] * sinf(m_angles[JT_shoulder]);
  damping = m_frictions[JT_shoulder] * m_velocities[JT_shoulder];
  double shdAcceleration = (torqShd - interInertia - coriolis - gravity - damping) / normalInertia;
  
  // synchronous update
  m_accelerations[JT_elbow] = elbAcceleration;
  m_accelerations[JT_shoulder] = shdAcceleration; 
  
  // integrate to get velocities and positions
  m_velocities[JT_elbow] += m_accelerations[JT_elbow] * timeStep;
  m_angles[JT_elbow] += m_velocities[JT_elbow] * timeStep;
  
  m_velocities[JT_shoulder] += m_accelerations[JT_shoulder] * timeStep;
  m_angles[ JT_shoulder] += m_velocities[JT_shoulder] * timeStep;
  
  // limit clamping
  if(m_angles[JT_elbow] > m_limits[JT_elbow][0])
  {
    m_angles[JT_elbow] = m_limits[JT_elbow][0];
    m_velocities[JT_elbow] = 0.0;
    m_accelerations[JT_elbow] = 0.0;    
  }
  else if (m_angles[JT_elbow] < m_limits[JT_elbow][1])
  {
    m_angles[JT_elbow] = m_limits[JT_elbow][1];
    m_velocities[JT_elbow] = 0.0;
    m_accelerations[JT_elbow] = 0.0;    
  }
  
  if(m_angles[JT_shoulder] > m_limits[JT_shoulder][0])
  {
    m_angles[JT_shoulder] = m_limits[JT_shoulder][0];
    m_velocities[JT_shoulder] = 0.0;
    m_accelerations[JT_shoulder] = 0.0;    
  }
  else if (m_angles[JT_shoulder] < m_limits[JT_shoulder][1])
  {
    m_angles[JT_shoulder] = m_limits[JT_shoulder][1];
    m_velocities[JT_shoulder] = 0.0;
    m_accelerations[JT_shoulder] = 0.0;    
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

} // namespace dmx
