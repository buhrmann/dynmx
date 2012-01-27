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
// Equations from Hollerbach and Flash 1981. Also see Shadmehr 1990 or Uno et al 1989.
// Typical dimensions from Arnon(1990), referenced in Karniel and Inbar (1997), 
// "A model for learning human reaching movements", and (1996).
// Also see Gribble (1998), or Uno and Kawato (1989).
//----------------------------------------------------------------------------------------------------------------------
Arm::Arm(){ }

//----------------------------------------------------------------------------------------------------------------------
Arm::~Arm(){ }

//----------------------------------------------------------------------------------------------------------------------  
void Arm::init()
{  
  m_integrator = kInteg_heun; 
  
  m_gravity = 9.81;
  
  const ci::XmlTree* settings = SETTINGS;
  if (settings->hasChild("Config/Arm"))
  {
    // Use setting from globals file
    const ci::XmlTree& xml = settings->getChild("Config/Arm");
    
    if(xml.hasAttribute("Gravity"))
    {
      m_gravity = xml.getAttributeValue<double>("Gravity");
    }
    
    if(xml.hasAttribute("Integrator"))
    {
      const std::string& integratorName = xml.getAttributeValue<std::string>("Integrator");
      if(integratorName == "heun")
      {
        m_integrator = kInteg_heun;
      }
      else if (integratorName == "euler")
      {
        m_integrator = kInteg_euler;
      }
    }    
    
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
    
    bool locked[2] = {false, false};
    if(xml.getChild("Elbow").hasAttribute("Locked"))
    {
      locked[JT_elbow] = xml.getChild("Elbow").getAttributeValue<bool>("Locked");
    }
    if(xml.getChild("Shoulder").hasAttribute("Locked"))
    {
      locked[JT_shoulder] = xml.getChild("Shoulder").getAttributeValue<bool>("Locked");
    }
    setJointLocked(JT_elbow, locked[JT_elbow]);
    setJointLocked(JT_shoulder, locked[JT_shoulder]);
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
    m_limits[JT_elbow][1] = /*-limit; //*/-0.01f;    
    
    m_jointLocked[0] = m_jointLocked[1] = false;
  }    
    
  reset(0,0);
}  

//----------------------------------------------------------------------------------------------------------------------
void Arm::reset(float elbAngle, float shdAngle)
{
  m_state.angles[JT_elbow] = elbAngle;
  m_state.angles[JT_shoulder] = shdAngle;  
  
  for(int i = 0; i < 2; i++)
  {  
    m_state.velocities[i] = 0.0;
    m_state.accelerations[i] = 0.0;
    m_state.coriolisAcc[i] = 0.0;
    m_state.gravityAcc[i] = 0.0;
    m_state.inertiaAcc[i] = 0.0;
    m_state.interactionAcc[i] = 0.0;
    m_state.dampingAcc[i] = 0.0;
    m_state.torques[i] = 0.0;    
  }
  
  forwardKinematics();
  
  m_trajectory.clear();
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
  forwardKinematics(m_state.angles[JT_elbow], m_state.angles[JT_shoulder], m_elbowPos, m_effectorPos);
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

//----------------------------------------------------------------------------------------------------------------------  
void Arm::solveEuler(const double* torques, float dt)
{  
  computeAccelerations(m_state);  

  // integrate to get velocities and positions
  m_state.velocities[JT_elbow] += m_state.accelerations[JT_elbow] * dt;
  m_state.angles[JT_elbow] += m_state.velocities[JT_elbow] * dt;
  
  m_state.velocities[JT_shoulder] += m_state.accelerations[JT_shoulder] * dt;
  m_state.angles[ JT_shoulder] += m_state.velocities[JT_shoulder] * dt;    
}
  
// With predictive correction of Euler error
//----------------------------------------------------------------------------------------------------------------------  
void Arm::solveImprovedEuler(float timeStep)
{
  // Todo: and fix to 0.5 below for little optimisation
  const double w1 = 0.5;
  const double w2 = 1.0 - w1;
  
  float dt = timeStep;
  
  // 1. Use Euler to create predicted state
  computeAccelerations(m_state);
  State nextState = m_state;
  nextState.angles[JT_elbow] = m_state.angles[JT_elbow] + m_state.velocities[JT_elbow] * dt;  
  nextState.velocities[JT_elbow] = m_state.velocities[JT_elbow] + m_state.accelerations[JT_elbow] * dt;
  nextState.angles[JT_shoulder] = m_state.angles[JT_shoulder] + m_state.velocities[JT_shoulder] * dt;    
  nextState.velocities[JT_shoulder] = m_state.velocities[JT_shoulder] + m_state.accelerations[JT_shoulder] * dt;
  
  // 2. Compute predicted acceleration at the next time step
  computeAccelerations(nextState);
  
  // 3. Integrate for real now, using average of current and predicted acceleration
  m_state.angles[JT_elbow]     +=  (w1 * m_state.velocities[JT_elbow] + w2 * nextState.velocities[JT_elbow]) * dt;
  m_state.angles[JT_shoulder]  +=  (w1 * m_state.velocities[JT_shoulder] + w2 * nextState.velocities[JT_shoulder]) * dt;      
  
  m_state.velocities[JT_elbow]    += (w1 * m_state.accelerations[JT_elbow] + w2 * nextState.accelerations[JT_elbow]) * dt;
  m_state.velocities[JT_shoulder] += (w1 * m_state.accelerations[JT_shoulder] + w2 * nextState.accelerations[JT_shoulder]) * dt;  
  
  // Update rest of variables whose average lead to this integration
  m_state.accelerations[JT_elbow]    = (w1 * m_state.accelerations[JT_elbow]    + w2 * nextState.accelerations[JT_elbow] );
  m_state.accelerations[JT_shoulder] = (w1 * m_state.accelerations[JT_shoulder] + w2 * nextState.accelerations[JT_shoulder] );  
  m_state.coriolisAcc[JT_elbow]    = (w1 * m_state.coriolisAcc[JT_elbow]    + w2 * nextState.coriolisAcc[JT_elbow] );
  m_state.coriolisAcc[JT_shoulder] = (w1 * m_state.coriolisAcc[JT_shoulder] + w2 * nextState.coriolisAcc[JT_shoulder] );  
  m_state.gravityAcc[JT_elbow]    = (w1 * m_state.gravityAcc[JT_elbow]    + w2 * nextState.gravityAcc[JT_elbow] );
  m_state.gravityAcc[JT_shoulder] = (w1 * m_state.gravityAcc[JT_shoulder] + w2 * nextState.gravityAcc[JT_shoulder] );  
  m_state.inertiaAcc[JT_elbow]    = (w1 * m_state.inertiaAcc[JT_elbow]    + w2 * nextState.inertiaAcc[JT_elbow] );
  m_state.inertiaAcc[JT_shoulder] = (w1 * m_state.inertiaAcc[JT_shoulder] + w2 * nextState.inertiaAcc[JT_shoulder] );  
  m_state.interactionAcc[JT_elbow]    = (w1 * m_state.interactionAcc[JT_elbow]    + w2 * nextState.interactionAcc[JT_elbow] );
  m_state.interactionAcc[JT_shoulder] = (w1 * m_state.interactionAcc[JT_shoulder] + w2 * nextState.interactionAcc[JT_shoulder] );  
  m_state.dampingAcc[JT_elbow]    = (w1 * m_state.dampingAcc[JT_elbow]    + w2 * nextState.dampingAcc[JT_elbow] );
  m_state.dampingAcc[JT_shoulder] = (w1 * m_state.dampingAcc[JT_shoulder] + w2 * nextState.dampingAcc[JT_shoulder] );      
}
  
//----------------------------------------------------------------------------------------------------------------------
void Arm::update(float timeStep, double torqElb, double torqShd)  
{  
  m_state.torques[JT_elbow] = torqElb;
  m_state.torques[JT_shoulder] = torqShd;
  
  if(m_integrator == kInteg_heun)
  {
    solveImprovedEuler(timeStep);
  }
  else
  {
    solveEuler(m_state.torques, timeStep);
  }
  
  // Limit clamping  
  if(m_state.angles[JT_elbow] > m_limits[JT_elbow][0])
  {
    m_state.angles[JT_elbow] = m_limits[JT_elbow][0];
    m_state.velocities[JT_elbow] = 0.0;
    m_state.accelerations[JT_elbow] = 0.0;    
  }
  else if (m_state.angles[JT_elbow] < m_limits[JT_elbow][1])
  {
    m_state.angles[JT_elbow] = m_limits[JT_elbow][1];
    m_state.velocities[JT_elbow] = 0.0;
    m_state.accelerations[JT_elbow] = 0.0;    
  }
  
  if(m_state.angles[JT_shoulder] > m_limits[JT_shoulder][0])
  {
    m_state.angles[JT_shoulder] = m_limits[JT_shoulder][0];
    m_state.velocities[JT_shoulder] = 0.0;
    m_state.accelerations[JT_shoulder] = 0.0;    
  }
  else if (m_state.angles[JT_shoulder] < m_limits[JT_shoulder][1])
  {
    m_state.angles[JT_shoulder] = m_limits[JT_shoulder][1];
    m_state.velocities[JT_shoulder] = 0.0;
    m_state.accelerations[JT_shoulder] = 0.0;    
  }  
  // get new end effector position
  forwardKinematics();
  
  // store trajectory
  m_trajectory.push_back(m_effectorPos);
  if(m_trajectory.size() >= MaxTrajPoints)
  {
    m_trajectory.pop_front();
  }
  
#define CHECK_ROUND_TRIP 0
#if CHECK_ROUND_TRIP
  float elbAngle, shdAngle;
  float elbDir = m_angles[JT_elbow] > 0 ? 1 : -1;
  inverseKinematics(m_effectorPos[0], m_effectorPos[1], elbDir, elbAngle, shdAngle);
#endif
}

// Equations from Hollerbach and Flash 1981.     
//----------------------------------------------------------------------------------------------------------------------
void Arm::computeAccelerations(State& state)
{
  // calculate these only once:
  double cosElbAngle = cosf(state.angles[JT_elbow]);
  double sinElbAngle = sinf(state.angles[JT_elbow]);
  double sinElbShd = sinf(state.angles[JT_elbow] + state. angles[JT_shoulder]);
  
  // compute elbow acceleration
  double elbAcceleration = 0.0;
  state.gravityAcc[JT_elbow] = m_gravity * m_masses[JT_elbow] * m_lengths[JT_elbow] * sinElbShd;
  if(!m_jointLocked[JT_elbow]) 
  {
    state.coriolisAcc[JT_elbow] = m_massElbLLHalf * sqr(state.velocities[JT_shoulder]) * sinElbAngle;
    state.inertiaAcc[JT_elbow] = m_inertias[JT_elbow] + m_massElbLSq4;
    state.interactionAcc[JT_elbow] = state.accelerations[JT_shoulder] * (m_inertias[JT_elbow] + m_massElbLSq4 + (m_massElbLLHalf * cosElbAngle));     
    state.dampingAcc[JT_elbow] = m_frictions[JT_elbow] * state.velocities[JT_elbow];
    elbAcceleration = (state.torques[JT_elbow] - state.interactionAcc[JT_elbow] - state.coriolisAcc[JT_elbow] - state.gravityAcc[JT_elbow] - state.dampingAcc[JT_elbow]) / state.inertiaAcc[JT_elbow];
  }
  
  // compute shoulder acceleration
  double shdAcceleration = 0.0; 
  if(!m_jointLocked[JT_shoulder])
  { 
    state.coriolisAcc[JT_shoulder] = - m_massElbLLHalf * sqr(state.velocities[JT_elbow]) * sinElbAngle
    - m_massElbLL * state.velocities[JT_elbow] * state.velocities[JT_shoulder] * sinElbAngle;
    
    state.inertiaAcc[JT_shoulder] = (m_inertias[JT_elbow] + m_inertias[JT_shoulder] 
                                 + m_massElbLL * cosElbAngle 
                                 + (m_masses[JT_shoulder] * m_lengthsSq[JT_shoulder]) / 4.0f
                                 + m_massElbLSq4 
                                 + (m_masses[JT_elbow] * m_lengthsSq[JT_shoulder]) );
    
    state.interactionAcc[JT_shoulder] = state.accelerations[JT_elbow] * (m_inertias[JT_elbow] + m_massElbLSq4 + (m_massElbLLHalf * cosElbAngle));
    
    // shoulder gravity is elbow gravity + shoulder
    state.gravityAcc[JT_shoulder] = state.gravityAcc[JT_elbow];
    state.gravityAcc[JT_shoulder] += m_gravity * (m_masses[JT_elbow] + m_masses[JT_shoulder]) * m_lengths[JT_shoulder] * sinf(state.angles[JT_shoulder]);
    state.dampingAcc[JT_shoulder] = m_frictions[JT_shoulder] * state.velocities[JT_shoulder];
    shdAcceleration = (state.torques[JT_shoulder] - state.interactionAcc[JT_shoulder] - state.coriolisAcc[JT_shoulder] - state.gravityAcc[JT_shoulder] - state.dampingAcc[JT_shoulder]) / state.inertiaAcc[JT_shoulder];
  }
  
  // synchronous update  
  state.accelerations[JT_elbow] = elbAcceleration;
  state.accelerations[JT_shoulder] = shdAcceleration;      
}  
  
//----------------------------------------------------------------------------------------------------------------------  
void Arm::toXml(ci::XmlTree& xml)
{
  ci::XmlTree arm ("Arm", "");
  arm.setAttribute("Gravity", m_gravity);
  
  // Skeleton
  ci::XmlTree upperArm("UpperArm", "");
  upperArm.setAttribute("Length", m_lengths[JT_shoulder]);
  upperArm.setAttribute("Mass", m_masses[JT_shoulder]);  
  upperArm.setAttribute("Inertia", m_inertias[JT_shoulder]);    
  arm.push_back(upperArm);
  
  ci::XmlTree lowerArm("LowerArm", "");
  lowerArm.setAttribute("Length", m_lengths[JT_elbow]);
  lowerArm.setAttribute("Mass", m_masses[JT_elbow]);    
  lowerArm.setAttribute("Inertia", m_inertias[JT_elbow]);      
  arm.push_back(lowerArm);
  
  // Joints
  ci::XmlTree shoulder("Shoulder", "");
  shoulder.setAttribute("Friction", m_frictions[JT_shoulder]);
  shoulder.setAttribute("UpperLimit", m_limits[JT_shoulder][0]);
  shoulder.setAttribute("LowerLimit", m_limits[JT_shoulder][1]);
  shoulder.setAttribute("Locked", m_jointLocked[JT_shoulder]);
  arm.push_back(shoulder);
  
  ci::XmlTree elbow("Elbow", "");
  elbow.setAttribute("Friction", m_frictions[JT_elbow]);
  elbow.setAttribute("UpperLimit", m_limits[JT_elbow][0]);
  elbow.setAttribute("LowerLimit", m_limits[JT_elbow][1]);
  elbow.setAttribute("Locked", m_jointLocked[JT_elbow]);  
  arm.push_back(elbow);
  
  xml.push_back(arm);
}
  
  
} // namespace dmx
