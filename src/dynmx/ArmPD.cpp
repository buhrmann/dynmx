/*
 *  ArmPD.cpp
 *  cinder_dynmx
 *
 *  Created by Thomas Buhrmann on 28/06/2011.
 *  Copyright 2011 __MyCompanyName__. All rights reserved.
 *
 */

#include "ArmPD.h"
#include "MathUtils.h"

namespace dmx
{

//----------------------------------------------------------------------------------------------------------------------  
void ArmPD::init()
{
  // First let base of arm init
  Arm::init();  
}
  
//----------------------------------------------------------------------------------------------------------------------  
void ArmPD::reset()
{
  // Let base class do its initialisation first.
  Arm::reset();  
  m_pd[0].reset();
  m_pd[1].reset();
  
  m_desiredAngle[0] = m_desiredAngle[1] = 0.0;  
}
  
//----------------------------------------------------------------------------------------------------------------------
void ArmPD::updatePD(float dt, float angle1, float angle2)
{
  float elbTorque = m_pd[JT_elbow].update(angle1, m_state.angles[JT_elbow], m_state.velocities[JT_elbow], dt);
  float shdTorque = m_pd[JT_shoulder].update(angle2, m_state.angles[JT_shoulder], m_state.velocities[JT_shoulder], dt);
  Arm::update(dt, elbTorque, shdTorque);
  //update(dt);
}

//----------------------------------------------------------------------------------------------------------------------
void ArmPD::updatePosition(float timeStep, float x, float y)
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
  
  m_desiredPos = Pos(x,y);
  inverseKinematics(m_desiredPos, 1.0f, m_desiredAngle[JT_elbow], m_desiredAngle[JT_shoulder]);
  updatePD(timeStep, m_desiredAngle[JT_elbow], m_desiredAngle[JT_shoulder]);
}


} // namespace dmx