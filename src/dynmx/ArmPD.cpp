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
void ArmPD::updatePD(float angle1, float angle2, float dt)
{
  float elbTorque = m_pd[JT_elbow].update(angle1, m_angles[JT_elbow], m_velocities[JT_elbow], dt);
  float shdTorque = m_pd[JT_shoulder].update(angle2, m_angles[JT_shoulder], m_velocities[JT_shoulder], dt);
  update(elbTorque*100*dt, shdTorque*100*dt, dt);
}

//----------------------------------------------------------------------------------------------------------------------
void ArmPD::updatePosition(float x, float y, float timeStep)
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


} // namespace dmx