/*
 *  ArmMuscled.cpp
 *  cinder_dynmx
 *
 *  Created by Thomas Buhrmann on 28/06/2011.
 *  Copyright 2011 __MyCompanyName__. All rights reserved.
 *
 */

#include "ArmMuscled.h"

namespace dmx
{

//----------------------------------------------------------------------------------------------------------------------
void ArmMuscled::init(float elbAngle, float shdAngle)
{
  // Let base class do its initialisation first.
  Arm::init(elbAngle, shdAngle);
  
  m_muscles.clear();
  
  // Todo: Temporarily create some muscles here
  m_muscles.push_back(Muscle(this));
  m_muscles[0].addPathPoint(ci::Vec2f(0, 0.04), JT_shoulder);
  m_muscles[0].addPathPoint(ci::Vec2f(0.975, 0.06), JT_shoulder);
  m_muscles[0].addPathPoint(ci::Vec2f(0.025, 0.06), JT_elbow);  
  m_muscles[0].addPathPoint(ci::Vec2f(0.5, 0.04), JT_elbow);
  m_muscles[0].init();
  
  m_muscles.push_back(Muscle(this));
  m_muscles[1].addPathPoint(ci::Vec2f(0.25, -0.04), JT_shoulder);
  m_muscles[1].addPathPoint(ci::Vec2f(0.25, -0.04), JT_elbow);
  m_muscles[1].init();
}

//----------------------------------------------------------------------------------------------------------------------
void ArmMuscled::updateMuscles(float dt)
{
  float elbTorque = 0, shdTorque = 0;
  
  // Muscles return torques to apply to joints they span.
  for(size_t i = 0; i < m_muscles.size(); ++i)
  {
    m_muscles[i].update(dt);
    
    elbTorque += m_muscles[i].getForce(); 
  }
    
  update(elbTorque, shdTorque, dt);
  
}

} // namespace dmx