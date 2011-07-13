//
//  TestModelArmMuscles.h
//  cinder_dynmx
//
//  Created by Thomas Buhrmann on 29/03/2011.
//  Copyright 2011 __MyCompanyName__. All rights reserved.
//

#ifndef _DMX_TEST_MODEL_ARM_MUSCLED_
#define _DMX_TEST_MODEL_ARM_MUSCLED_

#include "Dynmx.h"
#include "Model.h"
#include "ArmMuscled.h"
#include "Random.h"

//----------------------------------------------------------------------------------------------------------------------
class TestModelArmMuscled : public dmx::Model
{
public:

  TestModelArmMuscled() : m_arm(0), m_time(0) { m_arm = new dmx::ArmMuscled(); };
  
  virtual void init() 
  { 
    assert(m_arm); 
    
    // arm: parameters from Karniel and Inbar
    const float lowerArmMass = 1.3f;
    const float upperArmMass = 2.52f;
    const float lowerArmLength = 0.32f;
    const float upperArmLength = 0.33f;
    const float lowerArmInertia = lowerArmLength * lowerArmLength * lowerArmMass / 12.0f;
    const float upperArmInertia = upperArmLength * upperArmLength * upperArmMass / 12.0f;
    const float initialElbowAngle = 0.5f * PI;
    const float initialShoulderAngle = 0.0f * PI;
    m_arm->setParameters(lowerArmMass, upperArmMass, lowerArmLength, upperArmLength, lowerArmInertia, upperArmInertia);
    m_arm->setGravity(9.81f);    
    m_arm->init(initialElbowAngle, initialShoulderAngle);
  }
  
  virtual void update(float dt)
  { 
    m_time += dt;
    m_arm->updateMuscles(dt);    
  };
  
  dmx::ArmMuscled* m_arm;
  float m_time;
};

#endif