//
//  TestModelArm.h
//  cinder_dynmx
//
//  Created by Thomas Buhrmann on 29/03/2011.
//  Copyright 2011 __MyCompanyName__. All rights reserved.
//

#include "Model.h"
#include "Arm.h"
#include "Random.h"

//----------------------------------------------------------------------------------------------------------------------
class TestModelArm : public dmx::Model
{
public:
  TestModelArm() : m_arm(0), m_time(0) {};
  TestModelArm(Arm2d* arm) : m_arm(arm) { /*init();*/ };
  
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
    const float initialElbowAngle = 0.3f;
    const float initialShoulderAngle = 0.5f;
    m_arm->setParameters(lowerArmMass, upperArmMass, lowerArmLength, upperArmLength, lowerArmInertia, upperArmInertia);
    m_arm->setGravity(9.81f);    
    m_arm->init(initialElbowAngle, initialShoulderAngle);
        
    // PDs
    m_arm->m_pd[0].m_P = 15.0f;
    m_arm->m_pd[0].m_D = 3.05f;
    
    m_arm->m_pd[1].m_P = 50.0f;
    m_arm->m_pd[1].m_D = 5.1f;    
  }
  
  virtual void update(float dt)
  { 
    float timeStep = dt;
    m_time += timeStep;
    //double angle = UniformRandom(0, TWO_PI);
    double angle = 4.0f * m_time;
    const float radius = 0.15f;
    float x = radius * sinf(angle);
    float y = radius * cosf(angle);
    //m_arm->update(0.0f, 0.0f, 1.0f / 100.0f); 
    m_arm->updatePosition(0.4f + x, y, timeStep);
  };
  
  Arm2d* m_arm;
  float m_time;
};