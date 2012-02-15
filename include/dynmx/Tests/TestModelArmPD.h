//
//  TestModelArm.h
//  cinder_dynmx
//
//  Created by Thomas Buhrmann on 29/03/2011.
//  Copyright 2011 __MyCompanyName__. All rights reserved.
//

#ifndef _DMX_TEST_MODEL_ARM_
#define _DMX_TEST_MODEL_ARM_

#include "Dynmx.h"
#include "Model.h"
#include "Arm.h"
#include "ArmPD.h"
#include "Random.h"

//----------------------------------------------------------------------------------------------------------------------
class TestModelArmPD : public dmx::Model
{
public:

  TestModelArmPD() : m_arm(0), m_time(0) { m_arm = new dmx::ArmPD(); init(); };
  
  virtual void init() 
  { 
    assert(m_arm); 
    
    m_arm->init();
        
    // PDs
    m_arm->m_pd[0].m_P = 1.0f;
    m_arm->m_pd[0].m_D = -0.05f;
    
    m_arm->m_pd[1].m_P = 1.0f;
    m_arm->m_pd[1].m_D = -0.1f;    
  };
  
  virtual void reset() 
  { 
    ((dmx::Arm*)m_arm)->resetTo(0.5f*PI, 0.5f*PI); 
  };
  
  virtual void update(float dt)
  { 
    m_time += dt;
    double angle = 1.0f * m_time;
    const float radius = 0.15f;
    float x = radius * sinf(angle);
    float y = radius * cosf(angle);
    m_arm->updatePosition(dt, 0.4f + x, y);    
  };
  
  dmx::ArmPD* m_arm;
  float m_time;
};

#endif