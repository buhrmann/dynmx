//
//  TestModelArm.h
//  cinder_dynmx
//
//  Created by Thomas Buhrmann on 29/03/2011.
//  Copyright 2011 __MyCompanyName__. All rights reserved.
//

#include "Model.h"
#include "Arm.h"

//----------------------------------------------------------------------------------------------------------------------
class TestModelArm : public dmx::Model
{
public:
  TestModelArm() : m_arm(0) {};
  TestModelArm(Arm2d* arm) : m_arm(arm) { init(); };
  
  virtual void init() 
  { 
    assert(m_arm); 
    m_arm->setParameters(0.5f, 1.0f, 0.29f, 0.35f, 1.0f, 1.0f);
    m_arm->init(0.3f, 0.4f);
  }
  
  virtual void update(float dt)
  { 
    m_arm->update(0.0f, 0.0f, 1.0f / 30.0f); 
  };
  
  Arm2d* m_arm;
};