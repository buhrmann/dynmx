/*
 *  TestModelArmReflex.h
 *  cinder_dynmx
 *
 *  Created by Thomas Buhrmann on 10/14/11.
 *  Copyright 2011 __MyCompanyName__. All rights reserved.
 *
 */


#ifndef _DMX_TEST_MODEL_ARM_REFLEX_
#define _DMX_TEST_MODEL_ARM_REFLEX_

#include "Dynmx.h"
#include "Model.h"
#include "ArmReflex.h"
#include "Trajectory.h"
#include "Random.h"

#define RECORD_DATA 0

//----------------------------------------------------------------------------------------------------------------------
class TestModelArmReflex : public dmx::Model
{
public:
  
  enum TargetMode 
  {
    TM_Cursor,
    TM_Circle,
    TM_Trajectory
  };
  
  TestModelArmReflex() : m_arm(0), m_time(0) { m_arm = new dmx::ArmReflex(); init(); };
  
  virtual void init() 
  { 
    assert(m_arm); 
    
    m_arm->init();    
    
    m_targetMode = TM_Cursor;
    
    // Starting position
    const float startPosElb = dmx::PI_OVER_FOUR;
    const float startPosShd = dmx::PI_OVER_FOUR;
    Vec2f startPos, tmp;
    m_arm->forwardKinematics(startPosElb, startPosShd, tmp, startPos);
    m_targets.add(startPos, 1.5f);
    
    // Synergistic movement
    const float target1PosElb = dmx::PI_OVER_TWO;
    const float target1PosShd = 0.0f;
    Vec2f target1Pos;
    m_arm->forwardKinematics(target1PosElb, target1PosShd, tmp, target1Pos);     
    m_targets.add(target1Pos, 1.5f);    
    m_targets.setLoop(true);
    
    reset();
  }
  
  virtual void reset()
  {
    m_arm->reset();    
  }
  
  virtual void update(float dt)
  { 
    m_time += dt;
    
    switch (m_targetMode) 
    {
      case TM_Circle:
      {
        double angle = 1.0f * m_time;
        const float radius = 0.15f;
        float x = 0.45f + radius * sinf(angle);
        float y = radius * cosf(angle);
        m_arm->update(dmx::Pos(x,y), dt);                 
        break;        
      }
      case TM_Trajectory:
      {
        Vec2f target = m_targets.getPositionAtTime(m_time);
        m_arm->update(target, dt);
        break;        
      }
      case TM_Cursor:
      {
        m_arm->update(m_arm->getTarget(), dt);        
        break;
      }
    }
    
  };
  
  dmx::ArmReflex* m_arm;
  float m_time;
  dmx::Trajectory<ci::Vec2f> m_targets;
  TargetMode m_targetMode;
};

#endif