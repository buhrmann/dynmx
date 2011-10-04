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
#include "Arm.h"
#include "ArmMuscled.h"
#include "Trajectory.h"
#include "Random.h"
#include "Recorder.h"

#define RECORD_DATA 0

//----------------------------------------------------------------------------------------------------------------------
class TestModelArmMuscled : public dmx::Model
{
public:

  TestModelArmMuscled() : m_arm(0), m_time(0) { m_arm = new dmx::ArmMuscled(); init(); };
  
  virtual void init() 
  { 
    assert(m_arm); 

    m_arm->init();    
    m_saved = false;
    
    
    // Starting position
    const float startPosElb = PI_OVER_FOUR;
    const float startPosShd = PI_OVER_FOUR;
    Vec2f startPos, tmp;
    m_arm->forwardKinematics(startPosElb, startPosShd, tmp, startPos);
    m_targets.add(startPos, 1.5f);
    
    // Synergistic movement
    const float target1PosElb = PI_OVER_TWO;
    const float target1PosShd = 0.0f;
    Vec2f target1Pos;
    m_arm->forwardKinematics(target1PosElb, target1PosShd, tmp, target1Pos);     
    m_targets.add(target1Pos, 1.5f);
    
    m_targets.setLoop(true);
    
    reset();
  }
  
  virtual void reset()
  {
    const float initialElbowAngle = 0.75 * PI;
    const float initialShoulderAngle = 0.0f * PI;    
    m_arm->reset(initialElbowAngle, initialShoulderAngle);    
  }
  
  virtual bool hasFinished()
  {
#if RECORD_DATA    
    return m_time >= 1.0f;
#else
    return false;
    //return m_time >= m_targets.getDuration();
#endif
  }
  
  virtual void update(float dt)
  { 
    m_time += dt;
    
#if 0    
    double angle = 1.0f * m_time;
    const float radius = 0.15f;
    float x = 0.45f + radius * sinf(angle);
    float y = radius * cosf(angle);
    m_arm->updatePosition(dmx::Pos(x,y), dt);         
#endif
    
//    Vec2f target = getTarget();
//    m_arm->updatePosition(target, dt);
    
    m_arm->updatePosition(m_arm->getTarget(), dt);

    //m_arm->update(dt); 
    
#if RECORD_DATA    
    m_recorder.push_back("time", m_time);
    m_recorder.push_back("angle_elb", m_arm->getJointAngle(dmx::JT_elbow));
    m_recorder.push_back("angle_shd", m_arm->getJointAngle(dmx::JT_shoulder));
    
    m_recorder.push_back("length_0", m_arm->getMuscle(0)->getLength());
    m_recorder.push_back("length_n_0", m_arm->getMuscle(0)->getNormalisedLength()); 
    m_recorder.push_back("velocity_0", m_arm->getMuscle(0)->getVelocity()); 
    m_recorder.push_back("velocity_n_0", m_arm->getMuscle(0)->getNormalisedVelocity()); 
    m_recorder.push_back("activeForce_0", m_arm->getMuscle(0)->getActiveForce());
    m_recorder.push_back("passiveForce_0", m_arm->getMuscle(0)->getPassiveForce());
    m_recorder.push_back("velocityForce_0", m_arm->getMuscle(0)->getVelocityForce());
    m_recorder.push_back("excitation_0", m_arm->getMuscle(0)->getExcitation());
    m_recorder.push_back("activation_0", m_arm->getMuscle(0)->getActivation());
    m_recorder.push_back("force_0", m_arm->getMuscle(0)->getForce());
    
    m_recorder.push_back("length_1", m_arm->getMuscle(1)->getLength());
    m_recorder.push_back("length_n_1", m_arm->getMuscle(1)->getNormalisedLength()); 
    m_recorder.push_back("velocity_1", m_arm->getMuscle(1)->getVelocity()); 
    m_recorder.push_back("velocity_n_1", m_arm->getMuscle(1)->getNormalisedVelocity()); 
    m_recorder.push_back("activeForce_1", m_arm->getMuscle(1)->getActiveForce());
    m_recorder.push_back("passiveForce_1", m_arm->getMuscle(1)->getPassiveForce());
    m_recorder.push_back("velocityForce_1", m_arm->getMuscle(1)->getVelocityForce());
    m_recorder.push_back("excitation_1", m_arm->getMuscle(1)->getExcitation());
    m_recorder.push_back("activation_1", m_arm->getMuscle(1)->getActivation());
    m_recorder.push_back("force_1", m_arm->getMuscle(1)->getForce());    
#endif
    
#if RECORD_DATA        
    if(hasFinished())
    {
      finish();
    }
#endif    

  };
  
  Vec2f getTarget()
  {
    return m_targets.getPositionAt(m_time);
  }
  
  virtual void finish()
  {
#if RECORD_DATA
    m_recorder.saveTo(dmx::DATA_DIR + "MuscleData.txt");
    m_recorder.clear();
#endif
  }
  
  dmx::ArmMuscled* m_arm;
  dmx::Recorder m_recorder;
  bool m_saved;
  float m_time;
  dmx::Trajectory<ci::Vec2f> m_targets;
};

#endif