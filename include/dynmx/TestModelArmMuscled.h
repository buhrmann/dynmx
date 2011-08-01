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
#include "Recorder.h"

//----------------------------------------------------------------------------------------------------------------------
class TestModelArmMuscled : public dmx::Model
{
public:

  TestModelArmMuscled() : m_arm(0), m_time(0) { m_arm = new dmx::ArmMuscled(); };
  
  virtual void init() 
  { 
    assert(m_arm); 

    m_arm->init();
    //const float initialElbowAngle = 0.5f * PI;
    //const float initialShoulderAngle = 0.0f * PI;    
    //m_arm->reset(initialElbowAngle, initialShoulderAngle);
    
    m_saved = false;
  }
  
  virtual void reset()
  {
    m_arm->reset();
  }
  
  virtual void update(float dt)
  { 
    m_time += dt;
 
    m_arm->update(dt); 
    
#if 0
    m_recorder.push_back("angle_elb", m_arm->getJointAngle(dmx::JT_elbow));
    m_recorder.push_back("angle_shd", m_arm->getJointAngle(dmx::JT_shoulder));

    m_recorder.push_back("length_0", m_arm->getMuscle(0).getLength());
    m_recorder.push_back("length_n_0", m_arm->getMuscle(0).getNormalisedLength()); 
    m_recorder.push_back("velocity_0", m_arm->getMuscle(0).getVelocity()); 
    m_recorder.push_back("velocity_n_0", m_arm->getMuscle(0).getNormalisedVelocity()); 
    m_recorder.push_back("activeForce_0", m_arm->getMuscle(0).getActiveForce());
    m_recorder.push_back("passiveForce_0", m_arm->getMuscle(0).getPassiveForce());
    m_recorder.push_back("velocityForce_0", m_arm->getMuscle(0).getVelocityForce());
    m_recorder.push_back("excitation_0", m_arm->getMuscle(0).getExcitation());
    m_recorder.push_back("activation_0", m_arm->getMuscle(0).getActivation());
    m_recorder.push_back("force_0", m_arm->getMuscle(0).getForce());
    
    m_recorder.push_back("length_1", m_arm->getMuscle(1).getLength());
    m_recorder.push_back("length_n_1", m_arm->getMuscle(1).getNormalisedLength()); 
    m_recorder.push_back("velocity_1", m_arm->getMuscle(1).getVelocity()); 
    m_recorder.push_back("velocity_n_1", m_arm->getMuscle(1).getNormalisedVelocity()); 
    m_recorder.push_back("activeForce_1", m_arm->getMuscle(1).getActiveForce());
    m_recorder.push_back("passiveForce_1", m_arm->getMuscle(1).getPassiveForce());
    m_recorder.push_back("velocityForce_1", m_arm->getMuscle(1).getVelocityForce());
    m_recorder.push_back("excitation_1", m_arm->getMuscle(1).getExcitation());
    m_recorder.push_back("activation_1", m_arm->getMuscle(1).getActivation());
    m_recorder.push_back("force_1", m_arm->getMuscle(1).getForce());    
    
    if(!m_saved && m_time >= 1.0f)
    {
      m_recorder.saveTo(dmx::DATA_DIR + "MuscleData.txt");
      m_recorder.clear();
      m_saved = true;
    }
#endif       
  };
  
  virtual void shutDown()
  {
  }
  
  dmx::ArmMuscled* m_arm;
  dmx::Recorder m_recorder;
  bool m_saved;
  float m_time;
};

#endif