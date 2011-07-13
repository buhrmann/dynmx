/*
 *  TestAppArm.h
 *  cinder_dynmx
 *
 *  Created by Thomas Buhrmann on 06/02/2011.
 *  Copyright 2011 __MyCompanyName__. All rights reserved.
 *
 */

#ifndef _DMX_TEST_APP_ARM_
#define _DMX_TEST_APP_ARM_

#include "Arm.h"
#include "TestViewArm.h"
#include "TestModelArm.h"
#include "TestModelArmMuscled.h"

//----------------------------------------------------------------------------------------------------------------------
class TestAppArm : public dmx::App
{
public:
  
  TestAppArm(TestModelArm* armModel)
  {
    m_model = armModel;
    m_view = new TestViewArm(armModel->m_arm);
  };
  
  dmx::Arm* m_arm;
};

//----------------------------------------------------------------------------------------------------------------------
class TestAppArmMuscled : public dmx::App
{
public:
  
  TestAppArmMuscled(TestModelArm* armModel)
  {
    m_model = armModel;
    m_view = new TestViewArm(armModel->m_arm, true);
  };
  
  dmx::Arm* m_arm;
};

#endif
