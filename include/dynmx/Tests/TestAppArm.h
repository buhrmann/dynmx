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

class TestAppArm : public dmx::App
{
public:
  
  TestAppArm()
  {
    m_arm = new Arm2d();
    m_model = new TestModelArm(m_arm);
    m_view = new TestViewArm(m_arm);
    m_view->m_app = this;
  };
  
  Arm2d* m_arm;
};

#endif
