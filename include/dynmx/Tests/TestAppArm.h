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
    Arm2d arm;
    m_model = new TestModelArm(&arm);
    m_view = new TestViewArm(&arm);
  };
  
};

#endif
