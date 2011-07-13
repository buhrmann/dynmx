/*
 *  TestModel.h
 *  openDynmx
 *
 *  Created by Thomas Buhrmann on 13/01/2011.
 *  Copyright 2011 __MyCompanyName__. All rights reserved.
 *
 */

#ifndef _DMX_TEST_MODEL_
#define _DMX_TEST_MODEL_

#include "Model.h"

//----------------------------------------------------------------------------------------------------------------------
class TestModel : public dmx::Model
{
public:

  TestModel(){};
  virtual void init() { m_time = 0.0; m_val = 0.0; };
  virtual void update(float dt) { m_time += dt; m_val = sin(0.5f * m_time); };
  virtual bool hasFinished() { return m_time > 10.0f; };
  
  float m_val;
  float m_time;
};

#endif