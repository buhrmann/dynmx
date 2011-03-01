/*
 *  TestModel.h
 *  openDynmx
 *
 *  Created by Thomas Buhrmann on 13/01/2011.
 *  Copyright 2011 __MyCompanyName__. All rights reserved.
 *
 */

#include "Model.h"

//----------------------------------------------------------------------------------------------------------------------
class TestModel : public dmx::Model
{
public:

  virtual void init() { m_time = 0.0; m_val = 0.0; };
  virtual void update(float dt) { m_time += dt; m_val = sin(0.1*m_time); };
  
  float m_val;
  float m_time;
};
