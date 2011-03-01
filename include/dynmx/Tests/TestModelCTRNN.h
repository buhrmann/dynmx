/*
 *  TestModel.h
 *  openDynmx
 *
 *  Created by Thomas Buhrmann on 13/01/2011.
 *  Copyright 2011 __MyCompanyName__. All rights reserved.
 *
 */

#include "Model.h"
#include "CTRNN.h"

//----------------------------------------------------------------------------------------------------------------------
class TestModelCTRNN : public dmx::Model
{
public:
  TestModelCTRNN() : m_ctrnn(0) {};
  TestModelCTRNN(CTRNN* ctrnn) : m_ctrnn(ctrnn) { init(); };
  virtual void init() { assert(m_ctrnn); m_ctrnn->randomizeState(-0.01, 0.01); m_ctrnn->randomizeWeights(-10.01, 10.01);};
  virtual void update(float dt) { m_ctrnn->update(1.0f/10.0f); };
  
  CTRNN* m_ctrnn;
};