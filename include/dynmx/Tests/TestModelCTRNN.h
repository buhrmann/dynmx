/*
 *  TestModel.h
 *  openDynmx
 *
 *  Created by Thomas Buhrmann on 13/01/2011.
 *  Copyright 2011 __MyCompanyName__. All rights reserved.
 *
 */

#ifndef _DMX_TEST_MODEL_CTRNN_
#define _DMX_TEST_MODEL_CTRNN_

#include "Model.h"
#include "CTRNN.h"

//----------------------------------------------------------------------------------------------------------------------
class TestModelCTRNN : public dmx::Model
{
public:
  
  TestModelCTRNN() : m_ctrnn(0) {};
  TestModelCTRNN(int numNeurons) { m_ctrnn = new CTRNN(numNeurons); init(); };
  
  virtual void init() { assert(m_ctrnn); reset(); };
  virtual void reset() { m_ctrnn->randomizeState(-0.01, 0.01); m_ctrnn->randomizeWeights(-10.01, 10.01); };
  virtual void update(float dt) { m_ctrnn->update(dt); };
  CTRNN* m_ctrnn;
};

#endif