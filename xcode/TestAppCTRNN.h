/*
 *  TestAppCTRNN.h
 *  cinder_dynmx
 *
 *  Created by Thomas Buhrmann on 06/02/2011.
 *  Copyright 2011 __MyCompanyName__. All rights reserved.
 *
 */

#ifndef _DMX_TEST_APP_CTRNN_
#define _DMX_TEST_APP_CTRNN_

#include "TestViewCTRNN.h"
#include "TestModelCTRNN.h"

class TestAppCTRNN : public dmx::App
{
public:

  TestAppCTRNN()
  {
    m_ctrnn = new CTRNN(12);
    m_ctrnn->randomizeState(-1.0f, 1.0f);
    m_ctrnn->randomizeWeights(-10.0f, 10.0f);
    m_ctrnn->randomizeBiases(-10.0f, 10.0f);
    m_ctrnn->randomizeTimeConstants(2.0 / 30.0, 5.0f);    
    m_model = new TestModelCTRNN(m_ctrnn);
    m_view = new TestViewCTRNN(m_ctrnn);
  };
  
  virtual ~TestAppCTRNN() { delete m_ctrnn; };
  
  CTRNN* m_ctrnn;
};

#endif
