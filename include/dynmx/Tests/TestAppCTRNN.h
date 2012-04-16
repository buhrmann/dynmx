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

  TestAppCTRNN(TestModelCTRNN* model)
  {
    model->m_ctrnn->randomizeState(-1.0f, 1.0f);
    model->m_ctrnn->randomizeWeights(-10.0f, 10.0f);
    model->m_ctrnn->randomizeBiases(-10.0f, 10.0f);
    model->m_ctrnn->randomizeTimeConstants(2.0 / 30.0, 5.0f);
    m_model = model; 
    m_view = new TestViewCTRNN(model->m_ctrnn);
  };
  
  dmx::CTRNN* m_ctrnn;
};

#endif
