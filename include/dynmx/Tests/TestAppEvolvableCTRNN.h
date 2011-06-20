/*
 *  TestAppEvolvableCTRNN.h
 *  cinder_dynmx
 *
 *  Created by Thomas Buhrmann on 01/03/2011.
 *  Copyright 2011 __MyCompanyName__. All rights reserved.
 *
 */

#ifndef _DMX_TEST_APP_EVOLVABLE_CTRNN_
#define _DMX_TEST_APP_EVOLVABLE_CTRNN_

#include "TestViewEvolvableCTRNN.h"
#include "TestEvolvableCTRNN.h"
#include "GARunner.h"

class TestAppEvolvableCTRNN : public dmx::App
{
public:

  TestAppEvolvableCTRNN()
  {
    m_evoCtrnn = new TestEvolvableCTRNN(6);
    m_model = new dmx::GARunner(m_evoCtrnn);
    m_view = new TestViewEvolvableCTRNN(this, (dmx::GARunner*)m_model);
  };
  
  virtual void update() { dmx::App::update(1.0f / 30.0f); };
    
  TestEvolvableCTRNN* m_evoCtrnn;
};

#endif
