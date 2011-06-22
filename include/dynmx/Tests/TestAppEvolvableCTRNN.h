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

  TestAppEvolvableCTRNN(TestEvolvableCTRNN* evoCtrnn)
  {
    m_model = new dmx::GARunner(evoCtrnn);
    m_view = new TestViewEvolvableCTRNN(this, m_model);
  };
    
};

#endif
