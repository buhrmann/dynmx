/*
 *  main.cpp
 *  cinder_dynmx
 *
 *  Created by Thomas Buhrmann on 05/02/2011.
 *  Copyright 2011 __MyCompanyName__. All rights reserved.
 *
 */

#include "Dynmx.h"
#include "TestApp.h"
#include "TestAppCTRNN.h"
#include "TestAppEvolvableCTRNN.h"
#include "TestAppArm.h"

#include "EvoApp.h"
#include "Model.h"
#include "TestEvolvableCTRNN.h"

//----------------------------------------------------------------------------------------------------------------------
int main( int argc, char * const argv[] )
{
  const bool visual = false;
  
  TestEvolvableCTRNN ctrnn (5);
  if(visual)
  {
    TestAppEvolvableCTRNN app (&ctrnn);
    dmx::EvoApp evoApp(&ctrnn, &app);
    evoApp.start(true);
  }
  else
  {
    dmx::EvoApp evoApp(&ctrnn);
    evoApp.start(true);    
  }

   
  return 0;
}

