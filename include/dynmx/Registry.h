/*
 *  Registry.h
 *  cinder_dynmx
 *
 *  Created by Thomas Buhrmann on 26/06/2011.
 *  Copyright 2011 __MyCompanyName__. All rights reserved.
 *
 */

#ifndef _DMX_REGISTRY_
#define _DMX_REGISTRY_

#include "Dynmx.h"

namespace dmx
{

enum Models 
{
  kModelNone,
  kModelTest,
  kModelTestCTRNN,
  kModelTestEvolvable,
  kModelTestEvolvableCTRNN,
  kModelTestArm,
  kNumModels
};

#if 0  
static const char* ModelNames [kNumModels] =
{
  "None",
  "Test",
  "TestCTRNN",
  "TestEvolvable",
  "TestEvolvableCTRNN",
  "TestArm",
};
#endif

} // namespace dmx

#endif