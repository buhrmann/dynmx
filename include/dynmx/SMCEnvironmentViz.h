/*
 *  SMCEnvironmentViz.h
 *  cinder_dynmx
 *
 *  Created by Thomas Buhrmann on 12/18/11.
 *  Copyright 2011 __MyCompanyName__. All rights reserved.
 *
 */

#ifndef _DMX_SMC_ENVIRONMENT_VIZ_
#define _DMX_SMC_ENVIRONMENT_VIZ_

#include "Scene.h"
#include "SMCEnvironment.h"

namespace dmx
{
  
//----------------------------------------------------------------------------------------------------------------------
// Visualisation of an SMCEnvironment
//----------------------------------------------------------------------------------------------------------------------
class SMCEnvironmentViz : public NodeGroup
{
public:
  
  SMCEnvironmentViz(SMCEnvironment* environment) : m_environment(environment) {};
  
  virtual void update();
  
protected:
  
  virtual void init() { NodeGroup::init(); };
  void drawGaussian(const Gaussian& g) const;
  
  SMCEnvironment* m_environment;
  
}; // class
  
} // namespace

#endif