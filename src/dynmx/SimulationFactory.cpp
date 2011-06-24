/*
 *  SimulationFactory.cpp
 *  cinder_dynmx
 *
 *  Created by Thomas Buhrmann on 24/06/2011.
 *  Copyright 2011 __MyCompanyName__. All rights reserved.
 *
 */

#include "Dynmx.h"
#include "App.h"
#include "Simulation.h"
#include "SimulationFactory.h"

// Individual models and view used by the factory
#include "GARunner.h"
#include "TestEvolvableCTRNN.h"
#include "TestAppEvolvableCTRNN.h"


namespace dmx
{

//----------------------------------------------------------------------------------------------------------------------
Simulation* SimulationFactory::create()
{
  Model* model = 0;
  App* app = 0;
  
  // Check whether to run visually or not 
  bool visual = false;
  if (dmx::SETTINGS->hasChild("Config/Globals/Visual"))
  {
    visual = dmx::SETTINGS->getChild("Config/Globals/Visual").getAttributeValue<bool>("Value");
  }
  
  const std::string modelName = dmx::SETTINGS->getChild("Config/Globals/Model").getAttributeValue<std::string>("Value");
  if("GA" == modelName)
  {
    TestEvolvableCTRNN* ctrnn = new TestEvolvableCTRNN(5);
    model = new GARunner (ctrnn);
    if(visual)
    {
      app = new TestAppEvolvableCTRNN((GARunner*)model);
    }    
  }
  
  Simulation* sim = new Simulation (model, app);
  return sim;
}

} // namespace dmx
