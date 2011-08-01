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
#include "TestModel.h"
#include "TestModelArm.h"
#include "TestModelArmMuscled.h"
#include "TestModelCTRNN.h"

#include "TestEvolvable.h"
#include "TestEvolvableCTRNN.h"
#include "TestApp.h"
#include "TestAppArm.h"
#include "TestAppCTRNN.h"
#include "TestAppEvolvableCTRNN.h"


namespace dmx
{

//----------------------------------------------------------------------------------------------------------------------
Simulation* SimulationFactory::create()
{
  Model* model  = 0;
  App* app = 0;
  
  // Check whether to run visually or not 
  bool visual = false;
  if (SETTINGS->hasChild("Config/Globals/Visual"))
  {
    visual = SETTINGS->getChild("Config/Globals/Visual").getAttributeValue<bool>("Value");
  }
  
  const std::string modelName = SETTINGS->getChild("Config/Globals/Model").getAttributeValue<std::string>("Value");
  
  // Switch on the chosen kind of model to run.
  if ("GA" == modelName)
  {
    Evolvable* evolvable;
    
    const std::string evolvableName = SETTINGS->getChild("Config/GA/Evolvable").getAttributeValue<std::string>("Name");
        
    if ("TestEvolvableCTRNN" == evolvableName)
    {    
      const int numNeurons = SETTINGS->getChild("Config/GA/Evolvable/NumNeurons").getAttributeValue<int>("Value");
      evolvable = new TestEvolvableCTRNN(numNeurons);
      model = new GARunner (evolvable);
      if(visual)
      {
        app = new TestAppEvolvableCTRNN((GARunner*)model);
      }
    }
    else if ("TestEvolvable" == evolvableName)
    {
      // Doesn't have a visualisation
      const int fitnessFunction = SETTINGS->getChild("Config/GA/Evolvable/FitnessFunction").getAttributeValue<int>("Value");
      evolvable = new TestEvolvable(fitnessFunction);
      model = new GARunner (evolvable);
    }
  }
  else if ("TestArm" == modelName)
  {
    model = new TestModelArm();
    if (visual)
    {
      app = new TestAppArm((TestModelArm*)model);
    }
  }
  else if ("TestArmMuscled" == modelName)
  {
    model = new TestModelArmMuscled();
    if (visual)
    {
      app = new TestAppArmMuscled((TestModelArm*)model);
    }
  }
  else if ("TestCTRNN" == modelName)
  {
    const int numNeurons = SETTINGS->getChild("Config/CTRNN/NumNeurons").getAttributeValue<int>("Value");
    model = new TestModelCTRNN(numNeurons);
    if(visual)
    {
      app = new TestAppCTRNN((TestModelCTRNN*)model);
    }    
  }
  else if ("Test" == modelName)
  {
    model = new TestModel();
    if(visual)
    {
      app = new TestApp((TestModel*)model);
    }
  }
  
  // If a model could be created given the information in the config file, return it, otherwise indicate failure.
  if(model != 0)
  {
    Simulation* sim = new Simulation (model, app);
    return sim;
  }
  else
  {
    std::cout << "AppFail: No model found with name as specified in Config.xml!" << endl;
  }

  return 0;
}

} // namespace dmx
