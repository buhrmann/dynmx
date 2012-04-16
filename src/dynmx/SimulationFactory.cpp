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
#include "GATester.h"

#include "EvoArmCoCon.h"

#include "TestModel.h"
#include "TestModelArmPD.h"
#include "TestModelArmReflex.h"
#include "TestModelEvolvableArm.h"
#include "TestModelCTRNN.h"

#include "TestEvolvable.h"
#include "TestEvolvableCTRNN.h"
#include "TestAppCTRNN.h"

#include "TestView.h"
#include "TestViewEvolvableCTRNN.h"

#include "ArmView.h"
#include "ArmPDView.h"
#include "ArmMuscledView.h"
#include "ArmReflexView.h"

#include "SMCAgent.h"
#include "SMCAgentEvo.h"
#include "SMCAgentLineDis.h"
#include "SMCView.h"

#include "Spin.h"

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
  if ("GA" == modelName || "GAEval" == modelName)
  {
    Evolvable* evolvable;
    const std::string evolvableName = SETTINGS->getChild("Config/GA/Evolvable").getAttributeValue<std::string>("Name");
    
    // Evolve or Evaluate?
    bool evaluateOnly = true;
    if(SETTINGS->hasChild("Config/GA/Eval"))
    {
      evaluateOnly = SETTINGS->getChild("Config/GA/Eval").getAttributeValue<bool>("Run");
    }
        
    if ("TestEvolvableCTRNN" == evolvableName)
    {    
      const int numNeurons = SETTINGS->getChild("Config/GA/Evolvable/NumNeurons").getAttributeValue<int>("Value");
      evolvable = new TestEvolvableCTRNN(numNeurons);
      model = new GARunner (evolvable);
      if(visual)
      {
        View* view = new TestViewEvolvableCTRNN((GARunner*)model);
        app = new App(model, view);
      }
    }
    else if ("SMCAgentEvo" == evolvableName)
    {    
      evolvable = new SMCAgentEvo();
      model = evaluateOnly ? (Model*) new GATester(evolvable) : (Model*) new GARunner (evolvable);
      if(visual)
      {
        View* view = new SMCView((SMCAgentEvo*)evolvable);
        app = new App(model, view);
      }
    }
    else if ("SMCAgentLineDis" == evolvableName)
    {    
      evolvable = new SMCAgentLineDis();
      model = evaluateOnly ? (Model*) new GATester(evolvable) : (Model*) new GARunner (evolvable);
      if(visual)
      {
        View* view = new SMCView((SMCAgentEvo*)evolvable);
        app = new App(model, view);
      }
    }    
    else if ("TestEvolvable" == evolvableName)
    {
      // Doesn't have a visualisation
      const int fitnessFunction = SETTINGS->getChild("Config/GA/Evolvable/FitnessFunction").getAttributeValue<int>("Value");
      evolvable = new TestEvolvable(fitnessFunction);
      model = new GARunner (evolvable);
    }
    else if ("EvoArmCoCon" == evolvableName)
    {
      EvoArmCoCon* evoArm = new EvoArmCoCon();
      model = evaluateOnly ? new GATester(evoArm) : model = new GARunner(evoArm);
      if(visual)
      {         
        View* view = new ArmReflexView(evoArm->m_arm);
        app = new App(model, view);
      }
    }      
    
    // Enable highest level of logging
    if(evaluateOnly)
    {
      ((GATester*)model)->setVerbosity(GARunner::kGAVerbosityTrial);
    }
  }
  else if ("TestArm" == modelName)
  {
    model = new Arm();
    model->init();
    if (visual)
    {
      View* view = new ArmView((Arm*)model);
      app = new App(model, view);
    }
  }
  else if ("TestArmPD" == modelName)
  {
    model = new TestModelArmPD();
    model->init();
    if (visual)
    {
      View* view = new ArmPDView(((TestModelArmPD*)model)->m_arm);
      app = new App(model, view);
    }
  }  
  else if ("TestArmMuscled" == modelName)
  {
    model = new ArmMuscled();
    model->init();
    if (visual)
    {
      View* view = new ArmMuscledView(((ArmMuscled*)model));
      app =  new App(model, view);
    }    
  }
  else if ("TestArmReflex" == modelName)
  {
    TestModelArmReflex* armR = new TestModelArmReflex();
    model = armR;
    if (visual)
    {
      View* view = new ArmReflexView(((ArmReflex*)armR->m_arm));
      app =  new App(model, view);
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
  /*
  else if ("SMCAgent" == modelName)
  {
    model = new SMCAgent();
    if(visual)
    {
      View* view = new SMCView((SMCAgent*)model);      
      app = new App(model, view);
    }
  } */ 
  else if ("Test" == modelName)
  {
    model = new TestModel();
    if(visual)
    {
      View* view = new TestView((TestModel*)model);      
      app = new App(model, view);
    }
  }
  else if ("Spin" == modelName)
  {
    model = new SpinningWheel();
    if(visual)
    {
      View* view = new SpinningWheelView((SpinningWheel*)model);      
      app = new App(model, view);
    }
  }  
  
  // If a model could be created given the information in the config file, return it, otherwise indicate failure.
  if(model != 0)
  {
    return Simulation::create(model, app);
  }
  else
  {
    std::cout << "AppFail: No model found with name as specified in Config.xml!" << endl;
  }

  return 0;
}

} // namespace dmx
