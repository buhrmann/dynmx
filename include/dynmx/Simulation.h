/*
 *  Simulation.h
 *  cinder_dynmx
 *
 *  Created by Thomas Buhrmann on 21/06/2011.
 *  Copyright 2011 __MyCompanyName__. All rights reserved.
 *
 */
 
#ifndef _DNM_SIMULATION_
#define _DNM_SIMULATION_

#include "cinder/Timer.h"

namespace dmx
{

class Model;
class App;

//----------------------------------------------------------------------------------------------------------------------
// Base class for simulations that can run both visually and non-visually
//----------------------------------------------------------------------------------------------------------------------
class Simulation
{
public:
  Simulation(Model* model, App* app = 0) : m_model(model), m_app(app) {};
  
  void setApp(App* app) { m_app = app; };
  
  virtual void start();
  
protected:

  virtual void runVisual();
  virtual void runNonVisual();
    
  Model* m_model;
  App* m_app;
  
  cinder::Timer m_timer;
};

} // namespace dmx

#endif