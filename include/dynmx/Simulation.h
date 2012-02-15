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
// Singleton base class for simulations that can run both visually and non-visually
//----------------------------------------------------------------------------------------------------------------------
class Simulation
{
public:

  static Simulation* getInstance() { return s_instance; };
  static Simulation* create(Model* model, App* app);
  void start();
  
protected:
  
  Simulation(Model* model, App* app = 0) : m_model(model), m_app(app) {};

  void runVisual();
  void runNonVisual();
    
  Model* m_model;
  App* m_app;
  cinder::Timer m_timer;

  static Simulation* s_instance;
};

} // namespace dmx

#endif