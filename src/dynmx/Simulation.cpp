/*
 *  Simulation.cpp
 *  cinder_dynmx
 *
 *  Created by Thomas Buhrmann on 21/06/2011.
 *  Copyright 2011 __MyCompanyName__. All rights reserved.
 *
 */

#include "cinder/app/AppBasic.h"

#include "Dynmx.h"
#include "Simulation.h"
#include "App.h"
#include "Model.h"
#include "MathUtils.h"

namespace dmx
{

Simulation* Simulation::s_instance = NULL;  

//----------------------------------------------------------------------------------------------------------------------  
Simulation* Simulation::create(Model* model, App* app) 
{ 
  if(!s_instance) 
    s_instance = new Simulation(model, app); 
  
  return s_instance; 
}  
  
//----------------------------------------------------------------------------------------------------------------------
void Simulation::start()
{
  // Cinder++ needs to set itself up (sets up memory on Mac, does nothing on windoze)
  ci::app::AppBasic::prepareLaunch();
  
  // Running time
  m_timer.start();
  
  if(m_app)
  {
    runVisual();  
  }
  else 
  {
    runNonVisual(); 
  } 
  
  // Output running time
  m_timer.stop();
  double runningTime = m_timer.getSeconds();
  int h, m, s;
  secondsToTime(runningTime, h, m, s);
  std::cout << "Total running time:" << h << ":" << m << ":" << s << std::endl;
  
  std::cout << "Output in " << DATA_DIR << std::endl;
  
  // Cinder++ wants to clean up after itself
  ci::app::AppBasic::cleanupLaunch();  
}

//----------------------------------------------------------------------------------------------------------------------
void Simulation::runVisual()
{  
  char** const argv = 0;
  const int antiAliasingSteps = 4; // Todo: read from config.xml.
  cinder::app::Renderer* renderer = new ci::app::RendererGl(antiAliasingSteps);
  cinder::app::AppBasic::executeLaunch((ci::app::AppBasic*)m_app, renderer, "Test", 0, argv);  
}

//----------------------------------------------------------------------------------------------------------------------
void Simulation::runNonVisual()
{  
  // Read global config
  float dt = 1.0 / (float) DEFAULT_FRAMERATE;
  if (SETTINGS->hasChild("Config/Globals/FrameRate"))
  {
    dt = 1.0 / (float) SETTINGS->getChild("Config/Globals/FrameRate").getAttributeValue<int>("Value");
  }
  
  while(!m_model->hasFinished())
  {
    m_model->update(dt);
  }
}

} // namespace dmx
