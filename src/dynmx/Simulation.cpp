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
#include "Model.h"

namespace dmx
{

//----------------------------------------------------------------------------------------------------------------------
void Simulation::start()
{
  // Cinder++ needs to set itself up (sets up memory on Mac, does nothing on windoze)
  ci::app::AppBasic::prepareLaunch();
  
  if(m_app)
  {
    runVisual();  
  }
  else 
  {
    runNonVisual(); 
  } 
  
  // Cinder++ wants to clean up after itself
  ci::app::AppBasic::cleanupLaunch();  
}

//----------------------------------------------------------------------------------------------------------------------
void Simulation::runVisual()
{
  char** const argv = 0;
  cinder::app::Renderer* renderer = new ci::app::RendererGl;
  cinder::app::AppBasic::executeLaunch((ci::app::AppBasic*)m_app, renderer, "Test", 0, argv);  
}

//----------------------------------------------------------------------------------------------------------------------
void Simulation::runNonVisual()
{
  const float dt = 1.0f / 30.0f;
  while(!m_model->hasFinished())
  {
    m_model->update(dt);
  }
}

} // namespace dmx