/*
 *  EvoApp.cpp
 *  cinder_dynmx
 *
 *  Created by Thomas Buhrmann on 21/06/2011.
 *  Copyright 2011 __MyCompanyName__. All rights reserved.
 *
 */

#include "cinder/app/AppBasic.h"

#include "Dynmx.h"
#include "EvoApp.h"
#include "Model.h"
#include "App.h"
#include "GARunner.h"

namespace dmx
{

//----------------------------------------------------------------------------------------------------------------------
void EvoApp::start(bool evolve)
{
  // Cinder++ needs to set itself up (sets up memory on Mac, does nothing on windoze)
  ci::app::AppBasic::prepareLaunch();
  
  if(m_app)
  {
    runVisual(evolve);  
  }
  else 
  {
    runNonVisual(evolve); 
  } 
  
  // Cinder++ wants to clean up after itself
  ci::app::AppBasic::cleanupLaunch();  
}

//----------------------------------------------------------------------------------------------------------------------
void EvoApp::runVisual(bool evolve)
{
  char** const argv = 0;
  cinder::app::Renderer* renderer = new ci::app::RendererGl;
  ((GARunner*)m_app->m_model)->setVerbosity(GARunner::kGAVerbosityMax);
  cinder::app::AppBasic::executeLaunch((ci::app::AppBasic*)m_app, renderer, "Test", 0, argv);  
}

//----------------------------------------------------------------------------------------------------------------------
void EvoApp::runNonVisual(bool evolve)
{
  GARunner gaRunner (m_evolvable);
  gaRunner.setVerbosity(GARunner::kGAVerbosityMax);
  
  const float dt = 1.0f / 30.0f;
  const int numGenerations = 10;
  while(gaRunner.getGA()->getCurrentGeneration() < numGenerations)
  {
    gaRunner.update(dt);
  }
}

} // namespace dmx