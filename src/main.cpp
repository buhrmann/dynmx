/*
 *  main.cpp
 *  cinder_dynmx
 *
 *  Created by Thomas Buhrmann on 05/02/2011.
 *  Copyright 2011 __MyCompanyName__. All rights reserved.
 *
 */

#include "Dynmx.h"
#include "TestApp.h"
#include "TestAppCTRNN.h"
#include "TestAppEvolvableCTRNN.h"
#include "TestAppArm.h"

#include "cinder/xml.h"

// Toggle profiling
#define PROFILING 0

// Choose experiment to run
#define EXPERIMENT TestAppArm

// Which renderer to launch with
#define RENDERER ci::app::RendererGl

// Cross-Platform app launching
#ifdef DYNMX_WIN
#define LAUNCH(APP, RENDERER, NAME) cinder::app::AppBasic::executeLaunch(APP, RENDERER, #NAME);
#else DYNMX_MAC
#define LAUNCH(APP, RENDERER, NAME) cinder::app::AppBasic::executeLaunch(APP, RENDERER, #NAME, argc, argv);
#endif

// Launches a visual simulation
//----------------------------------------------------------------------------------------------------------------------
void runVisual( int argc, char * const argv[] )
{
  cinder::app::AppBasic *app = new EXPERIMENT;
  cinder::app::Renderer *ren = new RENDERER;
  LAUNCH(app, ren, EXPERIMENT);
}

// Runs a console only simulation
//----------------------------------------------------------------------------------------------------------------------
void runNonVisual()
{
  // Create
  ci::XmlTree progressLog ("GAEvolution", "");
  TestEvolvableCTRNN m_evoCtrnn (12);
  dmx::GARunner gaRunner (&m_evoCtrnn);//, &progressLog);
  gaRunner.setVerbosity(dmx::GARunner::kGAVerbosityMax);
  const bool incremental = false;
  if(incremental)
  {
    ci::XmlTree previousExp (ci::loadFile(std::string(DATA_BASE_DIR) + "11_06_20__12_21_18/GA_FinalPopulation.xml"));
    gaRunner.getGA()->fromXml(previousExp.getChild("Experiment"));
  }
  
  // Run
  const float dt = 1.0f / 30.0f;
  const int numGenerations = 10;
  while(gaRunner.getGA()->getCurrentGeneration() < numGenerations)
  {
    gaRunner.update(dt);
  }
  
  // Store results and setup
  ci::XmlTree experimentLog ("Experiment", "");
  gaRunner.getGA()->toXml(experimentLog, true);
  experimentLog.write(ci::writeFile(dmx::DATA_DIR + "GA_FinalPopulation.xml"));
  progressLog.write(ci::writeFile(dmx::DATA_DIR + "GA_Evolution.xml"));   
}

//----------------------------------------------------------------------------------------------------------------------
int main( int argc, char * const argv[] )
{
  // Cinder++ needs to set itself up (sets up memory on Mac, does nothing on windoze)
  ci::app::AppBasic::prepareLaunch();
  
#if PROFILING
  cout << "Start of Main" << endl;
#endif  
  
  const bool visual = true;
  if(visual)
  {
    runVisual(argc, argv);
  }
  else
  {
    runNonVisual();
  }
  
#if PROFILING
  cout << "End of Main" << endl;
#endif   
  
  // Cinder++ wants to clean up after itself
  ci::app::AppBasic::cleanupLaunch();
  
  return 0;
}

