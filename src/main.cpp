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
#include "TestAppEvolvableCTRNN.h"
#include "TestAppArm.h"

#include "cinder/xml.h"

#define VISUALISATION 0

#if VISUALISATION
//CINDER_APP_BASIC( TestApp, ci::app::RendererGl )
//CINDER_APP_BASIC( TestAppCTRNN, ci::app::RendererGl )
CINDER_APP_BASIC( TestAppEvolvableCTRNN, ci::app::RendererGl )
//CINDER_APP_BASIC( TestAppArm, ci::app::RendererGl )

#else
int main( int argc, char * const argv[] )
{
  // Run
  TestEvolvableCTRNN m_evoCtrnn (12);
  dmx::GARunner gaRunner (&m_evoCtrnn);
  const float dt = 1.0f / 30.0f;
  while(gaRunner.getGA()->getCurrentGeneration() < 2)
  {
    gaRunner.update(dt);
  }
  
  // Store results and setup
  gaRunner.getGA()->savePopulation(dmx::DATA_DIR + "GA_EndPopulation.txt");
  ci::XmlTree store ("Experiment", "");
  gaRunner.getGA()->toXml(store, true);
  store.write(ci::writeFile(dmx::DATA_DIR + "GA_EndPopulation.xml"));
  
  // test roundtrip
  dmx::GARunner gaRunner2 (&m_evoCtrnn);
  ci::XmlTree test (ci::loadFile(dmx::DATA_DIR + "GA_EndPopulation.xml"));
  gaRunner2.getGA()->fromXml(test.getChild("Experiment"));
  ci::XmlTree store2 ("Experiment", "");
  gaRunner2.getGA()->toXml(store2, true);
  store2.write(ci::writeFile(dmx::DATA_DIR + "GA_EndPopulation2.xml"));
}

#endif
