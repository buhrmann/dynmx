/*
 *  main.cpp
 *  cinder_dynmx
 *
 *  Created by Thomas Buhrmann on 05/02/2011.
 *  Copyright 2011 __MyCompanyName__. All rights reserved.
 *
 */

#include "Simulation.h"
#include "SimulationFactory.h"

//----------------------------------------------------------------------------------------------------------------------
int main( int argc, char * const argv[] )
{
  // Createa a simulation, using the global config file as input
  dmx::Simulation* sim = dmx::SimulationFactory::create();
  
  // Kick off the simulation. If non-visual this returns whenever the model's hasFinished functions returns true.
  // If visual, might run until window is closed. 
  sim->start();
   
  // Everything went fine apparently.
  return 0;
}

