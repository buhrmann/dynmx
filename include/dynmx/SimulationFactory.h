/*
 *  SimulationFactory.h
 *  cinder_dynmx
 *
 *  Created by Thomas Buhrmann on 24/06/2011.
 *  Copyright 2011 __MyCompanyName__. All rights reserved.
 *
 */

namespace dmx
{

class Simulation;



//----------------------------------------------------------------------------------------------------------------------
// Simplifies the creation of new apps / simulations by name from the global config file
//----------------------------------------------------------------------------------------------------------------------
class SimulationFactory
{
public:
  static Simulation* create();
};

} // namespace dmx