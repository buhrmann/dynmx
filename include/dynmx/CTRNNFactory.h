/*
 *  untitled.h
 *  cinder_dynmx
 *
 *  Created by Thomas Buhrmann on 3/22/12.
 *  Copyright 2012 __MyCompanyName__. All rights reserved.
 *
 */


#ifndef CTRNN_FACTORY_H
#define CTRNN_FACTORY_H

#include "CTRNN.h"
#include "Topology.h"

namespace dmx
{
  
//----------------------------------------------------------------------------------------------------------------------
// A CTRNN Factory
//----------------------------------------------------------------------------------------------------------------------
class CTRNNFactory 
{
public:
  
  // Helper for storing the ranges of CTRNN parameters
  //--------------------------------------------------------------------------------------------------------------------
  struct DecodeLimits
  {
    DecodeLimits() : weight(-10,10), bias(-10,10), gain(-10,10), tau(0.2,2.0) {};
    Range weight, bias, tau, gain;
  };
  
  //static bool decode(CTRNN& ctrnn, const double* params, const DecodeLimits& limits, int numInputs = 0);
  static bool decode(CTRNN& ctrnn, const double* params, const DecodeLimits& limits, const Topology& top);
  
protected:
  
};

} // namespace

#endif
