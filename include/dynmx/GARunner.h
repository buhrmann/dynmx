/*
 *  ofxGARunner.h
 *  openDynmx
 *
 *  Created by Thomas Buhrmann on 10/01/2011.
 *  Copyright 2011 __MyCompanyName__. All rights reserved.
 *
 */

#ifndef _GA_RUNNER_
#define _GA_RUNNER_

#include "Model.h"
#include "GA.h"
#include <assert.h>

namespace dmx
{

//----------------------------------------------------------------------------------------------------------------------
// Interface for GA helper
//----------------------------------------------------------------------------------------------------------------------
class Evolvable
{
public:
  virtual int getNumGenes() = 0;
  virtual void decodeGenome(const double* genome) = 0;
  virtual float getFitness() = 0;
  virtual void update(float dt) {};
  virtual void reset() {};
};

// helpers
//----------------------------------------------------------------------------------------------------------------------
static double mapUnitIntervalToRange(double val, double min, double max)
{
  assert(max > min);
  assert(val >=0.0 && val <= 1.0);
  return min + val * (max - min);
}


//----------------------------------------------------------------------------------------------------------------------
// Interface for apps running a GA
//----------------------------------------------------------------------------------------------------------------------
class GARunner : public Model
{
public:
  GARunner(Evolvable* evolvable);
  
  virtual void update(float dt);
  virtual void init();
  
protected:  
  
  Evolvable* m_evolvable;
  
  GA* m_ga;
  GADescriptor m_gaDesc;
  
  float m_time;
  float m_accFitness;
  int m_trial;
  int m_numGenomesTested;
};

} // namespace dmx
#endif