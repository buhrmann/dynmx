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

#include "cinder/xml.h"

#include <assert.h>

namespace dmx
{

//----------------------------------------------------------------------------------------------------------------------
// Interface for an evolvable model
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
  
  enum GAVerbosity
  {
    kGAVerbosityNone,
    kGAVerbosityPopulation,    
    kGAVerbosityGenome,    
    kGAVerbosityTrial
  };
  
  GARunner(Evolvable* evolvable);
  
  // Inherited from Model
  virtual void update(float dt);
  virtual void init();
  virtual void reset();
  virtual bool hasFinished() { return m_ga->getCurrentGeneration() == m_numGenerations; };
  
  Evolvable* getEvolvable() { return m_evolvable; };
  GA* getGA() { return m_ga; };
  
  void setVerbosity(GAVerbosity v) { m_verbosity = v; };
  
protected:  
  
  void generationToXml(ci::XmlTree* xml, uint32_t gen, const double* genome, float bestFit, float avgFit);
  
  // An instance of the model to evolve
  Evolvable* m_evolvable;
  
  // An instance of the actual algorithm
  GA* m_ga;
  
  float m_time;
  float m_accFitness;
  float m_trialDuration;  
  uint16_t m_trial;
  uint16_t m_numTrials;
  uint32_t m_prevGeneration;  /// For detecting if GA has incremented a generation
  uint32_t m_numGenerations;
  
  int m_verbosity;
  
  ci::XmlTree* m_progressLog;
  ci::XmlTree* m_resultsLog;
};

} // namespace dmx

#endif
