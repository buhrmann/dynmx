/*
 *  GATester.h
 *  cinder_dynmx
 *
 *  Created by Thomas Buhrmann on 9/16/11.
 *  Copyright 2011 __MyCompanyName__. All rights reserved.
 *
 */
#ifndef _GA_TESTER_
#define _GA_TESTER_

#include "Dynmx.h"
#include "Model.h"
#include "GARunner.h"
#include "Recorder.h"

#include "cinder/xml.h"

#include <assert.h>

namespace dmx
{
  
//----------------------------------------------------------------------------------------------------------------------
// Interface for apps running a GA
//----------------------------------------------------------------------------------------------------------------------
class GATester : public Model
{
public:
  
  GATester(Evolvable* evolvable);
  
  // Inherited from Model
  virtual void update(float dt);
  virtual void init();
  virtual void reset();

  virtual bool hasFinished() { return m_trial >= m_numTrials; };
  Evolvable* getEvolvable() { return m_evolvable; };
  ci::XmlTree& getXml() { return *m_modelXml; };
  
  void setVerbosity(GARunner::GAVerbosity v) { m_verbosity = v; };
  
  void enableRecording(bool record) { m_record = record; };
  
protected:
  
  void mutate(double* genome, int numGenes, double maxMut);

  
  // An instance of the model to evaluate
  double* m_genome;
  int m_numGenes;
  Evolvable* m_evolvable;
  
  float m_time;
  float m_accFitness;
  uint32_t m_trial;
  uint32_t m_numTrials;
  
  int m_verbosity;
  bool m_record;
  
  ci::XmlTree* m_modelXml;  
  Recorder m_recorder;
  Recorder m_fitnessLog;
  
  bool m_mutate;
  double m_mutateMin;
  double m_mutateMax;
  double m_mutateStep;
  int m_numEvalsPerMutation;
  
  long m_idum;                // seed for random number generator
};

} // namespace dmx

#endif