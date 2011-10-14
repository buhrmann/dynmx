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
  
  void setVerbosity(GARunner::GAVerbosity v) { m_verbosity = v; };
  
protected:  
  
  // An instance of the model to evaluate
  Evolvable* m_evolvable;
  
  float m_time;
  float m_accFitness;
  uint16_t m_trial;
  uint16_t m_numTrials;
  
  int m_verbosity;
  
  ci::XmlTree* m_modelXml;  
};

} // namespace dmx

#endif