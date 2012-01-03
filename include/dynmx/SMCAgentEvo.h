/*
 *  SMCAgentEvo.h
 *  cinder_dynmx
 *
 *  Created by Thomas Buhrmann on 12/22/11.
 *  Copyright 2011 __MyCompanyName__. All rights reserved.
 *
 */

#ifndef _DMX_SMC_AGENT_EVO_
#define _DMX_SMC_AGENT_EVO_

#include "SMCAgent.h"
#include "GARunner.h"

namespace dmx
{
  
#define SMC_TRIAL_DURATION 10.0f
#define SMC_CIRC_AND_TRI 0
  
//----------------------------------------------------------------------------------------------------------------------
// A minimal agent scanning the encironment with a distance sensor
//----------------------------------------------------------------------------------------------------------------------
class SMCAgentEvo : public Evolvable
{
  
public:
  SMCAgentEvo(int numNeurons);
  
  virtual int getNumGenes();
  virtual void decodeGenome(const double* genome);
  virtual float getFitness();
  
  virtual void update(float dt);
  virtual void init();
  virtual void reset();
  
  virtual bool hasFinished() { return m_agent->getTime() >= SMC_TRIAL_DURATION; };
  
  SMCAgent* getAgent() { return m_agent; };
  
protected:
  
  SMCAgent* m_agent;
  float m_fitness;
};
  
} // namespace

#endif