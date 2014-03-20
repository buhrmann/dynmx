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
#include "Bacterium.h"
#include "GARunner.h"

namespace dmx
{
  
#define SMC_CIRC_AND_TRI 1
  
//----------------------------------------------------------------------------------------------------------------------
// A minimal agent scanning the encironment with a distance sensor
//----------------------------------------------------------------------------------------------------------------------
class SMCAgentEvo : public Evolvable
{
  
friend class SMCView;
  
public:
  SMCAgentEvo();
  SMCAgentEvo(SMCAgent* agent);
  ~SMCAgentEvo();

  // Inherited from Model  
  virtual void update(float dt);
  virtual void init();
  virtual void reset();  
  
  // Inherited from Evolvable
  virtual int getNumGenes();
  virtual void decodeGenome(const double* genome);
  virtual float getFitness();
  virtual void nextTrial(int trial = 0) {};
  virtual bool hasFinished() { return m_agent->getTime() >= m_trialDuration; };
  
  virtual void toXml(ci::XmlTree& xml);
  virtual void record(Recorder& recorder);
  
  // Custom
  SMCAgent* getAgent() { return m_agent; };
  
protected:
  
  virtual void updateFitness(float dt) {};
  
  SMCAgent* m_agent;
  NetLimits m_netLimits;
  float m_fitness;
  float m_fitnessInst;
  float m_trialDuration;
  float m_fitnessEvalDelay;
};
  
} // namespace

#endif