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
#include "Topology.h"

namespace dmx
{
  
#define SMC_CIRC_AND_TRI 1
  
//----------------------------------------------------------------------------------------------------------------------
// A minimal agent scanning the encironment with a distance sensor
//----------------------------------------------------------------------------------------------------------------------
class SMCAgentEvo : public Evolvable
{
  
public:
  SMCAgentEvo();
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
  virtual inline bool hasFinished() { return m_agent->getTime() >= m_trialDuration; };
  
  virtual void toXml(ci::XmlTree& xml);
  virtual void record(Recorder& recorder);
  
  // Custom
  SMCAgent* getAgent() { return m_agent; };
  const Topology& getTopology() const { return m_topology; };
  
protected:
  
  virtual void updateFitness() {};
  
  SMCAgent* m_agent;
  Topology m_topology;
  float m_fitness;
  float m_trialDuration;
  float m_fitnessEvalDelay;
};
  
} // namespace

#endif