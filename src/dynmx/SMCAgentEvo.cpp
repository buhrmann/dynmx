/*
 *  SMCAgentEvo.cpp
 *  cinder_dynmx
 *
 *  Created by Thomas Buhrmann on 12/22/11.
 *  Copyright 2011 __MyCompanyName__. All rights reserved.
 *
 */

#include "SMCAgentEvo.h"

namespace dmx
{

//----------------------------------------------------------------------------------------------------------------------  
SMCAgentEvo::SMCAgentEvo()
{
  if (SETTINGS->hasChild("Config/GA/Evolvable/Topology"))
  {
    Topology topology;
    topology.fromXml(SETTINGS->getChild("Config/GA/Evolvable/Topology"));
    m_agent = new SMCAgent(topology);
  }
  
  init();
}

//----------------------------------------------------------------------------------------------------------------------
SMCAgentEvo::SMCAgentEvo(SMCAgent* agent)
{
  m_agent = agent;
  
  init();
}

//----------------------------------------------------------------------------------------------------------------------    
SMCAgentEvo::~SMCAgentEvo()
{
  delete m_agent;
}

//----------------------------------------------------------------------------------------------------------------------    
void SMCAgentEvo::init() 
{
  if (SETTINGS->hasChild("Config/GA/Evolvable"))
  {
    const ci::XmlTree& xml = SETTINGS->getChild("Config/GA/Evolvable");
    m_netLimits.fromXml(xml.getChild("NetLimits"));
    m_trialDuration = xml.getChild("TrialDuration").getValue<double>(10.0);
    m_fitnessEvalDelay = xml.getChild("FitnessEvalDelay").getValue<double>(8.0);    
  }
};  

//----------------------------------------------------------------------------------------------------------------------  
void SMCAgentEvo::reset() 
{ 
  // Set agent back to zero
  m_agent->reset();   
  
  m_fitness = 0.0f;

#define NN_RAND 0
#if NN_RAND
  float min = -1;
  float max = 1;
  m_agent->getCTRNN().randomizeState(min, max);  
#else
  m_agent->getCTRNN().zeroStates();
#endif  
 }
  
//----------------------------------------------------------------------------------------------------------------------  
void SMCAgentEvo::update(float dt) 
{ 
  m_agent->update(dt);
  
  updateFitness(dt);
}
  

//----------------------------------------------------------------------------------------------------------------------
int SMCAgentEvo::getNumGenes()
{
  return m_agent->getTopology().getNumParameters();
}

//----------------------------------------------------------------------------------------------------------------------
void SMCAgentEvo::decodeGenome(const double* genome)
{
  m_agent->getTopology().decode(m_agent->getCTRNN(), genome, m_netLimits);
}

//----------------------------------------------------------------------------------------------------------------------
float SMCAgentEvo::getFitness()
{
  return - sqrt(m_fitness);
}

//----------------------------------------------------------------------------------------------------------------------  
void SMCAgentEvo::toXml(ci::XmlTree& xml)
{
  ci::XmlTree evolvable ("SMCAgent", "");
  evolvable.setAttribute("Fitness", getFitness());
  
  // Let agent output xml
  m_agent->toXml(evolvable);
  
  evolvable.push_back(ci::XmlTree("TrialDuration", toString(m_trialDuration)));
  evolvable.push_back(ci::XmlTree("FitnessEvalDelay", toString(m_fitnessEvalDelay)));  
                      
  m_agent->getTopology().toXml(evolvable);
  
  m_netLimits.toXml(evolvable);
  
  xml.push_back(evolvable);
}

//----------------------------------------------------------------------------------------------------------------------  
void SMCAgentEvo::record(Recorder& recorder)
{
  m_agent->record(recorder);
}

} // namespace