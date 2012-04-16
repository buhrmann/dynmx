/*
 *  SMCAgentEvo.cpp
 *  cinder_dynmx
 *
 *  Created by Thomas Buhrmann on 12/22/11.
 *  Copyright 2011 __MyCompanyName__. All rights reserved.
 *
 */

#include "SMCAgentEvo.h"
#include "CTRNNFactory.h"

namespace dmx
{

//----------------------------------------------------------------------------------------------------------------------  
SMCAgentEvo::SMCAgentEvo()
{
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
  const ci::XmlTree* settings = SETTINGS;
  if (settings->hasChild("Config/GA/Evolvable"))
  {
    // Use setting from globals file
    const ci::XmlTree& xml = settings->getChild("Config/GA/Evolvable");
  
    m_topology.fromXml(xml.getChild("Topology"));
    
    m_agent = new SMCAgent(m_topology.getSize());
    
    m_agent->setMaxSensorDistance(xml.getChild("MaxSensorDist").getValue<double>(1.0));
    m_agent->getDistanceSensor().setTransferFunction(xml.getChild("SensorTransferFunc").getValue<std::string>("Binary"));
    m_agent->setMaxSpeed(xml.getChild("MaxSpeed").getValue<double>(1.0));
    m_agent->setMaxAngularSpeed(degreesToRadians(xml.getChild("MaxAngularSpeed").getValue<double>(180)));
    m_agent->setMaxAngle(degreesToRadians(xml.getChild("MaxAngle").getValue<double>(90)));
    m_agent->setMaxPosition(xml.getChild("MaxPosition").getValue<double>(0.5));
    m_agent->setAngleWraps(xml.getChild("AngleWraps").getValue<bool>(1));
    m_agent->setPositionWraps(xml.getChild("PositionWraps").getValue<bool>(1));
    m_trialDuration = xml.getChild("TrialDuration").getValue<double>(10.0);
    m_fitnessEvalDelay = xml.getChild("FitnessEvalDelay").getValue<double>(8.0);    
    
    // Load environment objects
    m_agent->getEnvironment().fromXml(xml.getChild("Environment"));
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
  
  updateFitness();
}
  

//----------------------------------------------------------------------------------------------------------------------
int SMCAgentEvo::getNumGenes()
{
  return m_topology.getNumParameters();
}

//----------------------------------------------------------------------------------------------------------------------
void SMCAgentEvo::decodeGenome(const double* genome)
{
  // Todo: add to xml!
  CTRNNFactory::DecodeLimits limits;
  limits.tau.set(0.2, 2.0);
  CTRNNFactory::decode(m_agent->getCTRNN(), genome, limits, m_topology);
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
                      
  m_topology.toXml(evolvable);
  
  xml.push_back(evolvable);
}

//----------------------------------------------------------------------------------------------------------------------  
void SMCAgentEvo::record(Recorder& recorder)
{
  m_agent->record(recorder);
}

} // namespace