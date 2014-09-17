/*
 *  LegBac.cpp
 *  cinder_dynmx
 *
 *  Created by Thomas Buhrmann on 12/18/11.
 *  Copyright 2011 __MyCompanyName__. All rights reserved.
 *
 */

#include "LegBac.h"
#include "MathUtils.h"
#include "AdapNN.h"
#include "AdapTop.h"

namespace dmx
{
  
//----------------------------------------------------------------------------------------------------------------------
LegBac::LegBac()
{
  init();
}

//----------------------------------------------------------------------------------------------------------------------
LegBac::~LegBac()
{
  delete m_ctrnn;
}

//----------------------------------------------------------------------------------------------------------------------
void LegBac::init()
{
  const ci::XmlTree* settings = SETTINGS;
  if (settings->hasChild("Config/GA/Evolvable"))
  {
    const ci::XmlTree& xml = settings->getChild("Config/GA/Evolvable");

    m_adaptive = xml.getChild("Topology").getAttributeValue<bool>("adaptive", false);
    if (m_adaptive)
    {
      // Two-phase creation, as the (virtual) fromXML call at construction time cannot reach derived class!
      m_topology = new AdapTop();
      m_topology->fromXml(xml.getChild("Topology"));
      m_ctrnn = new AdapNN(m_topology);
    }
    else
    {
      m_topology = new Topology(xml.getChild("Topology"));
      m_ctrnn = new CTRNN(m_topology->getSize());
    }
    
    m_numLegTrials = xml.getChild("Trials").getAttributeValue<int>("legNum", 0);
    m_numBacTrials = xml.getChild("Trials").getAttributeValue<int>("bacNum", 0);
    m_legDuration = xml.getChild("Trials").getAttributeValue<float>("legDur", 10);
    m_bacDuration = xml.getChild("Trials").getAttributeValue<float>("bacDur", 10);
    float bacFoodDur = xml.getChild("Trials").getAttributeValue<float>("bacFoodDur", 10);
    m_bac.m_foodDur = bacFoodDur;
    
    m_bacFitMax = xml.getChild("Trials").getAttributeValue<float>("bacFitMax", 1);
    m_legFitMax = xml.getChild("Trials").getAttributeValue<float>("legFitMax", 1);
  }
  
  m_legged = m_numLegTrials > 0;
  
}

//----------------------------------------------------------------------------------------------------------------------
void LegBac::reset()
{
  m_time = 0.0f;
  m_fitness = 0;
  
  m_topology->randomiseWeights(m_ctrnn, -4, 4);
  
  if (m_legged)
  {
    m_leg.setNet(m_ctrnn);
    m_leg.reset();
  }
  else
  {
    m_bac.setNet(m_ctrnn);
    m_bac.reset();
  }
}

//----------------------------------------------------------------------------------------------------------------------
void LegBac::update(float dt)
{
  if (m_legged)
  {
    m_leg.update(dt);
  }
  else
  {
    m_bac.update(dt);
    if(m_adaptive)
      ((AdapNN*)m_ctrnn)->setReward(m_bac.getReward());
  }
  
  m_time += dt;
}

//----------------------------------------------------------------------------------------------------------------------
float LegBac::getFitness()
{
  if (m_legged)
    return m_leg.getFinalFitness() / m_legFitMax;
  else
    return m_bac.getFinalFitness() / m_bacFitMax;
}

//----------------------------------------------------------------------------------------------------------------------
void LegBac::nextTrial(int trial)
{
  // Swap bodies ...
  if (m_legged && (trial == m_numLegTrials))
    m_legged = false;
}

//----------------------------------------------------------------------------------------------------------------------
bool LegBac::hasFinished()
{
  if(m_legged)
    return m_time >= m_legDuration;
  else
    return m_time >= m_bacDuration;
}

//----------------------------------------------------------------------------------------------------------------------
void LegBac::endOfEvaluation(float fit)
{
  // Get ready for new evaluation by starting again with legged body
  m_legged = true;
}

//----------------------------------------------------------------------------------------------------------------------
int LegBac::getNumGenes()
{
  return m_topology->getNumParameters();
}

//----------------------------------------------------------------------------------------------------------------------
void LegBac::processGenome(double* genome)
{
  CTRNN ctrnn(m_topology->getSize());
  m_topology->decode(ctrnn, genome);
  m_ctrnn->setCenterCrossing();
  m_topology->encode(ctrnn, genome);
}
  
//----------------------------------------------------------------------------------------------------------------------
void LegBac::decodeGenome(const double* genome)
{
  m_topology->decode(*m_ctrnn, genome);
  
#define TEST_ROUNDTRIP 0
#if TEST_ROUNDTRIP
  //m_ctrnn->setCenterCrossing();
  const int N = m_topology.getNumParameters();
  double newgenome [N];
  m_topology.encode(*m_ctrnn, newgenome, m_netLimits);
  
  std::cout << "Genome round trip: " << std::endl;
  for(int i = 0; i < N; ++i)
    std::cout << i << ": " << genome[i] - newgenome[i] << std::endl;
  std::cout << std::endl;
#endif
}

//----------------------------------------------------------------------------------------------------------------------
void LegBac::toXml(ci::XmlTree& xml)
{
  ci::XmlTree evolvable ("LegBac", "");
  evolvable.setAttribute("Fitness", getFitness());
  
  m_topology->toXml(evolvable);
  m_ctrnn->toXml(evolvable);
  
  xml.push_back(evolvable);
}

//----------------------------------------------------------------------------------------------------------------------
void LegBac::record(Recorder& recorder)
{  
  // ctrnn
  m_ctrnn->record(recorder);
}  
  
} // namespace
