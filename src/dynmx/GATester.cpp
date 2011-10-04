/*
 *  GATester.cpp
 *  cinder_dynmx
 *
 *  Created by Thomas Buhrmann on 9/16/11.
 *  Copyright 2011 __MyCompanyName__. All rights reserved.
 *
 */

#include "GATester.h"

#if DEBUGGING
#include <iostream>
#endif

namespace dmx
{

//----------------------------------------------------------------------------------------------------------------------
GATester::GATester(Evolvable* evolvable) : m_evolvable(evolvable)
{
  init();
}

//----------------------------------------------------------------------------------------------------------------------
void GATester::init()
{
  // Debug level
  const ci::XmlTree* settings = SETTINGS;  
  if(settings->hasChild("Config/Globals"))
  {
    m_verbosity = settings->getChild("Config/Globals/DebugLevel").getAttributeValue<int>("Value");
  }
  else
  {
    m_verbosity = GARunner::kGAVerbosityNone;
  }  

  // Load info about trials etc
  if (SETTINGS->hasChild("Config/GA/Eval"))
  {
    const ci::XmlTree& ga = settings->getChild("Config/GA/Eval");
    m_numTrials = ga.getChild("NumTrials").getAttributeValue<int>("Value");
  }   
  
  // Load best genome from prev GA run
  if(SETTINGS->hasChild("Config/GA/Eval/LoadFrom"))
  {
    std::string fnm = SETTINGS->getChild("Config/GA/Eval/LoadFrom").getAttributeValue<std::string>("Value");
    ci::XmlTree gaProgress(ci::loadFile(fnm));
    assert(gaProgress.hasChild("GAProgress"));
    
    // Iterate to last generation (the best individual)
    ci::XmlTree::ConstIter lastGeneration = gaProgress.begin("GAProgress/Generation");
    for (ci::XmlTree::ConstIter generation = gaProgress.begin("GAProgress/Generation"); generation != gaProgress.end(); ++generation)
    {
      lastGeneration = generation;
    }

    // Extract the genome
    const ci::XmlTree& genome = lastGeneration->getChild("BestGenome");
    
    // Convert to double array
    int numGenes = gaProgress.getChild("GAProgress").getAttributeValue<int>("NumGenes");  
    double genes[numGenes];
    int i = 0;
    for (ci::XmlTree::ConstIter gene = genome.begin(); gene != genome.end(); ++gene)
    {
      const double d = gene->getAttributeValue<float>("Value");
      genes[i] = d;
      i++;
    }
    
    // Decode
    m_evolvable->decodeGenome(&genes[0]);  
  }
  
  reset();
}

//----------------------------------------------------------------------------------------------------------------------
void GATester::reset()
{
  m_time = 0.0f;
  m_accFitness = 0.0f;
  m_trial = 0;
  m_evolvable->reset();  
}

//----------------------------------------------------------------------------------------------------------------------
void GATester::update(float dt)
{
  m_time += dt;
  
  if(!m_evolvable->hasFinished())
  {
    // Update simulation
    m_evolvable->update(dt);
  }
  else
  {
    // End of trial: update GA
    const float fitness = m_evolvable->getFitness();
#if DEBUGGING
    if( m_verbosity >= GARunner::kGAVerbosityTrial)
      std::cout << "Trial " << m_trial << ": fitness = " << fitness << std::endl; 
#endif
    
    m_accFitness += fitness;
    m_trial++;
    
    // Finished all trials? Move on to evaluate next genome.
    if (m_trial == m_numTrials)
    {
      m_accFitness /= m_numTrials;
#if DEBUGGING
      if( m_verbosity >= GARunner::kGAVerbosityGenome)
        std::cout << "Total fitness = " << m_accFitness << std::endl; 
#endif      
    }
    
    // Reset simulation for new trial
    m_time = 0.0;
    m_evolvable->reset();
  }
}

  
} // namespace dmx
