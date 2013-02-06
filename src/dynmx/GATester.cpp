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
  
  // For storing evolvable to disk at the end
  m_modelXml = new ci::XmlTree("Evolvable", ""); 
  
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
  if (settings->hasChild("Config/GA/Eval"))
  {
    const ci::XmlTree& eval = settings->getChild("Config/GA/Eval");
    
    m_numTrials = (eval / "NumTrials")["Value"].as<int>();
    
    // Flag whether to record state during run
    if (eval.hasChild("RecordState"))
    {
      m_record = (eval / "RecordState")["Value"].as<bool>();
    }
    else 
    {
      m_record = false;
    }

  
    // Load best genome from prev GA run, but only if this is a "Run" (not e.g. in GARunner automatic testing)
    bool decodeSaved = eval["Run"].as<bool>();    
    if(decodeSaved && eval.hasChild("LoadFrom"))
    {
      std::string fnm = (eval / "LoadFrom")["Value"];
      ci::XmlTree bestGenomeXml(ci::loadFile(fnm));
      assert(bestGenomeXml.hasChild("GABestGenome"));
      
      // Extract the genome
      const ci::XmlTree& genome = bestGenomeXml / "GABestGenome/Genome";
      
      // Convert to double array
      int numGenes = genome["NumGenes"].as<int>();  
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
      
      // Store genome in xml file, even if not read here. It could have been decoded pre
      m_modelXml->push_back(ci::XmlTree(genome));
    }
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
    
    // Record state
    if(m_record)
    {
      m_evolvable->record(m_recorder);
    }
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
    
    // Write model data to xml
    if(hasFinished())
    {
      // Store description of evolvable
      m_evolvable->toXml(*m_modelXml);
      m_modelXml->write(ci::writeFile(dmx::DATA_DIR + "Evolvable.xml"));  
      
      // Store state data
      if(m_record)
      {
        m_recorder.saveTo(dmx::DATA_DIR + "State.txt");
      }
    }
    
    // Reset simulation for new trial. Do reset here, at end, so the writing out can happen before, when actual
    // values still persist.
    m_time = 0.0;
    m_evolvable->reset();
    m_evolvable->nextTrial(m_trial);
  }
  
}

  
} // namespace dmx
