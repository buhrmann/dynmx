/*
 *  ofxGARunner.cpp
 *  openDynmx
 *
 *  Created by Thomas Buhrmann on 10/01/2011.
 *  Copyright 2011 __MyCompanyName__. All rights reserved.
 *
 */

#include "GARunner.h"
#if DEBUGGING
#include <iostream>
#endif

namespace dmx
{

//----------------------------------------------------------------------------------------------------------------------
GARunner::GARunner(Evolvable* evolvable) : m_evolvable(evolvable)
{
  init();
}

//----------------------------------------------------------------------------------------------------------------------
void GARunner::init()
{
  const ci::XmlTree* settings = SETTINGS;
  if (settings->hasChild("Config/GA"))
  {
    const ci::XmlTree& ga = settings->getChild("Config/GA");
    m_gaDesc.populationSize = ga.getChild("PopulationSize").getAttributeValue<int>("Value");
  }

  m_verbosity = kGAVerbosityNone;
  
  // load GA desciptor from file ...
  // m_gaDesc.load("");
  
  // create GA
  const int numGenes = m_evolvable->getNumGenes();
  m_ga = new GA(m_gaDesc.populationSize, numGenes, m_gaDesc.demeSize);
  // setup mutation parameters etc..
  // m_ga.set(...
  
  m_time = 0.0f;
  m_accFitness = 0.0f;
  m_trial = 0;
  m_prevGeneration = 0;
  
  // setup simulation model
  m_evolvable->decodeGenome(m_ga->getCurrentGenome());
  m_evolvable->reset();
  
  m_progressLog = new ci::XmlTree("GAProgress", ""); 
  m_resultsLog = new ci::XmlTree("GAResult", "");
}

//----------------------------------------------------------------------------------------------------------------------
void GARunner::update(float dt)
{
  m_time += dt;
  
  if(m_time < m_gaDesc.trialDuration)
  {
    // Update simulation
    //-------------------------------------------------------
    m_evolvable->update(dt);
  }
  else
  {
    // End of trial: update GA
    //-------------------------------------------------------
    const float fitness = m_evolvable->getFitness();
#if DEBUGGING
    if( m_verbosity >= kGAVerbosityTrial)
      std::cout << "Trial " << m_trial << ": fitness = " << fitness << std::endl; 
#endif
    
    m_accFitness += fitness;
    m_trial++;
    
    // Finished all trials? Move on to evaluate next genome.
    //-------------------------------------------------------
    if (m_trial == m_gaDesc.numTrials)
    {
      m_accFitness /= m_gaDesc.numTrials;
#if DEBUGGING
      if( m_verbosity >= kGAVerbosityGenome)
        std::cout << "Total fitness = " << m_accFitness << std::endl; 
#endif      
      
      // Set the fitness ...
      m_ga->setFitness(m_accFitness);
      // ... and get a new genome to evaluate
      m_evolvable->decodeGenome(m_ga->getCurrentGenome());
      m_trial = 0;
      m_accFitness = 0.0;
      
      // End of one "generation"
      //-------------------------------------------------------
      const uint32_t currentGen = m_ga->getCurrentGeneration();
      if (m_prevGeneration != currentGen)
      {
        float bestFitness;
        const double* bestGenome = m_ga->getBestGenome(bestFitness);
        const float avgFitness = m_ga->getAvgFitness();  
        
        generationToXml(m_progressLog, currentGen, bestGenome, bestFitness, avgFitness);
        
#if DEBUGGING
        if (m_verbosity >= kGAVerbosityPopulation)
        {
          std::cout << "Generation " << currentGen 
                    << ": BestFit = " << bestFitness << " | AvgFit = " << avgFitness << std::endl; 
        }
#endif     
        m_prevGeneration = currentGen;
        
        // When the maximum number of generations has been reached. 
        //---------------------------------------------------------
        if (hasFinished())
        { 
          // Write results to file
          getGA()->toXml(*m_resultsLog, true);
          m_resultsLog->write(ci::writeFile(dmx::DATA_DIR + "GA_Result.xml"));
          m_progressLog->write(ci::writeFile(dmx::DATA_DIR + "GA_Progress.xml"));  
        }
      }
    }
    
    // Reset simulation for new trial
    //-------------------------------------------------------
    m_time = 0.0;
    m_evolvable->reset();
  }
}

// Write a generation's best individual to xml 
//----------------------------------------------------------------------------------------------------------------------
void GARunner::generationToXml(ci::XmlTree* xmlTree, uint32_t gen, const double* bestGenome, float bestFitness, float avgFitness)
{
  assert(xmlTree != 0);
  
  ci::XmlTree generation ("Generation", "");
  generation.setAttribute("Index", gen); 
  generation.setAttribute("BestFitness", bestFitness);
  generation.setAttribute("AverageFitness", avgFitness);
  ci::XmlTree genome ("BestGenome", "");
  
  for(size_t i = 0; i < m_ga->getGenomeSize(); ++i)
  {
    ci::XmlTree gene ("Gene", "");
    gene.setAttribute("Index", i);
    gene.setAttribute("Value", bestGenome[i]);
    genome.push_back(gene);
  }
  
  generation.push_back(genome);
  xmlTree->push_back(generation);
}

} // namespace dmx
