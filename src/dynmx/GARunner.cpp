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

#define DEFAULT_GA_POPSIZE 50
#define DEFAULT_GA_DEMESIZE 5
#define DEFAULT_GA_MUTATIONMAX 0.01f
#define DEFAULT_GA_RECOMBINATIONRATE 0.05f
#define DEFAULT_GA_NUMTRIALS
#define DEFAULT_GA_

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
  const int numGenes = m_evolvable->getNumGenes();
  
  // Create an instance of the actual GA algorithm
  const ci::XmlTree* settings = SETTINGS;
  if (settings->hasChild("Config/GA"))
  {
    // Use setting from globals file
    const ci::XmlTree& ga = settings->getChild("Config/GA");
    int populationSize = ga.getChild("PopulationSize").getAttributeValue<int>("Value");
    int demeSize = ga.getChild("DemeSize").getAttributeValue<int>("Value");
    
    m_ga = new GA(populationSize, numGenes, demeSize);
    m_ga->setRecombinationRate(ga.getChild("RecombinationRate").getAttributeValue<float>("Value"));
    m_ga->setMutationMax(ga.getChild("MutationMax").getAttributeValue<float>("Value"));
            
    m_numGenerations = ga.getChild("NumGenerations").getAttributeValue<int>("Value");
    m_numTrials = ga.getChild("NumTrials").getAttributeValue<int>("Value");
    m_trialDuration = ga.getChild("TrialDuration").getAttributeValue<float>("Value");
  }
  else
  {
    // If we have no config file, use default values instead
    m_ga = new GA(DEFAULT_GA_POPSIZE, numGenes, DEFAULT_GA_DEMESIZE);
    m_ga->setRecombinationRate(DEFAULT_GA_RECOMBINATIONRATE);
    m_ga->setMutationMax(DEFAULT_GA_MUTATIONMAX);
    m_numTrials = 1;
    m_trialDuration = 1.0f;
    m_numGenerations = 10;
  }

  if(settings->hasChild("Config/Globals"))
  {
    m_verbosity = settings->getChild("Config/Globals/DebugLevel").getAttributeValue<int>("Value");
  }
  else
  {
    m_verbosity = kGAVerbosityNone;
  }
  
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
  
  if(m_time < m_trialDuration)
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
    if (m_trial == m_numTrials)
    {
      m_accFitness /= m_numTrials;
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
