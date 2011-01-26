/*
 *  ofxGARunner.cpp
 *  openDynmx
 *
 *  Created by Thomas Buhrmann on 10/01/2011.
 *  Copyright 2011 __MyCompanyName__. All rights reserved.
 *
 */

#include "GARunner.h"

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
  m_numGenomesTested = 0;
  
  // setup simulation model
  m_evolvable->decodeGenome(m_ga->getCurrentGenome());
  m_evolvable->reset();
}

//----------------------------------------------------------------------------------------------------------------------
void GARunner::update(float dt)
{
  m_time += dt;
  
  if(m_time < m_gaDesc.trialDuration)
  {
    // Update simulation
    m_evolvable->update(dt);
  }
  else
  {
    // End of trial: update GA
    m_accFitness += m_evolvable->getFitness();
    m_trial++;
    
    // Finished all trials? Move on to evaluate next genome.
    if(m_trial == m_gaDesc.numTrials)
    {
      m_accFitness /= m_gaDesc.numTrials;      
      // set the fitness ...
      m_ga->setFitness(m_accFitness);
      // ... and get a new genome to evaluate
      m_evolvable->decodeGenome(m_ga->getCurrentGenome());
      m_trial = 0;
      m_accFitness = 0.0;
      m_numGenomesTested++;     
    }
    
    // Reset simulation for new trial
    m_time = 0.0;
    m_evolvable->reset();
  }
  
//  if(m_ga->getCurrentGeneration() == m_gaDesc.numGenerations)
//  {
//    return false;
//  }  
}

} // namespace dmx
