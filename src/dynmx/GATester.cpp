/*
 *  GATester.cpp
 *  cinder_dynmx
 *
 *  Created by Thomas Buhrmann on 9/16/11.
 *  Copyright 2011 __MyCompanyName__. All rights reserved.
 *
 */

#include "GATester.h"
#include "MathUtils.h"

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
  m_idum = (long)-time(0);	// "randomly" seed random number generator !
  
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
    
    std::string trialAgg = settings->getChild("Config/GA/Trials").getAttributeValue<std::string>("Combine", "Avg");
    if(trialAgg == "Avg")
      m_trialAggregation = GARunner::kGATrialAgg_Avg;
    else if(trialAgg == "Mult")
      m_trialAggregation = GARunner::kGATrialAgg_Mult;
    else if(trialAgg == "Min")
      m_trialAggregation = GARunner::kGATrialAgg_Min;
    
    // Flag whether to record state during run
    if (eval.hasChild("RecordState"))
    {
      m_record = (eval / "RecordState")["Value"].as<bool>();
    }
    else 
    {
      m_record = false;
    }
    
    // Mutations
    m_mutateMin = eval.getAttributeValue<double>("MutateMin", 0);
    m_mutateMax = eval.getAttributeValue<double>("MutateMax", 0);
    m_mutate = m_mutateMax > m_mutateMin;
    if(m_mutate)
    {
      m_mutateStep = eval.getAttributeValue<double>("MutateStep", 0);
      int mutationSteps = (m_mutateMax - m_mutateMin) / m_mutateStep;
      if(m_numTrials >= mutationSteps)
        m_numEvalsPerMutation = m_numTrials / mutationSteps;
      else
        m_numEvalsPerMutation = 1;
    }

    // Load best genome from prev GA run, but only if this is a "Run" (not e.g. in GARunner automatic testing)
    bool decodeSaved = eval["Run"].as<bool>();    
    if(decodeSaved && eval.hasChild("LoadFrom"))
    {
      std::string fnm = (eval / "LoadFrom")["Value"];
      fnm = pathExpandHome(fnm);
      ci::XmlTree bestGenomeXml(ci::loadFile(fnm));
      assert(bestGenomeXml.hasChild("GABestGenome"));
      
      // Extract the genome
      const ci::XmlTree& genomeXml = bestGenomeXml / "GABestGenome/Genome";
      
      // Convert to double array
      m_numGenes = genomeXml["NumGenes"].as<int>();
      m_genome = new double[m_numGenes];
      int i = 0;
      for (ci::XmlTree::ConstIter gene = genomeXml.begin(); gene != genomeXml.end(); ++gene)
      {
        const double d = gene->getAttributeValue<float>("Value");
        m_genome[i] = d;
        i++;
      }
      
      double* genome = new double [m_numGenes];
      std::copy(m_genome, m_genome + m_numGenes, genome);
      
      // Mutate
      if(m_mutate)
      {
        mutate(genome, m_numGenes, m_mutateMin);
      }
      
      // Decode
      m_evolvable->decodeGenome(genome);
      
      delete [] genome;
      
      // Store genome in xml file, even if not read here. It could have been decoded pre
      m_modelXml->push_back(ci::XmlTree(genomeXml));
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
    m_fitnessLog.push_back("trial", m_trial);
    m_fitnessLog.push_back("fitness", fitness);
    
    if(m_trialAggregation == GARunner::kGATrialAgg_Avg)
    {
      m_accFitness += fitness;
    }
    else if(m_trialAggregation == GARunner::kGATrialAgg_Mult)
    {
      m_accFitness = m_trial == 0 ? fitness : (m_accFitness * fitness);
    }
    else if(m_trialAggregation == GARunner::kGATrialAgg_Min)
    {
      m_accFitness = m_trial == 0 ? fitness : std::min(m_accFitness, fitness);
    }
    m_trial++;
    
    // Finished all trials? Move on to evaluate next genome.
    if (m_trial == m_numTrials)
    {
      if(m_trialAggregation == GARunner::kGATrialAgg_Avg)
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
      
      // Write fitness log
      m_fitnessLog.saveTo(dmx::DATA_DIR + "Fitness.txt");
    }
    
    // Reset simulation for new trial. Do reset here, at end, so the writing out can happen before, when actual
    // values still persist.

    if(m_mutate)
    {
      double* genome = new double [m_numGenes];
      std::copy(m_genome, m_genome + m_numGenes, genome);
      int step = m_trial / m_numEvalsPerMutation;
      double mutSize = m_mutateMin + step * m_mutateStep;
      mutate(genome, m_numGenes, mutSize);
      m_evolvable->decodeGenome(genome);
      delete [] genome;
    }
    
    m_time = 0.0;
    m_evolvable->reset();
    m_evolvable->nextTrial(m_trial);
  }
  
}
// --------------------------------------------------------------------------------------------
void GATester::mutate(double* genome, int numGenes, double maxMut)
{
  for (int i = 0; i < numGenes; i++)
  {
    const double mutation = (-1.0 + 2.0 * ran1(&m_idum)) * maxMut;
    
    genome[i] += mutation;
    
    // ensure values stay in [0,1] range
    if (genome[i] > 1)
    {
      genome[i] = 2.0 - genome[i];
    }
    else if (genome[i] < 0)
    {
      genome[i] *=  -1.0;
    }
  }
}
  
} // namespace dmx
