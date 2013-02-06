/*
 *  ofxGARunner.cpp
 *  openDynmx
 *
 *  Created by Thomas Buhrmann on 10/01/2011.
 *  Copyright 2011 __MyCompanyName__. All rights reserved.
 *
 */

#include "GARunner.h"
#include "GATester.h"

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
  m_reducedMutationMax = 0.001;
  m_reduceMutationMaxAt = -1;  // Indicates no automatic reduction of mutation rate

  // For storing data to disk at the end
  m_progressLog = new ci::XmlTree("GAProgress", ""); 
  m_resultsLog = new ci::XmlTree("GAResult", ""); 
  m_finalGenomeLog = new ci::XmlTree("GABestGenome", "");  
  
  // Debug level
  const ci::XmlTree* settings = SETTINGS;  
  if(settings->hasChild("Config/Globals"))
  {
    m_verbosity = settings->getChild("Config/Globals/DebugLevel").getAttributeValue<int>("Value");
  }
  else
  {
    m_verbosity = kGAVerbosityNone;
  }  
  
  const int numGenes = m_evolvable->getNumGenes();
  
  // Create an instance of the actual GA algorithm
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
    
    m_ga->setAvoidReevaluation(ga.getChild("AvoidReevaluation").getAttributeValue<bool>("Value", true));
    
    // Parameters for automatic reduction of mutation rate
    if(ga.hasChild("MutationMaxReduceAt"))
    {
      m_reducedMutationMax = ga.getChild("MutationMaxReduced").getAttributeValue<double>("Value");
      m_reducedRecombinationRate = ga.getChild("RecombinationRateReduced").getAttributeValue<double>("Value");
      m_reduceMutationMaxAt = ga.getChild("MutationMaxReduceAt").getAttributeValue<int>("Value");      
    }
    
    // Pick up from previous run
    bool incremental = ga.getChild("Incremental").getValue<bool>();
    if (incremental)
    {
      std::string fnm = ga.getChild("LoadFrom").getAttributeValue<std::string>("Value");
      if(ga.getChild("Incremental").getAttributeValue<bool>("seedOnly", 0))
      {
        // Read in best individual from last run
        // Convert from path to GA_Result.xml to Best_Genome.xml
        std::string bestFnm = fnm.substr(0, fnm.rfind("/") + 1) + "GA_BestGenome.xml";
        double bestGenome [numGenes];
        double bestFit = readBestGenome(bestFnm, &bestGenome[0], numGenes);
        
        if(ga.getChild("Incremental").getAttributeValue<bool>("seedAll", 1))
        {
          // Randomly distribute population around previous best
          double initialMutMax = ga.getChild("Incremental").getAttributeValue<double>("initialMutation");
          for (int i = 0; i < populationSize; ++i)
          {
            m_ga->setGenome(i, &bestGenome[0], MAX_NEG_FLOAT);
          }
          m_ga->randomise(false, initialMutMax);
          reset(false);
          // elitist: keep best unmutated
          m_ga->setGenome(0, &bestGenome[0], bestFit);
        }
        else
        {
          // Random population with best previous genome as single elitist seed
          reset(true);
          m_ga->setGenome(0, &bestGenome[0], bestFit);
        }
      }
      else
      {
        // Pick up from previous run with whole population and some initial jitter
        double initialMutMax = ga.getChild("Incremental").getAttributeValue<double>("initialMutation");
        ci::XmlTree prevResults(ci::loadFile(fnm));
        m_ga->fromXml(prevResults);
        m_ga->randomise(false, initialMutMax);
        reset(false);
      }
    }
    else
    {
      reset(true);
    }
    
    // What to save out in xml
    m_saveBestEachGen = false;
    if(ga.hasChild("SaveBestEachGeneration"))
    {
      m_saveBestEachGen = ga.getChild("SaveBestEachGeneration").getAttributeValue<bool>("Value"); 
    }

    // Run evaluation of best individual at end of evolution
    m_autoEval = ga.hasChild("AutoEval") ? (ga / "AutoEval")["Run"].as<bool>() : false; 

  }
  else
  {
    // If we have no config file, use default values instead
    m_ga = new GA(DEFAULT_GA_POPSIZE, numGenes, DEFAULT_GA_DEMESIZE);
    m_ga->setRecombinationRate(DEFAULT_GA_RECOMBINATIONRATE);
    m_ga->setMutationMax(DEFAULT_GA_MUTATIONMAX);
    m_numTrials = 1;
    m_numGenerations = 10;
    reset(true);    
  }

  // Saves these out, to make it easier to parse this data by generation etc.
  m_progressLog->setAttribute("NumGenerations", m_numGenerations);
  m_progressLog->setAttribute("NumGenes", numGenes);
}
  
//----------------------------------------------------------------------------------------------------------------------
void GARunner::reset(bool randomiseGenomes)
{
  m_time = 0.0f;
  m_accFitness = 0.0f;
  m_trial = 0;
  m_prevGeneration = 0;
  
  m_ga->reset(randomiseGenomes);
  
  // setup simulation model
  m_evolvable->decodeGenome(m_ga->getCurrentGenome());
  m_evolvable->reset();  
}

//----------------------------------------------------------------------------------------------------------------------
void GARunner::update(float dt)
{
  m_time += dt;
  
  if(!m_evolvable->hasFinished())
  {
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
    m_time = 0.0;
    
    m_evolvable->reset();        
    m_evolvable->nextTrial(m_trial);
    
    // Finished all trials? Move on to evaluate next genome.
    //-------------------------------------------------------
    if (m_trial == m_numTrials)
    {
      m_accFitness /= m_numTrials;
      
#if DEBUGGING
      if (m_verbosity >= kGAVerbosityGenome)
        std::cout << "Total fitness = " << m_accFitness << std::endl; 
#endif      
      
      // Set the fitness ...
      m_evolvable->endOfEvaluation(m_accFitness);
      m_ga->setFitness(m_accFitness);
      
      // ... and get a new genome to evaluate
      m_evolvable->decodeGenome(m_ga->getCurrentGenome());
      
      // Reset simulation for new trial
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
        
        // Store fitness for later analysis and potentially the best genome
        generationToXml(m_progressLog, currentGen, bestGenome, bestFitness, avgFitness);
        
#if DEBUGGING
        if (m_verbosity >= kGAVerbosityPopulation)
        {
          std::cout << "Generation " << currentGen 
                    << ": BestFit = " << bestFitness << " | AvgFit = " << avgFitness << std::endl; 
        }
#endif     
        m_prevGeneration = currentGen;
        
        // Automatic reduction of mutation rate
        if(m_reduceMutationMaxAt > 0 && currentGen == m_reduceMutationMaxAt)
        {
          m_ga->setMutationMax(m_reducedMutationMax);
          m_ga->setRecombinationRate(m_reducedRecombinationRate);
        }
        
        // When the maximum number of generations has been reached. 
        //---------------------------------------------------------
        if (hasFinished())
        { 
          // Write results to file
          getGA()->toXml(*m_resultsLog, true);
          m_resultsLog->write(ci::writeFile(dmx::DATA_DIR + "GA_Result.xml"));
          m_progressLog->write(ci::writeFile(dmx::DATA_DIR + "GA_Progress.xml"));
          
          genomeToXml(*m_finalGenomeLog, bestGenome, bestFitness);
          m_finalGenomeLog->write(ci::writeFile(dmx::DATA_DIR + "GA_BestGenome.xml"));
          
          // Evaluate best evolved individual
          if(m_autoEval)
          {
            test(bestGenome, bestFitness, dt);
          }

#if DEBUGGING
          // Output number of evaluations performed (handy for calculating speed of simulation; divide overall time by)
          const int numEvaluations = currentGen * m_ga->getPopulationSize() * m_numTrials;
          if (m_verbosity >= kGAVerbosityPopulation)
          {
            std::cout << "GA finished. Best fitness: " << bestFitness << ". Evaluated " <<  numEvaluations << " trials." << std::endl;
          }
#endif             
        }
      }
    }
    
  }
}


//----------------------------------------------------------------------------------------------------------------------
void GARunner::test(const double* genome, float fitness, float dt)
{
  m_evolvable->reset();
  m_evolvable->decodeGenome(genome);
  GATester tester (m_evolvable);
  // We're only doing this to record the state data! So overwrite any flag read in from xml config.
  tester.enableRecording(true);
  genomeToXml(tester.getXml(), genome, fitness);
  while(!tester.hasFinished())
  {
    tester.update(dt);
  }
}

//----------------------------------------------------------------------------------------------------------------------  
void GARunner::genomeToXml(ci::XmlTree& xml, const double* genome, float fitness)
{
  ci::XmlTree genXml ("Genome","");
  genXml.setAttribute("Fitness", fitness);
  genXml.setAttribute("NumGenes", m_ga->getGenomeSize());
  for(size_t i = 0; i < m_ga->getGenomeSize(); ++i)
  {
    ci::XmlTree gene ("Gene", "");
    gene.setAttribute("Index", i);
    gene.setAttribute("Value", genome[i]);
    genXml.push_back(gene);
  }
  xml.push_back(genXml);
}
  
// Write a generation's best individual to xml 
//----------------------------------------------------------------------------------------------------------------------
void GARunner::generationToXml(ci::XmlTree* xmlTree, uint32_t gen, const double* bestGenome, float bestFitness, float avgFitness)
{
  assert(xmlTree != 0);
  
  // Ouput Fitness
  ci::XmlTree generation ("Generation", "");
  generation.setAttribute("Index", gen); 
  generation.setAttribute("BestFitness", bestFitness);
  generation.setAttribute("AverageFitness", avgFitness);

  // Output best genome
  if(m_saveBestEachGen)
  {
    genomeToXml(generation, bestGenome, bestFitness);
  }
  
  xmlTree->push_back(generation);
}

} // namespace dmx
