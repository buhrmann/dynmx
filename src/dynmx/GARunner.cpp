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
  m_reducedMutationVar = 0.001;
  m_reduceMutationAt = -1;  // Indicates no automatic reduction of mutation rate

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
    int populationSize = ga.getChild("PopulationSize").getValue<int>(50);
    int demeSize = ga.getChild("DemeSize").getValue<int>(10);
    
    m_ga = new GA(populationSize, numGenes, demeSize);

    m_ga->setRecombinationRate(ga.getChild("Recombination").getAttributeValue<float>("Rate", 0.5));
    m_ga->setMutationVar(ga.getChild("Mutation").getAttributeValue<float>("Var", 0.1));
    m_ga->setMutationRate(ga.getChild("Mutation").getAttributeValue<float>("Rate", 1.0));
    
    std::string pdname = (ga / "Mutation" / "IntraGenomePD").getValue<std::string>("Gaussian");
    GA::GAProbDist pd = pdname == "Gaussian" ? GA::kPD_Gaussian : GA::kPD_Uniform;
    m_ga->setIntraGenomeMutPD(pd);
    
    pdname = (ga / "Mutation" / "InterGenomePD").getValue<std::string>("Gaussian");
    pd = pdname == "Gaussian" ? GA::kPD_Gaussian : GA::kPD_Uniform;
    m_ga->setInterGenomeMutPD(pd);
    
    m_reduceMutationAt = ga.getChild("Mutation").getAttributeValue<int>("ReduceAt");
    m_reducedMutationVar = ga.getChild("Mutation").getAttributeValue<double>("VarReduced");
    m_reducedRecombinationRate = ga.getChild("Recombination").getAttributeValue<double>("RateReduced");
    
    m_numGenerations = ga.getChild("NumGenerations").getValue<int>(100);
    m_outputInterval = ga.getChild("NumGenerations").getAttributeValue<int>("OutputInterval", 100);
    m_numTrials = ga.getChild("Trials").getValue<int>(1);
    
    std::string trialAgg = ga.getChild("Trials").getAttributeValue<std::string>("Combine", "Avg");
    if(trialAgg == "Avg")
      m_trialAggregation = kGATrialAgg_Avg;
    else if(trialAgg == "Mult")
      m_trialAggregation = kGATrialAgg_Mult;
    else if(trialAgg == "Min")
      m_trialAggregation = kGATrialAgg_Min;
    
    m_ga->setAvoidReevaluation(ga.getChild("AvoidReevaluation").getValue<bool>(true));
    
    if (ga.hasChild("Stages"))
    {
      m_stageFitThreshold = (ga / "Stages").getAttributeValue<float>("FitThreshold", 1);
      m_genFitBufferSize = (ga / "Stages").getAttributeValue<int>("NumGensAbove", 0);
      m_genFitBuffer.reserve(m_genFitBufferSize);
      m_genFitBuffer.resize(m_genFitBufferSize, 0);
    }
    
    // Pick up from previous run
    bool incremental = ga.getChild("Incremental").getValue<bool>();
    if (incremental)
    {
      std::string fnm = ga.getChild("LoadFrom").getValue<std::string>();
      fnm = pathExpandHome(fnm);
      
      if(ga.getChild("Incremental").getAttributeValue<bool>("seedOnly", 0))
      {
        // Read in best individual from last run
        // Convert from path to GA_Result.xml to Best_Genome.xml
        std::string bestFnm = fnm.substr(0, fnm.rfind("/") + 1) + "GA_BestGenome.xml";
        double bestGenome [numGenes];
        double bestFit = readBestGenome(bestFnm, &bestGenome[0], numGenes);
        (void) bestFit;
        
        if(ga.getChild("Incremental").getAttributeValue<bool>("seedAll", 1))
        {
          // Randomly distribute population around previous best
          double initialMutVar = ga.getChild("Incremental").getAttributeValue<double>("initialMutation");
          for (int i = 0; i < populationSize; ++i)
          {
            m_ga->setGenome(i, &bestGenome[0], MAX_NEG_FLOAT);
          }
          m_ga->randomise(false, initialMutVar);
          reset(false);
          
          // Elitist: keep best unmutated
          // In case fitness function changes, have to evaluate best again
          if(ga.getChild("Incremental").getAttributeValue<bool>("elitist", 0))
            m_ga->setGenome(0, &bestGenome[0], MAX_NEG_FLOAT);
        }
        else
        {
          // Random population with best previous genome as single elitist seed
          reset(true);
          m_ga->setGenome(0, &bestGenome[0], MAX_NEG_FLOAT);
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
    m_saveBestEachGen = ga.hasChild("SaveBestEachGeneration") ? (ga / "SaveBestEachGeneration").getValue<bool>() : false;

    // Run evaluation of best individual at end of evolution
    m_autoEval = ga.hasChild("AutoEval") ? (ga / "AutoEval").getValue<bool>(false) : false;

  }
  else
  {
    // If we have no config file, use default values instead
    m_ga = new GA(DEFAULT_GA_POPSIZE, numGenes, DEFAULT_GA_DEMESIZE);
    m_ga->setRecombinationRate(DEFAULT_GA_RECOMBINATIONRATE);
    m_ga->setMutationVar(DEFAULT_GA_MUTATIONMAX);
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
  m_stage = 0;
  
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
    // End of trial
    //-------------------------------------------------------
    const float fitness = m_evolvable->getFitness();
   
    updateTrialFitness(fitness);
    
    m_trial++;
    m_time = 0.0;
    
    // Finished all trials? Move on to evaluate next genome.
    //-------------------------------------------------------
    if (m_trial == m_numTrials)
    {
      
#if DEBUGGING
      if (m_verbosity >= kGAVerbosityGenome)
        std::cout << "Total fitness = " << m_accFitness << std::endl; 
#endif      
      
      // Set the fitness in the GA ...
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
        finishGeneration(currentGen, dt);
      }
      
    } // End of indiviual's evaluation (all trials)
    
    m_evolvable->reset();
    m_evolvable->nextTrial(m_trial);
  } // end of trial
}

//----------------------------------------------------------------------------------------------------------------------
void GARunner::updateTrialFitness(float fitness)
{
  if(m_trialAggregation == kGATrialAgg_Avg)
  {
    m_accFitness += (fitness / m_numTrials);
  }
  else if(m_trialAggregation == kGATrialAgg_Mult)
  {
    m_accFitness = m_trial == 0 ? fitness : (m_accFitness * fitness);
  }
  else if(m_trialAggregation == kGATrialAgg_Min)
  {
    m_accFitness = m_trial == 0 ? fitness : std::min(m_accFitness, fitness);
  }

#if DEBUGGING
  if( m_verbosity >= kGAVerbosityTrial)
    std::cout << "Trial " << m_trial << ": fitness = " << fitness << std::endl;
#endif
}
  
//----------------------------------------------------------------------------------------------------------------------
void GARunner::finishGeneration(int currentGen, float dt)
{
  float bestFitness;
  const double* bestGenome = m_ga->getBestGenome(bestFitness);
  const float avgFitness = m_ga->getAvgFitness();
  
  // Store fitness for later analysis and potentially the best genome
  generationToXml(m_progressLog, currentGen, bestGenome, bestFitness, avgFitness);
  
#if DEBUGGING
  if (m_verbosity >= kGAVerbosityPopulation)
  {
    std::cout << "Generation " << currentGen << " (Stage " << m_stage << " )"
    << ": BestFit = " << bestFitness
    << " | AvgFit = " << avgFitness << std::endl;
  }
  else if(m_verbosity == kGAVerbosityNone && currentGen % m_outputInterval == 0)
  {
    std::string dir = GLOBALS->getDataDirName();
    std::cout.precision(5);
    std::cout.width(5);
    std::cout.fill(5);
    std::cout << dir << " | G " << currentGen << " (S " << m_stage << " )"
              << " | B = " << bestFitness
              << "\t A = " << avgFitness << std::endl;
  }
#endif
  
  // Check whether next fitness stage is due and tell evolvable if needed
  updateFitnessStage(currentGen, bestFitness);
  
  m_prevGeneration = currentGen;
  
  // Automatic reduction of mutation rate
  if(currentGen == m_reduceMutationAt)
  {
    m_ga->setMutationVar(m_reducedMutationVar);
    m_ga->setRecombinationRate(m_reducedRecombinationRate);
  }
  
  // When the maximum number of generations has been reached.
  //---------------------------------------------------------
  if (hasFinished())
  {
    finishRun(bestGenome, bestFitness, dt);
  }
}

// Finish the evolutionary run (writes results to file etc.)
//----------------------------------------------------------------------------------------------------------------------
void GARunner::finishRun(const double* bestGenome, float bestFitness, float dt)
{
  // Write results to file
  getGA()->toXml(*m_resultsLog, true);
  m_resultsLog->write(ci::writeFile(dmx::DATA_DIR + "GA_Result.xml"));
  m_progressLog->write(ci::writeFile(dmx::DATA_DIR + "GA_Progress.xml"));
  
  genomeToXml(*m_finalGenomeLog, bestGenome, m_ga->getGenomeSize(), bestFitness);
  m_finalGenomeLog->write(ci::writeFile(dmx::DATA_DIR + "GA_BestGenome.xml"));
  
  // Evaluate best evolved individual
  if(m_autoEval)
  {
    test(bestGenome, bestFitness, dt);
  }
  
#if DEBUGGING
  // Output number of evaluations performed (handy for calculating speed of simulation; divide overall time by)
  const int numEvaluations = m_ga->getCurrentGeneration() * m_ga->getPopulationSize() * m_numTrials;
  if (m_verbosity >= kGAVerbosityPopulation)
  {
    std::cout << "GA finished. Best fitness: " << bestFitness << " in fitness stage " << m_stage << ". Evaluated " <<  numEvaluations << " trials." << std::endl;
  }
#endif
}

// Check if next fitness stage should be initiated??
//----------------------------------------------------------------------------------------------------------------------
void GARunner::updateFitnessStage(int currentGen, float bestFit)
{
  if(m_genFitBufferSize > 0)
  {
    m_genFitBuffer[(currentGen - 1) % m_genFitBufferSize] = bestFit;
    bool nextStage = true;
    for ( auto &f : m_genFitBuffer ) {
      nextStage &= (f > m_stageFitThreshold);
    }
    
    if(nextStage)
    {
      m_stage++;
      m_evolvable->nextStage(m_stage);
      std::fill(m_genFitBuffer.begin(), m_genFitBuffer.end(), 0); // reset buffer
      fitnessFunctionChanged();
    }
  }
}

//----------------------------------------------------------------------------------------------------------------------
void GARunner::test(const double* genome, float fitness, float dt)
{
  m_evolvable->reset();
  m_evolvable->nextTrial(0);
  m_evolvable->decodeGenome(genome);
  GATester tester (m_evolvable);
  // We're only doing this to record the state data! So overwrite any flag read in from xml config.
  tester.enableRecording(true);
  genomeToXml(tester.getXml(), genome, m_ga->getGenomeSize(), fitness);
  while(!tester.hasFinished())
  {
    tester.update(dt);
  }
}

//----------------------------------------------------------------------------------------------------------------------  
void GARunner::genomeToXml(ci::XmlTree& xml, const double* genome, int numGenes, float fitness)
{
  ci::XmlTree genXml ("Genome","");
  genXml.setAttribute("Fitness", fitness);
  genXml.setAttribute("NumGenes", numGenes);
  for(size_t i = 0; i < numGenes; ++i)
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
  generation.setAttribute("Stage", m_stage);

  // Output best genome
  if(m_saveBestEachGen)
  {
    genomeToXml(generation, bestGenome, m_ga->getGenomeSize(), bestFitness);
  }
  
  xmlTree->push_back(generation);
}

} // namespace dmx
