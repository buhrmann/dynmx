/*
 *  ofxGARunner.h
 *  openDynmx
 *
 *  Created by Thomas Buhrmann on 10/01/2011.
 *  Copyright 2011 __MyCompanyName__. All rights reserved.
 *
 */

#ifndef _GA_RUNNER_
#define _GA_RUNNER_

#include "Model.h"
#include "GA.h"
#include "Recorder.h"

#include "cinder/xml.h"

#include <assert.h>

namespace dmx
{

//----------------------------------------------------------------------------------------------------------------------
// Interface for an evolvable model
//----------------------------------------------------------------------------------------------------------------------
class Evolvable : public Model
{
public:
  virtual int getNumGenes() = 0;
  virtual void decodeGenome(const double* genome) = 0;
  virtual void processGenome(double* genome) {};
  virtual float getFitness() = 0;
  virtual void nextTrial(int trial = 0) {};
  virtual void nextStage(int stage){};
  virtual void endOfEvaluation(float fit) {};

  // Inherited from Model
  // An evolvable can just return  its fitness explicitly, so doesn't necessarily need an update etc...
  virtual void update(float dt) {};
  virtual void init() {};
  virtual void reset() {};
  
  virtual void toXml(ci::XmlTree& xml) {};
  virtual void record(Recorder& recorder) {};
};

//----------------------------------------------------------------------------------------------------------------------
static double readBestGenome(const std::string& fnm, double* retGenome, int genomeLength)
{
  ci::XmlTree bestGenomeXml(ci::loadFile(fnm));
  assert(bestGenomeXml.hasChild("GABestGenome"));
  
  // Extract the genome
  const ci::XmlTree& genome = bestGenomeXml / "GABestGenome/Genome";
  
  // Convert to double array
  int numGenes = genome["NumGenes"].as<int>();
  assert(numGenes == genomeLength);
  int i = 0;
  for (ci::XmlTree::ConstIter gene = genome.begin(); gene != genome.end(); ++gene)
  {
    const double d = gene->getAttributeValue<float>("Value");
    retGenome[i] = d;
    i++;
  }

  return genome.getAttributeValue<double>("Fitness", MAX_NEG_FLOAT);
}


//----------------------------------------------------------------------------------------------------------------------
// Interface for apps running a GA
//----------------------------------------------------------------------------------------------------------------------
class GARunner : public Model
{
  
public:
  
  enum GAVerbosity
  {
    kGAVerbosityNone,
    kGAVerbosityPopulation,    
    kGAVerbosityGenome,    
    kGAVerbosityTrial
  };
  
  enum GATrialAgg
  {
    kGATrialAgg_Avg,
    kGATrialAgg_Min,
    kGATrialAgg_Mult
  };
  
  GARunner(Evolvable* evolvable);
  
  // Inherited from Model
  virtual void update(float dt);
  virtual void init();
  virtual void reset() { reset(true); };
  virtual void reset(bool randomiseGenomes);
  virtual bool hasFinished() { return m_ga->getCurrentGeneration() == m_numGenerations; };
  
  Evolvable* getEvolvable() { return m_evolvable; };
  GA* getGA() { return m_ga; };
  
  int getNumTrials() { return m_numTrials; };
  void setNumTrials(int n) { m_numTrials = n; };
  
  void setVerbosity(GAVerbosity v) { m_verbosity = v; };
  
  void fitnessFunctionChanged() { m_ga->resetFitnesses(); };
  
  static void genomeToXml(ci::XmlTree& xml, const double* genome, int numGenes, float bestFit, int stage=0);
  
protected:  

  void updateTrialFitness(float f);
  void finishGeneration(int currentGen, float dt);
  void finishRun(const double* bestGenome, float bestFit, float dt);
  void generationToXml(ci::XmlTree* xml, uint32_t gen, const double* genome, float bestFit, float avgFit);
  void updateFitnessStage(int currentGen, float bestFit);
  void test(const double* evolvable, float fitness, float dt);
  
  // An instance of the model to evolve
  Evolvable* m_evolvable;
  
  // An instance of the actual algorithm
  GA* m_ga;
  
  float m_time;
  float m_accFitness;
  uint16_t m_trial;
  uint16_t m_numTrials;
  int16_t m_numTrialsStageMult;
  uint16_t m_stage;
  uint32_t m_prevGeneration;  /// For detecting if GA has incremented a generation
  uint32_t m_numGenerations;
  uint32_t m_outputInterval;
  
  int m_maxStage;
  float m_stageFitThreshold;
  uint16_t m_genFitBufferSize;
  std::vector<float> m_genFitBuffer;
  
  double m_reducedMutationVar;
  double m_reducedRecombinationRate;
  int m_reduceMutationAt;
  
  int m_verbosity;
  int m_trialAggregation;
  
  bool m_autoEval;
  
  ci::XmlTree* m_progressLog; // Saves fitness progress each generation and best individual
  ci::XmlTree* m_resultsLog;  // Saves whole population at end of evolution for later incremental pickup
  ci::XmlTree* m_finalGenomeLog; // Stores only best genome at the end of evolution
  bool m_saveBestEachGen;
};

} // namespace dmx

#endif
