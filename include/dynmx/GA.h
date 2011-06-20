/*
* Copyright (c) 2009 NaturalMotion Ltd. All rights reserved.
*
* Not to be copied, adapted, modified, used, distributed, sold,
* licensed or commercially exploited in any manner without the
* written consent of NaturalMotion.
*
* All non public elements of this software are the confidential
* information of NaturalMotion and may not be disclosed to any
* person nor used for any purpose not expressly approved by
* NaturalMotion in writing.
*
*/
#ifndef GA_H
#define GA_H

#include <math.h>
#include <stdio.h>
#include <vector>

#include "cinder/xml.h"

#include "MathUtils.h"

#define GA_BOUND_CHECK 0  // 0 = clamping, 1 = mirroring
#define MAX_NUM_PARALLEL_EVALS 8

// holds data for typical usage of a GA
// ---------------------------------------------------------------------------------------------------------------------
struct GADescriptor
{
  float trialDuration;
  int numTrials;
  int numGenerations;
  int populationSize;
  int genomeSize;
  int demeSize;
  
  GADescriptor() : trialDuration(1.0f), numTrials(1), numGenerations(100), populationSize(50), 
    demeSize(5), genomeSize(10) {};
};

// ---------------------------------------------------------------------------------------------------------------------
// Microbial GA with Demes				      
// for details see http://www.cogs.susx.ac.uk/users/inmanh/MicrobialGA_ECAL2009.pdf 
// ---------------------------------------------------------------------------------------------------------------------
class GA
{
public:

  GA(int popsize=100, int genlength=10, int demeWidth = 3);
  GA(const GADescriptor& desc);
  ~GA();

  // interface for repeated evaluation of individual genomes
  /// Returns pointer to one of two genomes in current tournament, but doesn't allow modification of its contents
  const double* getCurrentGenome() const;
  /// sets the fitness of the current genome and selects a new current genome
  void setFitness(float fit);

  // interface for evaluating N pairs of genomes in parallel
  // returns 2*N pointers to genomes in the population
  const std::vector<const double*> getNTournamentPairs(int n);
  void setNFitnessPairs(const std::vector<float>& fitnesses);
  
  int getGenomeSize() const { return m_genomeLength; };
  int getPopulationSize() const { return m_popSize; };
  int getCurrentGeneration () const { return m_generation;};

  /// Returns the best genome found so far and its fitness
  const double* getBestGenome(float &fitness) const;
  float getAvgFitness() const;

  /// Returns a genome that is the fitness-weighted average of the (evaluated) population
  const double* getWeightedAverageGenome(float &fitness) const;
  
  /// Returns complete population, e.g. for drawing
  double** getPopulation() const { return m_genomes; };
  float* getFitnesses() const {return m_fitnesses; };

  void setRandomSeed(long seed) { m_idum = seed; };
  void setDemeWidth(int width) { m_demeWidth = width; };
  void setMutationMax(double mut) { m_maxMutation = mut; };
  void setRecombinationRate(double rcr) { m_recombinationRate = rcr; };
  
  void loadPopulation(const std::string fnm);
  void savePopulation(const std::string fnm) const;
  void toXml(ci::XmlTree& xml, bool includeGenomes = true) const;
  bool fromXml(const ci::XmlTree& xml, bool includeGenomes = true);

  /// Allow manual setting of the genomes in the population
  void setGenome(int iGenome, const double *genome, float fitness);


protected:

  // to store which of the two genomes in the current tournament is being evaluated
  enum GA_STATE
  {
    GA_NONE_SELECTED = 0,
    GA_FIRST_GENOME,
    GA_SECOND_GENOME
  } m_currentGenome;

  void init();
  void reset();
  
  // internal interface for repeated evaluation of individual genomes
  // selects two random genomes for tournament from neighborhood in population
  void startNextTournament();   
  void finishThisTournament();  

  // determines winner and loser of tournament and mutates loser
  void performTournament(int indA, float fitA, int indB, float fitB);
  // loser of a tournament is being replaced by mutated version of the winner
  void mutate(int winner, int loser);
  // returns indices for a random pair of genomes from a neighbourhood defined by demeWidth
  void getRandomPairInDeme(int& indA, int& indB);
  // returns indices for a random pair of genomes that is different from a list of already existing pairs
  void getDifferentRandomPair(int& indA, int& indB, int* existingPairs, int numExistingPairs);
  // checks whether a pair of genomes is different from a list of already existing pairs
  bool pairIsDifferentFrom(int& indA, int& indB, int* existingPairs, int numExistingPairs);
  
  // print out result of evolution
  void printFitness(const std::string fnm) const;
  void printBestGenome(const std::string fnm) const;

  // the population of genomes: array of arrays of type SCALAR (float/double)
  double** m_genomes;
  // the fitness of each genome (may not be calculated yet)
  float *m_fitnesses;
  // The weighted average genome - only filled in when it's requested
  double *m_weightedAverageGenome;
  
  // stores indices of pairs of requested genomes for parallel evaluations
  int m_requestedGenomes[2 * MAX_NUM_PARALLEL_EVALS];

  int m_popSize;
  int m_genomeLength;
  int m_demeWidth;		// width of 1d neighborhood
  int m_generation;   // current generation
  int m_tournament;   // current tournament (number of evaluations so far)
  int m_currentIndA;  // genomes in current tournament
  int m_currentIndB;

  long m_idum;        // seed for random number generator

  double m_maxMutation;		    // amount of Gaussian vector mutation
  double m_recombinationRate;  // amount of Gaussian vector mutation

  float 
    m_fitnessA,   // fitness of first individual in current tournament
    m_fitnessB;   // fitness of second individual in current tournament
};

#endif // GA_H