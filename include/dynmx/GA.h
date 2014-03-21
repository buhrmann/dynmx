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

namespace dmx
{

#define GA_BOUND_CHECK 0  // 0 = clamping, 1 = mirroring
#define MAX_NUM_PARALLEL_EVALS 8

#define NON_EVAL -100000.f // MAX_NEG_FLOAT
#define EPS_NON_EVAL 10.f

//----------------------------------------------------------------------------------------------------------------------
// Microbial GA with Demes				      
// for details see http://www.cogs.susx.ac.uk/users/inmanh/MicrobialGA_ECAL2009.pdf 
//----------------------------------------------------------------------------------------------------------------------
class GA
{
public:
  
  enum GAProbDist
  {
    kPD_Uniform = 0,
    kPD_Gaussian,
  };

  GA(int popsize=100, int genlength=10, int demeWidth = 3);
  ~GA();

  void reset(bool randomizeGenomes = true);
  void resetFitnesses(float f = NON_EVAL) { std::fill(m_fitnesses, m_fitnesses + m_popSize, f); m_fitnessA = m_fitnessB = f; };
  
  // interface for repeated evaluation of individual genomes
  /// Returns pointer to one of two genomes in current tournament, but doesn't allow modification of its contents
  const double* getCurrentGenome() const;
  /// sets the fitness of the current genome and selects a new current genome
  void setFitness(float fit);
  
  uint16_t getGenomeSize() const { return m_genomeLength; };
  uint16_t getPopulationSize() const { return m_popSize; };
  uint32_t getCurrentGeneration () const { return m_generation; };

  /// Returns the best genome found so far and its fitness
  const double* getBestGenome(float &fitness) const;
  float getAvgFitness() const;

  /// Returns a genome that is the fitness-weighted average of the (evaluated) population
  const double* getWeightedAverageGenome(float &fitness) const;
  
  /// Returns complete population, e.g. for drawing
  const double* const* getPopulation() const { return m_genomes; };
  float* getFitnesses() const {return m_fitnesses; };

  void setRandomSeed(long seed) { m_idum = seed; };
  void setDemeWidth(int width) { m_demeWidth = width; };
  void setMutationVar(double v) { m_mutationVar = v; };
  void setMutationRate(double mr) { m_mutationRate = mr; };
  void setRecombinationRate(double rcr) { m_recombinationRate = rcr; };
  
  void setInterGenomeMutPD(GAProbDist pd) {m_interGenomeMutPD = pd; };
  void setIntraGenomeMutPD(GAProbDist pd) {m_intraGenomeMutPD = pd; };

  
  // If evaluation is deterministic an already evaluated individual does not need
  // to be evaluated again in a subsequent tournament.
  void setAvoidReevaluation(bool re) { m_avoidReevaluation = re; };
  
  void toXml(ci::XmlTree& xml, bool includeGenomes = true) const;
  bool fromXml(const ci::XmlTree& xml, bool includeGenomes = true);

  /// Allow manual setting of the genomes in the population
  void setGenome(int iGenome, const double *genome, float fitness=MAX_NEG_FLOAT);

  void randomise(bool absolute, double maxAmount);

protected:

  // to store which of the two genomes in the current tournament is being evaluated
  enum GA_STATE
  {
    GA_NONE_SELECTED = 0,
    GA_FIRST_GENOME,
    GA_SECOND_GENOME
  } m_currentGenome;

  void init();
  
  // internal interface for repeated evaluation of individual genomes
  // selects two random genomes for tournament from neighborhood in population
  void startNextTournament();
  // determines winner and loser of tournament and mutates loser
  void performTournament(uint16_t indA, float fitA, uint16_t indB, float fitB);
  // loser of a tournament is being replaced by mutated version of the winner
  void mutate(uint16_t winner, uint16_t loser);
  // returns indices for a random pair of genomes from a neighbourhood defined by demeWidth
  void getRandomPairInDeme(uint16_t& indA, uint16_t& indB);
  // returns indices for a random pair of genomes that is different from a list of already existing pairs

  // the population of genomes: array of arrays of type SCALAR (float/double)
  double** m_genomes;
  // the fitness of each genome (may not be calculated yet)
  float *m_fitnesses;
  // The weighted average genome - only filled in when it's requested
  double *m_weightedAverageGenome;
  
  // stores indices of pairs of requested genomes for parallel evaluations
  uint16_t m_requestedGenomes[2 * MAX_NUM_PARALLEL_EVALS];

  uint32_t m_generation;      // current generation  
  uint16_t m_demeWidth;       // width of 1d neighborhood    
  uint16_t m_popSize;
  uint16_t m_genomeLength;  
  uint16_t m_tournament;      // current tournament (number of evaluations so far)
  uint16_t m_currentIndA;     // genomes in current tournament
  uint16_t m_currentIndB;

  long m_idum;                // seed for random number generator

  double m_mutationVar;       // amount of Gaussian vector mutation
  double m_mutationRate;
  double m_recombinationRate; // amount of Gaussian vector mutation
  
  GAProbDist m_interGenomeMutPD;
  GAProbDist m_intraGenomeMutPD;

  float 
    m_fitnessA,   // fitness of first individual in current tournament
    m_fitnessB;   // fitness of second individual in current tournament
  
  bool m_avoidReevaluation;
};
  
} // namespace

#endif // GA_H