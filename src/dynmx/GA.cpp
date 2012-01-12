/*
 *  GA.cpp
 *  cinder_dynmx
 *
 *  Created by Thomas Buhrmann on 01/03/2011.
 *  Copyright 2011 __MyCompanyName__. All rights reserved.
 *
 */

#include "Dynmx.h"
#include "GA.h"

#include <time.h>
#include "assert.h"
#include "float.h"

// TODO: MAX_FLOAT
#define MAX_NEG_FLOAT -FLT_MAX
#define GA_DONT_REEVALUATE 1 

// --------------------------------------------------------------------------------------------
GA::GA(int popsize, int genomeLength, int demeWidth) : 
  m_popSize(popsize), 
  m_genomeLength(genomeLength),
  m_demeWidth(demeWidth)
{ 
  init();
}

// Allocation only
// --------------------------------------------------------------------------------------------
void GA::init()
{
  // allocate 
  m_genomes = new double*[m_popSize];
  for(int i = 0; i < m_popSize; i++)
  {
    m_genomes[i] = new double[m_genomeLength];
  }

  m_fitnesses = new float[m_popSize];     
  m_weightedAverageGenome = new double[m_popSize]; 
  
  // Some defaults, though should be set externally and not subsequently reset to these defaults
  m_maxMutation = 0.01;      // amount of mutation  (scales a gaussian random variable)
  m_recombinationRate = 0.05; // probability of copying a gene from winner to loser of tournament  
  
  reset();
}

// --------------------------------------------------------------------------------------------
GA::~GA()
{  
  for(int i = 0; i < m_popSize; ++i)
  {
    delete [] m_genomes[i];
  }
    
  delete [] m_genomes;
  delete [] m_fitnesses;
  delete [] m_weightedAverageGenome;
}

// --------------------------------------------------------------------------------------------
// allocate and initialize genome array
// --------------------------------------------------------------------------------------------
void GA::reset(bool randomizeGenomes)
{
  // Initialization
  m_tournament = 0;
  m_generation = 0;
  m_idum = (long)-time(0);	// seed random number generator
  m_currentGenome = GA_NONE_SELECTED;

  // Random initialization
  if(randomizeGenomes)
  {
    // Genomes
    for (int i = 0; i < m_popSize; i++)
    {
      for (int j = 0; j < m_genomeLength; j++)
      {
        m_genomes[i][j] = ran1(&m_idum);
      }
    }
  }
  
  // Fitness
  for(int i = 0; i < m_popSize; i++)
  {
    m_fitnesses[i] = MAX_NEG_FLOAT;
  }
  
  // select first pair of genomes for tournament
  startNextTournament();
}

// --------------------------------------------------------------------------------------------
// simply return current genome in tournament pair
// --------------------------------------------------------------------------------------------
const double* GA::getCurrentGenome() const
{
  if(m_currentGenome == GA_FIRST_GENOME)
  {
    return m_genomes[m_currentIndA];
  }
  else 
  {
    return m_genomes[m_currentIndB];
  }
}

// --------------------------------------------------------------------------------------------
// simply return the best genome found so far
// --------------------------------------------------------------------------------------------
const double* GA::getBestGenome(float& fitness) const
{
  fitness = MAX_NEG_FLOAT;
  double* result = 0;
  for (int i = 0 ; i < m_popSize ; ++i)
  {
    if (m_fitnesses[i] >= fitness)
    {
      fitness = m_fitnesses[i];
      result = m_genomes[i];
    }
  }
  return result;
}

// --------------------------------------------------------------------------------------------
float GA::getAvgFitness() const
{
  float avg = 0.0f;
  int n = 0;
  for (int i = 0 ; i < m_popSize ; ++i)
  {
    if(fabs(m_fitnesses[i] - MAX_NEG_FLOAT) > 10.0 )
    {
      avg += m_fitnesses[i];
      n++;
    }
  }
  return avg / n;
}

// --------------------------------------------------------------------------------------------
const double* GA::getWeightedAverageGenome(float &fitness) const
{
  fitness = 0.0f;
  for (int i = 0 ; i < m_genomeLength ; ++i)
  {
    m_weightedAverageGenome[i] = 0.0f;
  }

  float totalFitness = 0.0f;
  for (int i = 0 ; i < m_popSize ; ++i)
  {
    if (m_fitnesses[i] > 0.0f)
    {
      totalFitness += m_fitnesses[i];
      fitness += m_fitnesses[i] * m_fitnesses[i];
      for (int j = 0 ; j < m_genomeLength ; ++j)
        m_weightedAverageGenome[j] += m_genomes[i][j] * m_fitnesses[i];
    }
  }
  
  if (totalFitness > 0.0f)
  {
    fitness /= totalFitness;
    for (int i = 0 ; i < m_genomeLength ; ++i)
      m_weightedAverageGenome[i] /= totalFitness;
  }
  
  return m_weightedAverageGenome;
}


// --------------------------------------------------------------------------------------------
// set current genome's fitness (if it's the second, finish up tournament and start next one)
// --------------------------------------------------------------------------------------------
void GA::setFitness(float fit)
{
  if(m_currentGenome == GA_FIRST_GENOME)
  {
    // If 1st genome: simply cache fitness    
    m_fitnessA = fit;
    m_currentGenome = GA_SECOND_GENOME;
    //m_fitnesses[m_currentIndA] = fit;
  }
  else if(m_currentGenome == GA_SECOND_GENOME)
  {
    // If 2nd genome, finish tournament also    
    m_fitnessB = fit;
    //m_fitnesses[m_currentIndB] = fit;
    finishThisTournament();
    startNextTournament();
  }
}

// --------------------------------------------------------------------------------------------
// returns indices for a random pair of genomes from a neighbourhood defined by demeWidth
// --------------------------------------------------------------------------------------------
void GA::getRandomPairInDeme(uint16_t& indA, uint16_t& indB)
{
  indA = (uint16_t)(m_popSize * ran1(&m_idum));
  indB = indA;
  while (indB == indA)
  {
    indB = (indA + 1 + (uint16_t)(m_demeWidth * ran1(&m_idum))) % m_popSize;
  }
}

// --------------------------------------------------------------------------------------------
// returns indices for a random pair of genomes that is different from a list of already existing pairs
// --------------------------------------------------------------------------------------------
void GA::getDifferentRandomPair(uint16_t& indA, uint16_t& indB, uint16_t* existingPairs, uint16_t numPairs)
{
  do
  {
    getRandomPairInDeme(indA, indB);
  } while(!pairIsDifferentFrom(indA, indB, existingPairs, numPairs));
}

// --------------------------------------------------------------------------------------------
// checks whether a pair of genomes is different from a list of already existing pairs
// --------------------------------------------------------------------------------------------
bool GA::pairIsDifferentFrom(uint16_t& indA, uint16_t& indB, uint16_t* existingPairs, uint16_t numExistingPairs)
{
  bool isDifferent = true;
  for(int i = 0; i < numExistingPairs; i++)
  {
    int otherA = existingPairs[i * 2];
    int otherB = existingPairs[i * 2 + 1];
    if ( indA == otherA || indA == otherB || indB == otherA || indB == otherB )
    {
      isDifferent = false;
      break;
    }
  }
  return isDifferent;
}


// --------------------------------------------------------------------------------------------
// Generates next couple of genomes to evaluate (randomly from neighborhood in 1d population)
// --------------------------------------------------------------------------------------------
void GA::startNextTournament()
{
  getRandomPairInDeme(m_currentIndA, m_currentIndB);
  m_currentGenome = GA_FIRST_GENOME;
  
#if GA_DONT_REEVALUATE  
  // Test of potential speed up by not reevaluating already evaluated genomes
  if(m_fitnesses[m_currentIndA] != MAX_NEG_FLOAT)
  {
    setFitness(m_fitnesses[m_currentIndA]);
  }
  // Do same for second genome in tournament
  if(m_fitnesses[m_currentIndB] != MAX_NEG_FLOAT)
  {
    setFitness(m_fitnesses[m_currentIndB]);
  }
#endif
  
}

// --------------------------------------------------------------------------------------------
// decide winner and loser and perform mutation
// --------------------------------------------------------------------------------------------
void GA::finishThisTournament()
{
  performTournament(m_currentIndA, m_fitnessA, m_currentIndB, m_fitnessB);
}

// --------------------------------------------------------------------------------------------
// decide winner and loser and perform mutation
// --------------------------------------------------------------------------------------------
void GA::performTournament(uint16_t indA, float fitA, uint16_t indB, float fitB)
{
  int winner, loser;
  if (fitA > fitB)
  {
    winner = indA;
    loser = indB;
    assert(fitA >= m_fitnesses[indA]); // Make sure we're not overwriting a better one    
    m_fitnesses[winner] = fitA;
  }
  else 
  {
    winner = indB;
    loser = indA;
    assert(fitB >= m_fitnesses[indB]); // Make sure we're not overwriting a better one    
    m_fitnesses[winner] = fitB;
  }

  // do mutation
  mutate(winner, loser);

  // The loser's fitness is now undetermined
  m_fitnesses[loser] = MAX_NEG_FLOAT;

  m_tournament++;

  // reset at end of generation
  if (m_tournament % m_popSize == 0)
  {
    m_generation++;    
  }
}

// --------------------------------------------------------------------------------------------
// interface for N parallel tournaments
// first fills array with inidices of selected tournament pairs
// then returns a vector of pointers to selected genomes
// --------------------------------------------------------------------------------------------
const std::vector<const double*> GA::getNTournamentPairs(int n)
{
  assert((2*n <= m_popSize/2) && (n < MAX_NUM_PARALLEL_EVALS)); 
  
  // 1st pair randomly:
  getRandomPairInDeme(m_requestedGenomes[0], m_requestedGenomes[1]);
 
  // remaining pairs must be different:
  for(int i = 1; i < n; i++)
  {
    getDifferentRandomPair(m_requestedGenomes[2 * i], m_requestedGenomes[2 * i + 1], &m_requestedGenomes[0], i);
  }

  // collect pointers
  std::vector<const double*> tournamentPairs;
  for(int i = 0; i < n * 2; i++)
  {
    tournamentPairs.push_back(m_genomes[m_requestedGenomes[i]]);
  }
  
  return tournamentPairs;
}

// --------------------------------------------------------------------------------------------
void GA::setNFitnessPairs(const std::vector<float>& fitnesses)
{
  assert((fitnesses.size() < 2 * MAX_NUM_PARALLEL_EVALS) && (fitnesses.size() % 2 == 0));

  // use fitness to perform tournament/mutation
  for (int i = 0; i < (int) fitnesses.size() / 2; i++)
  {
    int indA = 2 * i;
    int indB = 2 * i + 1;
    performTournament(m_requestedGenomes[indA], fitnesses[indA], m_requestedGenomes[indB], fitnesses[indB]);
  }
}

// --------------------------------------------------------------------------------------------
// mutate loser with features from winner (genotype must stay within [0,1] boundaries)
// --------------------------------------------------------------------------------------------
void GA::mutate(uint16_t winner, uint16_t loser)
{
  for (int i = 0; i < m_genomeLength; i++)
  {      
    // recombination/genetic infection (some genes are copied from winner to loser)
    if (ran1(&m_idum) < m_recombinationRate) 
    {
      m_genomes[loser][i] = m_genomes[winner][i];
    }

    // gaussian mutation
    const float mutation = (float) (gasdev(&m_idum) * m_maxMutation);
    m_genomes[loser][i] += mutation;

    // ensure values stay in [0,1] range
    if (m_genomes[loser][i] > 1)
    {
#if GA_BOUND_CHECK == 0
      m_genomes[loser][i] = 1.0; // clamp
#else
      m_genomes[loser][i] = 2.0 - m_genomes[loser][i]; // mirror
#endif
    }
    else if (m_genomes[loser][i] < 0)
    {
#if GA_BOUND_CHECK == 0
      m_genomes[loser][i] =  0.0;
#else
      m_genomes[loser][i] *=  -1.0;
#endif
    }
  }
}

// --------------------------------------------------------------------------------------------
// Set a specific genome
// --------------------------------------------------------------------------------------------
void GA::setGenome(int iGenome, const double *genome, float fitness)
{
  for (int j=0; j < m_genomeLength; j++)
  {
    m_genomes[iGenome][j] = genome[j];
  }
  m_fitnesses[iGenome] = fitness;
}


//--------------------------------------------------------------------------------------------
void GA::toXml(ci::XmlTree& parent, bool includeGenomes) const
{
  ci::XmlTree ga ("GA", "");

  // Parameters
  ga.setAttribute("GenomeLength", m_genomeLength);
  ga.setAttribute("PopulationSize", m_popSize);
  ga.setAttribute("DemeWidth", m_demeWidth);
  ga.setAttribute("MaxMutation", m_maxMutation);
  ga.setAttribute("RecombinationRate", m_recombinationRate);
  float bestFit;
  getBestGenome(bestFit);
  ga.setAttribute("BestFitness", bestFit);
  ga.setAttribute("AvgFitness", getAvgFitness());  
  
  // Population
  if(includeGenomes)
  {
    for (int i = 0; i < m_popSize; i++)
    {
      ci::XmlTree genome ("Genome", "");
      genome.setAttribute("Index", i);
      genome.setAttribute("Fitness", m_fitnesses[i]);
      for (int j = 0; j < m_genomeLength; j++)
      {
        ci::XmlTree gene ("Gene", "");
        gene.setAttribute("Index", j);
        gene.setAttribute("Value", m_genomes[i][j]);
        genome.push_back(gene);
      }
      ga.push_back(genome);
    }
  }

  parent.push_back(ga);
};

// --------------------------------------------------------------------------------------------
bool GA::fromXml(const ci::XmlTree& parent, bool includeGenomes)
{
  assert(parent.hasChild("GAResult/GA"));
  
  const ci::XmlTree& ga = parent.getChild("GAResult/GA");
  int genomeLength = ga.getAttributeValue<int>("GenomeLength");
  int popSize = ga.getAttributeValue<int>("PopulationSize");
  
  // We can only read data from a GA of the same size as the current
  if(genomeLength == m_genomeLength && popSize == m_popSize)
  {
    // Read all genomes (children of GA)
    int i, j;
    i = j = 0;
    for(ci::XmlTree::ConstIter genome = ga.begin(); genome != ga.end(); ++genome)
    {
      m_fitnesses[i] = genome->getAttributeValue<float>("Fitness");    
      // Genes are stored as children of Gene
      j = 0;
      for(ci::XmlTree::ConstIter gene = genome->begin(); gene != genome->end(); ++gene)
      {
        m_genomes[i][j] = gene->getAttributeValue<float>("Value");
        j++;
      }
      i++;
    }
    return true;
  }
  
  return false;
}






