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
#include <iomanip> 

namespace dmx
{

#define GA_ERROR_LOG 0
  
// --------------------------------------------------------------------------------------------
static bool isNonEval(float f)
{
  return fabs(f - NON_EVAL) < EPS_NON_EVAL;
}

static bool isEval(float f)
{
  return fabs(f - NON_EVAL) >= EPS_NON_EVAL;
}
  
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
    
    // For safety, initialise to 0
    std::fill(m_genomes[i], m_genomes[i] + m_genomeLength, 0.0);
  }

  m_fitnesses = new float[m_popSize];
  std::fill(m_fitnesses, m_fitnesses + m_popSize, NON_EVAL);

  m_weightedAverageGenome = new double[m_popSize]; 
  
  // Some defaults, though should be set externally and not subsequently reset to these defaults
  m_maxMutation = 0.01;      // amount of mutation  (scales a gaussian random variable)
  m_mutationRate = 1.0;
  m_recombinationRate = 0.05; // probability of copying a gene from winner to loser of tournament  
  m_avoidReevaluation = true;
  
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
  
  m_idum = (long)-time(0);	// "randomly" seed random number generator !
  
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
      
      m_fitnesses[i] = NON_EVAL;
    }
  }
  
  // select first pair of genomes for tournament
  startNextTournament();
}

// --------------------------------------------------------------------------------------------
void GA::randomise(bool absolute, double maxAmount)
{
  if(absolute)
  {
    // Set to new random values in [0,1]
    for (int i = 0; i < m_popSize; i++)
    {
      for (int j = 0; j < m_genomeLength; j++)
      {
        m_genomes[i][j] = maxAmount * ran1(&m_idum);
        m_genomes[i][j] = clamp(m_genomes[i][j], 0.0, 1.0);
      }
    }  
  }
  else 
  {
    // Jitter values by +- maxAmount
    for (int i = 0; i < m_popSize; i++)
    {
      for (int j = 0; j < m_genomeLength; j++)
      {
        m_genomes[i][j] += (2 * maxAmount * ran1(&m_idum)) - maxAmount;
        m_genomes[i][j] = clamp(m_genomes[i][j], 0.0, 1.0);        
      }
    }   
  }

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
  fitness = NON_EVAL;
  double* result = 0;
  for (int i = 0; i < m_popSize; ++i)
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
    if(isEval(m_fitnesses[i]))
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
#if GA_ERROR_LOG
    std::cout << std::setprecision(10) << "Setting fitA to " << fit << std::endl;
#endif
    m_fitnessA = fit;
    m_currentGenome = GA_SECOND_GENOME;
    
    // Check whether we can skip second evaluation
    if (m_avoidReevaluation && isEval(m_fitnesses[m_currentIndB]))
    {
#if GA_ERROR_LOG
      std::cout << std::setprecision(10) << "GA WARN: indB " << m_currentIndB << "(" << m_fitnesses[m_currentIndB] <<  ") already evaled. " << std::endl;
#endif
      m_fitnessB = m_fitnesses[m_currentIndB];
      
      performTournament(m_currentIndA, m_fitnessA, m_currentIndB, m_fitnessB);
      startNextTournament();

    }
  }
  else if(m_currentGenome == GA_SECOND_GENOME)
  {
    // If 2nd genome, finish tournament also
#if GA_ERROR_LOG
    std::cout << std::setprecision(10) << "Setting fitB to " << fit << std::endl;
#endif
    m_fitnessB = fit;
    
    performTournament(m_currentIndA, m_fitnessA, m_currentIndB, m_fitnessB);
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
// Generates next couple of genomes to evaluate (randomly from neighborhood in 1d population)
// --------------------------------------------------------------------------------------------
void GA::startNextTournament()
{
  getRandomPairInDeme(m_currentIndA, m_currentIndB);
#if GA_ERROR_LOG
  std::cout << std::setprecision(10) << "Ga: New Tournament. " << m_currentIndA << ": " << m_fitnesses[m_currentIndA] << ". " << m_currentIndB << ": " << m_fitnesses[m_currentIndB] << std::endl;
#endif
  
  m_currentGenome = GA_FIRST_GENOME;
  
  if (m_avoidReevaluation)
  {
    // Test of potential speed up by not reevaluating already evaluated genomes
    if (isEval(m_fitnesses[m_currentIndA]))
    {
#if GA_ERROR_LOG
      std::cout << std::setprecision(10) << "GA WARN: indA " << m_currentIndA << "(" << m_fitnesses[m_currentIndA] <<  ") already evaled. " << std::endl;
#endif
      setFitness(m_fitnesses[m_currentIndA]);
    }
  }
}

// --------------------------------------------------------------------------------------------
// decide winner and loser and perform mutation
// --------------------------------------------------------------------------------------------
void GA::performTournament(uint16_t indA, float fitA, uint16_t indB, float fitB)
{
#if GA_ERROR_LOG
  std::cout << "Ga: perform tournament. " << indA << ": " << fitA << ". " << indB << ": " << fitB << std::endl;
  std::cout << std::setprecision(10) << "Ga: before tournament. " << indA << ": " << m_fitnesses[indA] << ". " << indB << ": " << m_fitnesses[indB] << std::endl;  
#endif
  
  int winner, loser;
  if (fitA >= fitB)
  {
    winner = indA;
    loser = indB;
    
    if(m_avoidReevaluation && fitA < m_fitnesses[winner])
    {
      std::cout << std::setprecision(10) << "GA ERROR: overwriting fitness of winnerA " << winner << "(" << m_fitnesses[winner] << ")" << " with new value: " << fitA << std::endl;
    }
    
    //assert(!m_avoidReevaluation || fitA >= m_fitnesses[winner]); // Make sure we're not overwriting a better one
    
    m_fitnesses[winner] = fitA;
  }
  else 
  {
    winner = indB;
    loser = indA;
    
    if(m_avoidReevaluation && fitB < m_fitnesses[winner])
    {
      std::cout << std::setprecision(10)  << "GA ERROR: overwriting fitness of winnerB " << winner << "(" << m_fitnesses[winner] << ")" << " with new value: " << fitB << std::endl;
    }
    
    assert(!m_avoidReevaluation || fitB >= m_fitnesses[winner]); // Make sure we're not overwriting a better one

    m_fitnesses[winner] = fitB;
  }
  
  // do mutation
  mutate(winner, loser);

  // The loser's fitness is now undetermined
  m_fitnesses[loser] = NON_EVAL;
  
#if GA_ERROR_LOG
  std::cout << std::setprecision(10) << "Ga: after tournament. " << winner << ": " << m_fitnesses[winner] << ". " << loser << ": " << m_fitnesses[loser] << std::endl;
#endif
  
  m_tournament++;

  // reset at end of generation
  if (m_tournament % m_popSize == 0)
  {
    m_generation++;    
  }
}

// --------------------------------------------------------------------------------------------
// Mutate loser with features from winner (genotype must stay within [0,1] boundaries)
// Mutations come from a random vector in a unit hypersphere, whose length is chosen from
// a gaussian, and its direction from a uniform distribution.
// --------------------------------------------------------------------------------------------
void GA::mutate(uint16_t winner, uint16_t loser)
{
  double randGausLength = gasdev(&m_idum) * m_maxMutation;
  
  for (int i = 0; i < m_genomeLength; i++)
  {      
    // recombination/genetic infection (some genes are copied from winner to loser)
    if (ran1(&m_idum) < m_recombinationRate) 
    {
      m_genomes[loser][i] = m_genomes[winner][i];
    }

    // Gaussian mutation
    //const double mutation = gasdev(&m_idum) * m_maxMutation;
    if ((m_mutationRate < 1) && (ran1(&m_idum) < m_mutationRate))
    {
      const double mutation = (-1.0 + 2.0 * ran1(&m_idum)) * randGausLength;
      m_genomes[loser][i] += mutation;
    }

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
  ga.setAttribute("MutationRate", m_mutationRate);
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
  // But we allow reading from a saved GA that has used fewer genes, this allows for incremental adding of genes
  // Genes not copied from previous GA will have been initialised to 0.
  if(genomeLength <= m_genomeLength && popSize == m_popSize)
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
  else 
  {
    std::cout << "Warning: GA has not been reloaded! Genome lengths or population sizes are not identical." << std::endl;
  }

  
  return false;
}

} // namespace




