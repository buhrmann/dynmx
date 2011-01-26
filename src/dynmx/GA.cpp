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
#include "GA.h"
#include <time.h>
#include "assert.h"

#ifdef WIN32
# pragma warning( disable : 4996 ) 
#endif


// --------------------------------------------------------------------------------------------
// con- and destructor
// --------------------------------------------------------------------------------------------
GA::GA(int popsize, int genomeLength, int demeWidth) : 
  m_popSize(popsize), 
  m_genomeLength(genomeLength),
  m_demeWidth(demeWidth)
  { 
    init(); 
  };

// --------------------------------------------------------------------------------------------
GA::~GA()
{  
  for(int i = 0; i < m_popSize; ++i)
    delete [] m_genomes[i];
    
  delete [] m_genomes;
  delete [] m_fitnesses;
  delete [] m_weightedAverageGenome;
}

// --------------------------------------------------------------------------------------------
// allocate and initialize genome array
// --------------------------------------------------------------------------------------------
void GA::init()
{
  // some defaults
  m_maxMutation = 0.01;      // amount of mutation  (scales a gaussian random variable)
  m_recombinationRate = 0.05; // probability of copying a gene from winner to loser of tournament

  // initialization
  m_tournament = 0;
  m_generation = 0;
  m_idum = (long)-time(0);	// seed random number generator
  m_currentGenome = GA_NONE_SELECTED;
  
  // allocate 
  m_genomes = new double*[m_popSize];
  for(int i = 0; i < m_popSize; i++)
    m_genomes[i] = new double[m_popSize];

  // random initialization
  for (int i = 0; i < m_popSize; i++)
    for (int j = 0; j < m_genomeLength; j++)
      m_genomes[i][j] = ran1(&m_idum);

  m_fitnesses = new float[m_popSize];
  for(int i = 0; i < m_popSize; i++)
    m_fitnesses[i] = -1.0f;

  m_weightedAverageGenome = new double[m_popSize];
  
  // select first pair of genomes for tournament
  startNextTournament();
}

// --------------------------------------------------------------------------------------------
// simply return current genome in tournament pair
// --------------------------------------------------------------------------------------------
const double* GA::getCurrentGenome() const
{
  if(m_currentGenome == GA_FIRST_GENOME)
    return m_genomes[m_currentIndA];
  else 
    return m_genomes[m_currentIndB];
}

// --------------------------------------------------------------------------------------------
// simply return the best genome found so far
// --------------------------------------------------------------------------------------------
const double* GA::getBestGenome(float &fitness) const
{
  fitness = -1.0f;
  double *result = 0;
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
  for (int i = 0 ; i < m_popSize ; ++i)
  {
    avg += m_fitnesses[i];
  }
  return avg / m_popSize;
}

// --------------------------------------------------------------------------------------------
const double* GA::getWeightedAverageGenome(float &fitness) const
{
  fitness = 0.0f;
  for (int i = 0 ; i < m_genomeLength ; ++i)
    m_weightedAverageGenome[i] = 0.0f;

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
  // If 1st genome: simply cache fitness
  if(m_currentGenome == GA_FIRST_GENOME)
  {
    m_fitnessA = fit;
    m_currentGenome = GA_SECOND_GENOME;
    m_fitnesses[m_currentIndA] = fit;
  }
  // If 2nd genome, finish tournament also
  else if(m_currentGenome == GA_SECOND_GENOME)
  {
    m_fitnessB = fit;
    m_fitnesses[m_currentIndB] = fit;
    finishThisTournament();
    startNextTournament();
  }
}

// --------------------------------------------------------------------------------------------
// returns indices for a random pair of genomes from a neighbourhood defined by demeWidth
// --------------------------------------------------------------------------------------------
void GA::getRandomPairInDeme(int& indA, int& indB)
{
  indA = (int)(m_popSize*ran1(&m_idum));
  indB = indA;
  while (indB == indA)
    indB = (indA + 1 + (int)(m_demeWidth*ran1(&m_idum))) % m_popSize;
}

// --------------------------------------------------------------------------------------------
// returns indices for a random pair of genomes that is different from a list of already existing pairs
// --------------------------------------------------------------------------------------------
void GA::getDifferentRandomPair(int& indA, int& indB, int* existingPairs, int numPairs)
{
  do
  {
    getRandomPairInDeme(indA, indB);
  } while(!pairIsDifferentFrom(indA, indB, existingPairs, numPairs));
}

// --------------------------------------------------------------------------------------------
// checks whether a pair of genomes is different from a list of already existing pairs
// --------------------------------------------------------------------------------------------
bool GA::pairIsDifferentFrom(int& indA, int& indB, int* existingPairs, int numExistingPairs)
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
void GA::performTournament(int indA, float fitA, int indB, float fitB)
{
  int winner, loser;
  if (fitA > fitB)
  {
    winner = indA;
    loser = indB;
    m_fitnesses[winner] = fitA;
  }
  else 
  {
    winner = indB;
    loser = indA;
    m_fitnesses[winner] = fitB;
  }

  // do mutation
  mutate(winner, loser);

  //m_fitnesses[loser] = -1.0f;

  m_tournament++;

  // reset at end of generation
  if ((m_tournament + 1) % m_popSize == 0)
  {
    printFitness("/Users/thomasbuhrmann/Code/Results/GATest/GA_fitness.txt");
    printBestGenome("/Users/thomasbuhrmann/Code/Results/GATest/GA_bestGenome.txt");
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
    tournamentPairs.push_back(m_genomes[m_requestedGenomes[i]]);
  return tournamentPairs;
}

// --------------------------------------------------------------------------------------------
void GA::setNFitnessPairs(const std::vector<float>& fitnesses)
{
  assert((fitnesses.size() < 2*MAX_NUM_PARALLEL_EVALS) && (fitnesses.size() % 2 == 0));

  // use fitness to perform tournament/mutation
  for (int i=0; i<(int) fitnesses.size()/2; i++)
  {
    int indA = 2*i;
    int indB = 2*i+1;
    performTournament(m_requestedGenomes[indA], fitnesses[indA], m_requestedGenomes[indB], fitnesses[indB]);
  }
}

// --------------------------------------------------------------------------------------------
// mutate loser with features from winner (genotype must stay within [0,1] boundaries)
// --------------------------------------------------------------------------------------------
void GA::mutate(int winner, int loser)
{
  for (int i=0; i < m_genomeLength; i++)
  {      
    // recombination/genetic infection (some genes are copied from winner to loser)
    if (ran1(&m_idum) < m_recombinationRate) 
      m_genomes[loser][i] = m_genomes[winner][i];

    // gaussian mutation
    m_genomes[loser][i] += (float) (gasdev(&m_idum) * m_maxMutation);

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

// --------------------------------------------------------------------------------------------
// load a previously saved population
// --------------------------------------------------------------------------------------------
void GA::loadPopulation(const char* fnm)
{
  FILE* fp = fopen(fnm, "r");
  if (fp)
  {
    // sanity check some things that must be the same
    int genomeLength;
    fscanf(fp, "%d", &genomeLength);
    if (genomeLength != m_genomeLength)
    {
      printf("Attempt to load a different genome length\n");
      fclose(fp);
      return;
    }

    int popSize;
    fscanf(fp, "%d", &popSize);
    if (popSize != m_popSize)
    {
      printf("Attempt to load a different population size\n");
      fclose(fp);
      return;
    }

    // overwrite internal settings
    fscanf(fp, "%d", &m_demeWidth);
    fscanf(fp, "%d", &m_generation);
    fscanf(fp, "%d", &m_tournament);
    fscanf(fp, "%d", &m_currentIndA);
    fscanf(fp, "%d", &m_currentIndB);

    fscanf(fp, "%lf", &m_maxMutation);
    fscanf(fp, "%lf", &m_recombinationRate);
    fscanf(fp, "%g", &m_fitnessA);
    fscanf(fp, "%g", &m_fitnessB);
    for (int i = 0; i < m_popSize; i++)
    {
      float fitness;
      fscanf(fp, "%f\t", &fitness); 
      m_fitnesses[i] = fitness;
      for (int j=0; j < m_genomeLength; j++)
      {
        double tempD;
        fscanf(fp, "%lf\t", &tempD); 
        m_genomes[i][j] = tempD;
      }
    }
    fclose(fp);
  }
  else
  {
    printf("Couldn't open population file for reading: %s\n", fnm);
  }
}

// --------------------------------------------------------------------------------------------
// save final population
// --------------------------------------------------------------------------------------------
void GA::savePopulation(const char* fnm) const
{
  FILE* fp = fopen(fnm,"w+");
  if (fp)
  {
    fprintf(fp, "%d\n", m_genomeLength);
    fprintf(fp, "%d\n", m_popSize);

    // overwrite internal settings
    fprintf(fp, "%d\n", m_demeWidth);
    fprintf(fp, "%d\n", m_generation);
    fprintf(fp, "%d\n", m_tournament);
    fprintf(fp, "%d\n", m_currentIndA);
    fprintf(fp, "%d\n", m_currentIndB);

    fprintf(fp, "%lf\n", m_maxMutation);
    fprintf(fp, "%lf\n", m_recombinationRate);
    fprintf(fp, "%g\n", m_fitnessA);
    fprintf(fp, "%g\n", m_fitnessB);

    for (int i=0; i < m_popSize; i++)
    {
      fprintf(fp, "%f\t", m_fitnesses[i]);
      for (int j=0; j < m_genomeLength; j++)
        fprintf(fp, "%lf\t", m_genomes[i][j]);
      fprintf(fp, "\n");
    }
    fflush(fp);
    fclose(fp);
  }
  else
  {
    printf("Couldn't open population file for writing: %s\n", fnm);
  }
}

// --------------------------------------------------------------------------------------------
// File output
// --------------------------------------------------------------------------------------------
void GA::printFitness(const char* fnm) const
{
  float max = -1.0f;
  (void) getBestGenome(max);
  float avg = getAvgFitness();

  FILE* fp = fopen(fnm, "a");
  if (fp)
  {
    fprintf(fp,"%i\t%f\t%f\n", m_generation, max, avg);
    fclose(fp);
  }
}

// --------------------------------------------------------------------------------------------
void GA::printBestGenome(const char* fnm) const
{
  // save best agent
  FILE* fp = fopen(fnm, "a");
  if (fp)
  {
    fprintf(fp, "%i\t", m_generation);
    float junk;
    for (int i=0; i < m_genomeLength; i++)    
      fprintf(fp, "%lf ", getBestGenome(junk)[i]);
    fprintf(fp, "\n");
    fclose(fp);
  }
}

// --------------------------------------------------------------------------------------------
// Fast random number generator from Numerical Recipes in C					            	
// --------------------------------------------------------------------------------------------
#define IAA 16807
#define IM 2147483647
#define AM (1. /IM)
#define IQ 127773
#define IR 2836
#define NTAB 32
#define NDIV (1+(IM-1)/NTAB)
#define EPS 1.2e-7
#define RNMX (1.0-EPS)

// --------------------------------------------------------------------------------------------
double GA::ran1(long *idum)
{
	int j;
	long k;
	static long iy=0;
	static long iv[NTAB];
	double temp;
	if (*idum <= 0 || !iy) 
  {
		if (-(*idum) < 1) 
      *idum = 1;
		else 
      *idum = -(*idum);
		for (j=NTAB+7; j>=0; j--) 
    {
			k = (*idum)/IQ;
			*idum=IAA*(*idum-k*IQ)-IR*k;
			if (*idum <0) 
        *idum += IM;
			if (j < NTAB) 
        iv[j] = *idum;
		}
		iy = iv[0];
	}
	k = (*idum)/IQ;
	*idum=IAA*(*idum-k*IQ)-IR*k;
	if (*idum < 0) 
    *idum += IM;
	j = iy/NDIV;
	iy=iv[j];
	iv[j] = *idum;
	if ((temp=AM*iy) > RNMX) 
    return RNMX;
	else 
    return temp;
}

// --------------------------------------------------------------------------------------------
double GA::gasdev(long *idum)
{
	static int iset=0;
	static double gset;
	double fac,rsq,v1,v2;
	if (iset == 0) 
  {
		do 
    {
			v1=2.0*ran1(idum)-1.0;
			v2=2.0*ran1(idum)-1.0;
			rsq=v1*v1+v2*v2;
		} while (rsq >= 1.0 || rsq==0.0);
		fac=sqrt(-2.0*log(rsq)/rsq);
		gset=v1*fac;
		iset=1;
		return v2*fac;
	} 
  else 
  {
		iset =0;
		return gset;
	}
}



