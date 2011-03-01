/*
 *  TestEvolvable.h
 *  openDynmx
 *
 *  Created by Thomas Buhrmann on 10/01/2011.
 *  Copyright 2011 __MyCompanyName__. All rights reserved.
 *
 */

#ifndef _TEST_EVOLVABLE_
#define _TEST_EVOLVABLE_

#include "GARunner.h"

//----------------------------------------------------------------------------------------------------------------------
//----------------------------------------------------------------------------------------------------------------------
class TestEvolvable : public dmx::Evolvable
{
public:

  // these two need to be kept in sync!
  enum Fitness_Func
  {
    kSphere = 0,
    kRosenBrock,
    kStep,
    kQuartic,
    kNumFitFunctions
  };
  
  static const int s_fitnessFuncNumGenes [kNumFitFunctions];

  TestEvolvable(int fitFunc) : m_fitnessFunction(fitFunc)
  {
    m_numGenes = s_fitnessFuncNumGenes[fitFunc];
    m_data = new double[m_numGenes];
  };
  
  ~TestEvolvable() { delete [] m_data; };

  virtual int getNumGenes();
  virtual void decodeGenome(const double* genome);
  virtual float getFitness();
  virtual void update(float dt);
  virtual void reset();
  
  int m_numGenes;
  double* m_data;
  int m_fitnessFunction;
};

//----------------------------------------------------------------------------------------------------------------------
// Implementation right here in header file
//----------------------------------------------------------------------------------------------------------------------
const int TestEvolvable::s_fitnessFuncNumGenes[kNumFitFunctions] = { 2, 2, 5, 30 };

//----------------------------------------------------------------------------------------------------------------------
int TestEvolvable::getNumGenes() 
{ 
  return m_numGenes; 
}

//----------------------------------------------------------------------------------------------------------------------
void TestEvolvable::decodeGenome(const double* g)
{
  for(int i = 0; i < m_numGenes; i++)
  {
    switch(m_fitnessFunction)
    {
    case kSphere:
      m_data[i] = dmx::mapUnitIntervalToRange(g[i], -5.12, 5.12);
      break;
    case kRosenBrock:
      m_data[i] = dmx::mapUnitIntervalToRange(g[i], -2.048, 2.048);
      break;
    case kStep:
      m_data[i] = dmx::mapUnitIntervalToRange(g[i], -5.12, 5.12);
      break;
    case kQuartic:
      m_data[i] = dmx::mapUnitIntervalToRange(g[i], -1.28, 1.28);
      break;
    }
  }
}

//----------------------------------------------------------------------------------------------------------------------
float TestEvolvable::getFitness()
{
  float fitness = 0.0;
  switch(m_fitnessFunction)
  {
  case kSphere:
    fitness = 1.0 - (m_data[0] * m_data[0] + m_data[1] * m_data[1]);
    break;
    
  case kRosenBrock:
    fitness = m_data[0] * m_data[0] - m_data[1];
    fitness = 100.0 * fitness * fitness;
    fitness += (1 - m_data[0]) * (1 - m_data[0]);
    fitness = 1.0 - fitness;
    break;    
    
  case kStep:
    for(int i = 0; i < m_numGenes; i++)
    {
      fitness += (int) fabs(m_data[i]);
    }
    fitness = 1.0 - fitness;
    break;    
    
  case kQuartic:
    fitness = 1;
    break;
  } 
  
  return fitness;
}

//----------------------------------------------------------------------------------------------------------------------
void TestEvolvable::update(float dt) {};

//----------------------------------------------------------------------------------------------------------------------
void TestEvolvable::reset() {};

//----------------------------------------------------------------------------------------------------------------------
#endif