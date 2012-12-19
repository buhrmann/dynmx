// *******************************
// Various Random Number Utilities
//
// RDB 2/95
// *******************************

#pragma once

#include <fstream>
#include "MathUtils.h"

using namespace std;


// Functions to manipulate the global random state for backward compatibility
//----------------------------------------------------------------------------------------------------------------------
void SetRandomSeed(long seed);
long GetRandomSeed(void);
void WriteRandomState(ostream& os);
void BinaryWriteRandomState(ofstream& bofs);
void ReadRandomState(istream& is);
void BinaryReadRandomState(ifstream& bifs);
double UniformRandom(double min,double max);
int UniformRandomInteger(int min,int max);
double GaussianRandom(double mean, double variance);
int ProbabilisticChoice(double prob);

//----------------------------------------------------------------------------------------------------------------------  
// The RandomState class declaration
//----------------------------------------------------------------------------------------------------------------------
class RandomState 
{
public:
  RandomState(long seedIn = 0) {SetRandomSeed(seedIn); gaussian_flag = 0; };
  ~RandomState() {};
  
  // Accessors
  void SetRandomSeed(long seed);
  long GetRandomSeed(void);
  
  // Helper functions
  double ran1(void);
  void GenerateNormals(void);
  
  // Return random deviates
  double UniformRandom(double min,double max);
  int UniformRandomInteger(int min,int max);
  double GaussianRandom(double mean, double variance);
  int ProbabilisticChoice(double prob);
  
  // Input/Output 
  void WriteRandomState(ostream& os);
  void BinaryWriteRandomState(ofstream& bofs);
  void ReadRandomState(istream& is);
  void BinaryReadRandomState(ifstream& bifs);
  

  long seed, idum, iy, iv[NTAB];
  int gaussian_flag;
  double gX1, gX2;
};
