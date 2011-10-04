// ***********************************************************
// A class for continuous-time recurrent neural networks
//
// RDB 
//  8/94 Created
//  12/98 Optimized integration
//  1/08 Added table-based fast sigmoid w/ linear interpolation
// ************************************************************
#ifndef CTRNN_H
#define CTRNN_H

// Uncomment the following line for table-based fast sigmoid w/ linear interpolation
#define FAST_SIGMOID

#include "Random.h"
#include <iostream>
#include <math.h>

#pragma once

//----------------------------------------------------------------------------------------------------------------------
// The sigmoid function
//----------------------------------------------------------------------------------------------------------------------
#ifdef FAST_SIGMOID
const int SigTabSize = 128;
const double SigTabRange = 15.0;
double fastsigmoid(double x);
#endif

inline double sigma(double x) 
{
  return 1.0 / (1.0 + exp(-x));
}

inline double sigmoid(double x)
{
#ifndef FAST_SIGMOID
  return sigma(x);
#else
  return fastsigmoid(x);
#endif
}

inline double InverseSigmoid(double y)
{
  return log(y / (1.0 - y));
}

//----------------------------------------------------------------------------------------------------------------------
// Other activation functions
//----------------------------------------------------------------------------------------------------------------------
inline double linearActivation(double y)
{
  if(y <= -1)
    return 0.0;
  else if (y >= 1)
    return 1.0;
  else 
    return 0.5 * (y + 1);
}

inline double sineActivation(double y)
{
  return 0.5 * (sin(y) + 1);
}


//----------------------------------------------------------------------------------------------------------------------
// The CTRNN class declaration
//----------------------------------------------------------------------------------------------------------------------
class CTRNN 
{

public:
  
  // Define a type of pointer to activation function
  typedef double (*ActivationFunction)(double);
  
  // Name the activation functions
  enum ActFuncNames
  {
    kAF_Sigmoid,
    kAF_Linear,
    kAF_Sine,
    kAF_NumFunctions
  };
  
  // An array of activation function pointers
  static ActivationFunction s_activationFunctions[kAF_NumFunctions];

  static int getNumRequiredParams(int numNeurons, bool numInputs);

  CTRNN(int newsize = 3);
  ~CTRNN();
  
  void update(double stepsize);
    
  // Accessors
  int getSize(void) { return size; };  
  double getState(int i) { return states[i]; };
  double getOutput(int i) { return outputs[i]; };
  double getBias(int i) { return biases[i]; };
  double getGain(int i) { return gains[i]; };
  double getTimeConstant(int i) { return taus[i]; };
  double getExternalInput(int i) { return externalinputs[i]; };
  double getWeight(int from, int to) { return weights[from][to]; };

  void decodeGenome(const double* params, int numInputs = 0);
  
  void setState(int i, double value) { states[i] = value; outputs[i] = sigmoid(gains[i] * (states[i] + biases[i])); };
  void setOutput(int i, double value) { outputs[i] = value; states[i] = InverseSigmoid(value)/gains[i] - biases[i]; };
  void setBias(int i, double value) { biases[i] = value; };
  void setGain(int i, double value) { gains[i] = value; };
  void setTimeConstant(int i, double value) { taus[i] = value; Rtaus[i] = 1 / value; };
  void setExternalInput(int i, double value) { externalinputs[i] = value; };
  void setWeight(int from, int to, double value) { weights[from][to] = value; };  
  void setCenterCrossing();
  void setActivationFunction(int actFuncName) { m_activationFunction = s_activationFunctions[actFuncName]; };
  void setActivationFunction(double (*pt2Func)(double)) { m_activationFunction = pt2Func; };
  void initActivationFunctionTable();

  // Input and output
  //friend ostream& operator<<(ostream& os, CTRNN& c);
  //friend istream& operator>>(istream& is, CTRNN& c);
                      
  // Control
  void randomizeState(double lb, double ub);
  void randomizeState(double lb, double ub, RandomState &rs);
  void randomizeOutput(double lb, double ub);
  void randomizeOutput(double lb, double ub, RandomState &rs);
  void randomizeWeights(double lb, double ub);
  void randomizeWeights(double lb, double ub, RandomState &rs);  
  void randomizeBiases(double lb, double ub);
  void randomizeTimeConstants(double lb, double ub);
  
  // function pointer for avoiding conditional statements
  ActivationFunction m_activationFunction;

  // internal data
  int size;
  
  double 
    *states, 
    *outputs, 
    *biases, 
    *gains, 
    *taus, 
    *Rtaus, 
    *externalinputs;
    
  double** weights;
  
};

#endif

