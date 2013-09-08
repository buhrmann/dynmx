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

#include "Dynmx.h"
#include "Random.h"
#include "Recorder.h"
#include "MathUtils.h"
#include <iostream>

namespace dmx
{

// Uncomment the following line for table-based fast sigmoid w/ linear interpolation
#define FAST_SIGMOID

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

void initSigmoidTable(void);

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
  
inline double identity(double y)
{
  return y;
}  


//----------------------------------------------------------------------------------------------------------------------
// The CTRNN class declaration
//----------------------------------------------------------------------------------------------------------------------
class CTRNN 
{
  
  // Define a type of pointer to activation function
  typedef double (*ActivationFunction)(double);
  
  // Define a type of pointer to member(!) update function
  typedef void (CTRNN::*UpdateFunction)(int, double);
  
public:
  
  // Name the activation functions
  enum ActFuncName
  {
    kAF_Sigmoid,
    kAF_Linear,
    kAF_Sine,
    kAF_Identity,
    kAF_NumFunctions
  };
  
  // Name the activation functions
  enum UpdFuncName
  {
    kUF_Neuron,
    kUF_Input,
    kUF_NumFunctions
  };  

  CTRNN(int newsize = 4);
  ~CTRNN();
  
  // Your regular CTRNN update all neurons are treated as standard neurons
  void update(double stepsize);
  
  // Dynamically switches between regular CTRNN update and other update functions, without branching (uses fct pointer)
  void updateDynamic(double stepsize);  

    
  // Accessors
  int getSize(void) const { return size; };  
  double getState(int i) const { return states[i]; };
  double getOutput(int i) const { return outputs[i]; };
  const double* getOutputs() const { return outputs; };
  double getBias(int i) const { return biases[i]; };
  double getGain(int i) const { return gains[i]; };
  double getTimeConstant(int i) const { return taus[i]; };
  double getExternalInput(int i) const { return externalinputs[i]; };
  double getWeight(int from, int to) const { return weights[from][to]; };
  const double* const* getWeights() const { return weights; };
  double getWeightSum() const;
  double getLargestWeight() const;
  
  void setState(int i, double value) { states[i] = value; outputs[i] = sigmoid(gains[i] * (states[i] + biases[i])); };
  void setStateDynamic(int i, double value) { states[i] = value; outputs[i] = (*m_activationFunctions[i])(states[i] + biases[i]); };
  void setOutput(int i, double value) { outputs[i] = value; states[i] = InverseSigmoid(value)/gains[i] - biases[i]; };
  void setBias(int i, double value) { biases[i] = value; };
  void setGain(int i, double value) { gains[i] = value; };
  void setTimeConstant(int i, double value) { taus[i] = value; Rtaus[i] = 1 / value; };
  void setExternalInput(int i, double value) { externalinputs[i] = value; };
  void setWeight(int from, int to, double value) { weights[from][to] = value; };  
  void setCenterCrossing();
  void setInputNeuron(int i);
  
  void setGlobalActivationFunction(ActFuncName name) { m_activationFunction = getActivationFunction(name); };
  void setActivationFunction(int i, ActFuncName name) { m_activationFunctions[i] = getActivationFunction(name); };
  void setUpdateFunction(int i, UpdFuncName name) {   m_updateFunctions[i] = getUpdateFunction(name); };
                      
  // Control
  void randomizeState(double lb, double ub);
  void randomizeState(double lb, double ub, RandomState &rs);
  void randomizeOutput(double lb, double ub);
  void randomizeOutput(double lb, double ub, RandomState &rs);
  void randomizeWeights(double lb, double ub);
  void randomizeWeights(double lb, double ub, RandomState &rs);  
  void randomizeBiases(double lb, double ub);
  void randomizeTimeConstants(double lb, double ub);
  void zeroStates();
  
  void toXml(ci::XmlTree& xml);
  void fromXml(const ci::XmlTree& xml);
  
  void record(Recorder& recorder);    
  
protected:  
  
  void updateNeuron(int i, double stepsize);
  void updateInput(int i, double stepsize);
  
  ActivationFunction getActivationFunction(ActFuncName name);
  UpdateFunction getUpdateFunction(UpdFuncName name);  
  
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
  
  UpdateFunction* m_updateFunctions;
  ActivationFunction* m_activationFunctions; 
    
  double** weights;
};
  
} // namespace

#endif

