/*
 *  CTRNNFactory.cpp
 *  cinder_dynmx
 *
 *  Created by Thomas Buhrmann on 3/20/12.
 *  Copyright 2012 __MyCompanyName__. All rights reserved.
 *
 */


#include "CTRNNFactory.h"

namespace dmx
{

//----------------------------------------------------------------------------------------------------------------------
// Without any kind of symmetry
//----------------------------------------------------------------------------------------------------------------------
  /*
bool CTRNNFactory::decode(CTRNN& ctrnn, const double* params, const DecodeLimits& limits, int numInputs)
{
  const int numNeurons = ctrnn.getSize();
  
  // inputs gains
  for(int i = 0; i < numNeurons; i++)
  {
    if(i < numInputs)
    {
      const double gain = map01To(params[i], limits.gain);
      ctrnn.setGain(i, gain);
    }
    else
    {
      ctrnn.setGain(i, 1.0);
    }
  }
  
  // biases and taus
  int I = numInputs;
  for(int i = 0; i < numNeurons; i++)
  {
    double bias = map01To(params[I + (2 * i)], limits.bias);
    double tau = map01To(params[I + (2 * i) + 1], limits.tau);
    ctrnn.setBias(i, bias);
    ctrnn.setTimeConstant(i, tau);
  }
  
  // weights
  I += 2 * numNeurons;
  int id = 0;
  for(int i = 0; i < numNeurons; i++)
  {
    for(int j = 0; j < numNeurons; j++)
    {
      double weight = map01To(params[I + id], limits.weight);
      ctrnn.setWeight(i, j, weight);
      id++;
    }
  }  
  
  return true;
}
   */
  
//----------------------------------------------------------------------------------------------------------------------  
// Input neurons are placeholder neurons only and are arranged in a layer different from real neurons.
// Each real neuron receives connections from all input neurons.
// Real neurons are are arranged in a bilaterally symmetric layer. I.e. first and last neuron are identical, as well
// as the second and second to last etc...
//----------------------------------------------------------------------------------------------------------------------
bool CTRNNFactory::decode(CTRNN& ctrnn, const double* params, const DecodeLimits& limits, const Topology& topology)
{

  int I = 0; // tracks starting index of a range of parameters
  
  int numUniqueInputs = topology.getNumUniqueNeurons(Topology::kLyr_Input);
  int numUniqueHidden = topology.getNumUniqueNeurons(Topology::kLyr_Hidden);
  int numUniqueOutputs = topology.getNumUniqueNeurons(Topology::kLyr_Output);
  int numInputs = topology.getNumInputs();
  int numInputNeurons = topology.getInputsAreNeurons() ? numInputs : 0;
  int numHidden = topology.getNumHidden();
  
  // Input gains, if inputs are not neurons (in which case weights take care of individual weighting)
  int lastInput = numInputs - 1;  
  if(!topology.getInputsAreNeurons())
  {
    for(int i = 0; i < numUniqueInputs; ++i)
    {
      double gain = map01To(params[I], limits.gain);
      ctrnn.setGain(i, gain);
      ++I;
      
      if(topology.isSymmetric())
      {
        ctrnn.setGain(lastInput - i, gain);  
      }
    }
  }
  
  // Weights from input to hidden layer
  int firstHidden = numInputNeurons;  
  int lastHidden = numInputNeurons + numHidden - 1;
  if(topology.getInputsAreNeurons())
  {
    for(int i = 0; i < numUniqueHidden; ++i)
    {
      for(int j = 0; j < numInputs; ++j)
      {
        double weight = map01To(params[I], limits.weight);
        int from = j;
        int to = firstHidden + i;
        ctrnn.setWeight(from, to, weight); // from ->to
        ctrnn.setInputNeuron(from);
        ++I;
        
        // If network is symmetric, this means connections from hidden layer to first hidden neuron
        // should be the same as those to last hidden neuron, though in reverse order. 
        if(topology.isSymmetric())
        {
          // Ensure middle hidden, if odd number, isn't decoded twice          
          int toSym = lastHidden - i;
          if(toSym != to)
          {
            int fromSym = lastInput - j;
            ctrnn.setWeight(fromSym, toSym, weight);
            ctrnn.setInputNeuron(fromSym);
          }
        } // if symmetric
      } // for numInputs
    } // for numUniqueHidden
  }
  
  
  // Hidden layer: biases and time constants
  for(int i = 0; i < numUniqueHidden; ++i)
  {
    double bias = map01To(params[I], limits.bias);
    double tau = map01To(params[I + 1], limits.tau);
    ctrnn.setBias(firstHidden + i, bias);
    ctrnn.setTimeConstant(firstHidden + i, tau);
    I += 2;
    
    if(topology.isSymmetric())
    {
      int iSym = lastHidden - i;
      ctrnn.setBias(iSym, bias);
      ctrnn.setTimeConstant(iSym, tau);      
    }
  }
  
  // Hidden intralayer weights
  for(int i = 0; i < numUniqueHidden; ++i)
  {
    for(int j = 0; j < numHidden; ++j)
    {
      double weight = map01To(params[I], limits.weight);
      int from = firstHidden + j;
      int to = firstHidden + i;
      ctrnn.setWeight(from, to, weight);
      ++I;
      
      if(topology.isSymmetric())
      {
        // Ensure middle hidden, if odd number, isn't decoded twice
        int toSym = lastHidden - i;
        if(toSym != to)
        {
          int fromSym = lastHidden - j;
          ctrnn.setWeight(fromSym, toSym, weight);
        }
      }
    }
  }  
  
  // Output layer: biases and time constants
  int firstOutput = numInputNeurons + numHidden;
  int lastOutput = firstOutput + topology.getNumOutputs() - 1;
  for(int i = 0; i < numUniqueOutputs; ++i)
  {
    double bias = map01To(params[I], limits.bias);
    double tau = map01To(params[I + 1], limits.tau);
    ctrnn.setBias(firstOutput + i, bias);
    ctrnn.setTimeConstant(firstOutput + i, tau);
    I += 2;
    
    if(topology.isSymmetric())
    {
      int iSym = lastOutput - i;
      ctrnn.setBias(iSym, bias);
      ctrnn.setTimeConstant(iSym, tau);      
    }
  }  
  
  // Weights from hidden layer to outputs
  for(int i = 0; i < numUniqueOutputs; ++i)
  {
    for(int j = 0; j < numHidden; ++j)
    {
      double weight = map01To(params[I], limits.weight);
      int from = firstHidden + j;
      int to = firstOutput + i;      
      ctrnn.setWeight(from, to, weight); // from ->to
      ++I;
      
      // If network is symmetric, this means connections from hidden layer to first hidden neuron
      // should be the same as those to last hidden neuron, though in reverse order. 
      if(topology.isSymmetric())
      {
        // Ensure middle hidden, if odd number, isn't decoded twice          
        int toSym = lastOutput - i;
        if(toSym != to)
        {
          int fromSym = lastHidden - j;
          ctrnn.setWeight(fromSym, toSym, weight);
        }
      } // if symmetric
    } // for numInputs
  } // for numUniqueHidden
  
  // Check we decoded correctly!
  int numReqParams = topology.getNumParameters();
  bool correct = I == numReqParams;
  assert(correct);
  
  return correct;
}
  
} // namespace