/*
 *  Topology.cpp
 *  cinder_dynmx
 *
 *  Created by Thomas Buhrmann on 3/22/12.
 *  Copyright 2012 __MyCompanyName__. All rights reserved.
 *
 */

#include "Topology.h"
#include "MathUtils.h"
#include "CTRNN.h"

namespace dmx
{
  
//----------------------------------------------------------------------------------------------------------------------
void NetLimits::toXml(ci::XmlTree& xml) const
{
  ci::XmlTree netXml ("NetLimits", "");
  
  weight.toXml(netXml, "Weight");
  bias.toXml(netXml, "Bias");
  gain.toXml(netXml, "Gain");
  tau.toXml(netXml, "TimeConstant");  
  
  xml.push_back(netXml);
}

//----------------------------------------------------------------------------------------------------------------------  
void NetLimits::fromXml(const ci::XmlTree& xml)
{
  if (xml.getTag() == "NetLimits")
  {
    weight.fromXml(xml, "Weight");
    bias.fromXml(xml, "Bias");
    gain.fromXml(xml, "Gain");
    tau.fromXml(xml, "TimeConstant");
  }
   
}  

// Returns the number of neurons in a layer that are uniquely encoded, taking into account symmetries and evenness
//----------------------------------------------------------------------------------------------------------------------
int Topology::getNumUniqueNeurons(LayerName layer) const
{
  if(size[layer] == 1)
    return 1;
  
  if(symmetric)
  {
    if(even(size[layer]))
      return size[layer] / 2;       // e.g. 2 if 4 neurons in layer
    else
      return 1 + (size[layer] / 2); // e.g. 3 if 5 neurons in layer
  }
  else 
  {
    return size[layer];
  }
}

//----------------------------------------------------------------------------------------------------------------------
int Topology::getNumParameters() const 
{
  int N = 0;
  int numUniqueInputs = getNumUniqueNeurons(kLyr_Input);
  int numUniqueHidden = getNumUniqueNeurons(kLyr_Hidden);
  int numUniqueOutputs = getNumUniqueNeurons(kLyr_Output);
  
  // Input layer: gains
  // -------------------------------
  // If inputs are not dedicated neurons, we need to encode gains.
  if(!inputsAreNeurons)
  {
    N += numUniqueInputs;
  }
  
  // Weights to hidden layer
  // If we have neurons as inputs, we need connections between inputs and hidden layer.
  // Symmetry means first and last hidden neuron receive same connections from inputs (though in reverse order).
  // But: the connections from input to given hidden neuron have to be different. Symmetry here would mean first 
  // and last sensor are ambiguous.
  if(inputsAreNeurons)
  {
    N += numUniqueHidden * size[kLyr_Input];
  }
  
  
  // Hidden layer: biases and time constants
  // --------------------------------------
  N += 2 * numUniqueHidden;
  
  // Intralayer weights
  N += numUniqueHidden * size[kLyr_Hidden];
  
  
  // Output layer: biases and time constants
  // ---------------------------------------
  N += 2 * numUniqueOutputs;
  
  // Weights from hidden layer
  N += numUniqueOutputs * size[kLyr_Hidden];
  
  // Intralayer weights
  if(outputsLaterallyConnected)
  {
    N += numUniqueOutputs * size[kLyr_Output];
  }
  
  return N;
}
  
  
//----------------------------------------------------------------------------------------------------------------------  
// Input neurons are placeholder neurons only and are arranged in a layer different from real neurons.
// Each real neuron receives connections from all input neurons.
// Real neurons are are arranged in a bilaterally symmetric layer. I.e. first and last neuron are identical, as well
// as the second and second to last etc...
//----------------------------------------------------------------------------------------------------------------------
bool Topology::decode(CTRNN& ctrnn, const double* params, const NetLimits& limits) const
{
  
  int I = 0; // tracks starting index of a range of parameters
  
  int numUniqueInputs = getNumUniqueNeurons(Topology::kLyr_Input);
  int numUniqueHidden = getNumUniqueNeurons(Topology::kLyr_Hidden);
  int numUniqueOutputs = getNumUniqueNeurons(Topology::kLyr_Output);
  int numInputs = getNumInputs();
  int numInputNeurons = getInputsAreNeurons() ? numInputs : 0;
  int numHidden = getNumHidden();
  
  // Input gains, if inputs are not neurons (in which case weights take care of individual weighting)
  int lastInput = numInputs - 1;  
  if(!getInputsAreNeurons())
  {
    for(int i = 0; i < numUniqueInputs; ++i)
    {
      double gain = map01To(params[I], limits.gain);
      ctrnn.setGain(i, gain);
      ++I;
      
      if(isSymmetric())
      {
        ctrnn.setGain(lastInput - i, gain);  
      }
    }
  }
  
  // Weights from input to hidden layer
  int firstHidden = numInputNeurons;  
  int lastHidden = numInputNeurons + numHidden - 1;
  if(getInputsAreNeurons())
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
        if(isSymmetric())
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
    
    if(isSymmetric())
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
      
      if(isSymmetric())
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
  int lastOutput = firstOutput + getNumOutputs() - 1;
  for(int i = 0; i < numUniqueOutputs; ++i)
  {
    double bias = map01To(params[I], limits.bias);
    double tau = map01To(params[I + 1], limits.tau);
    ctrnn.setBias(firstOutput + i, bias);
    ctrnn.setTimeConstant(firstOutput + i, tau);
    I += 2;
    
    if(isSymmetric())
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
      if(isSymmetric())
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
  
  
  // Output intralayer weights
  if(getOutputsAreLaterallyConnected())
  {
    for(int i = 0; i < numUniqueOutputs; ++i)
    {
      for(int j = 0; j < getNumOutputs(); ++j)
      {
        double weight = map01To(params[I], limits.weight);
        int from = firstOutput + j;
        int to = firstOutput + i;
        ctrnn.setWeight(from, to, weight);
        ++I;
        
        if(isSymmetric())
        {
          // Ensure middle output, if odd number, isn't decoded twice
          int toSym = lastOutput - i;
          if(toSym != to)
          {
            int fromSym = lastOutput - j;
            ctrnn.setWeight(fromSym, toSym, weight);
          }
        }
      }
    }  
  }
  
  // Check we decoded correctly!
  int numReqParams = getNumParameters();
  bool correct = I == numReqParams;
  assert(correct);
  
  return correct;
}


//----------------------------------------------------------------------------------------------------------------------
void Topology::toXml(ci::XmlTree& xml) const
{
  ci::XmlTree topXml ("Topology", "");
  topXml.setAttribute("Symmetric", symmetric);
  
  ci::XmlTree inputs ("Inputs", toString(size[0]));
  inputs.setAttribute("asNeurons", inputsAreNeurons);
  topXml.push_back(inputs);
  
  topXml.push_back(ci::XmlTree ("Hidden", toString(size[1])));
  
  ci::XmlTree outputs ("Outputs", toString(size[2]));
  outputs.setAttribute("laterallyConnected", outputsLaterallyConnected);  
  topXml.push_back(outputs);
  
  xml.push_back(topXml);
}
  
//----------------------------------------------------------------------------------------------------------------------  
void Topology::fromXml(const ci::XmlTree& xml)
{
  if (xml.getTag() == "Topology")
  {
    symmetric = xml.getAttributeValue<bool>("symmetric");
    
    size[0] = xml.getChild("Inputs").getValue<int>();
    size[1] = xml.getChild("Hidden").getValue<int>();
    size[2] = xml.getChild("Outputs").getValue<int>();
    
    inputsAreNeurons = xml.getChild("Inputs").getAttributeValue<bool>("asNeurons");
    outputsLaterallyConnected = xml.getChild("Outputs").getAttributeValue<bool>("laterallyConnected");        
  }
}

} // namespace