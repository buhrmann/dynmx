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

namespace dmx
{

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
  
  return N;
}

//----------------------------------------------------------------------------------------------------------------------
void Topology::toXml(ci::XmlTree& xml)
{
  ci::XmlTree topXml ("Topology", "");
  topXml.setAttribute("Symmetric", symmetric);
  topXml.setAttribute("InputsAreNeurons", inputsAreNeurons);
  
  topXml.push_back(ci::XmlTree ("Inputs", toString(size[0])));
  topXml.push_back(ci::XmlTree ("Hidden", toString(size[1])));
  topXml.push_back(ci::XmlTree ("Outputs", toString(size[2]))); 
  
  xml.push_back(topXml);
}
  
//----------------------------------------------------------------------------------------------------------------------  
void Topology::fromXml(const ci::XmlTree& xml)
{
  if (xml.getTag() == "Topology")
  {
    symmetric = xml.getAttributeValue<bool>("Symmetric");
    inputsAreNeurons = xml.getAttributeValue<bool>("InputsAreNeurons");
    size[0] = xml.getChild("Inputs").getValue<int>();
    size[1] = xml.getChild("Hidden").getValue<int>();
    size[2] = xml.getChild("Outputs").getValue<int>();
  }
}

} // namespace