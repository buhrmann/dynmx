/*
 *  Topology.h
 *  cinder_dynmx
 *
 *  Created by Thomas Buhrmann on 3/22/12.
 *  Copyright 2012 __MyCompanyName__. All rights reserved.
 *
 */

#ifndef DMX_TOPOLOGY_H
#define DMX_TOPOLOGY_H

#include "Dynmx.h"
#include "MathUtils.h"

namespace dmx
{
  
class CTRNN;  
  
// Helper for storing the ranges of CTRNN parameters
//--------------------------------------------------------------------------------------------------------------------
struct NetLimits
{
  NetLimits() : weight(-10,10), bias(-10,10), gain(-10,10), tau(0.2,2.0) {};
  void toXml(ci::XmlTree& xml) const;
  void fromXml(const ci::XmlTree& xml);  
  
  Range weight, bias, tau, gain;
};  

//----------------------------------------------------------------------------------------------------------------------  
// Defines a topology for decoding a neural network
//----------------------------------------------------------------------------------------------------------------------
class Topology
{ 
public:
  
  enum LayerName
  {
    kLyr_Input = 0,
    kLyr_Hidden,
    kLyr_Output,
    kLyr_NumLayers
  };
  
  Topology() : inputsAreNeurons(true), weightCutoff(0) { setSize(1, 2, 0); setSymmetric(true); };
  Topology(int l0, int l1, int l2, bool inputsAsNeurons = true) : inputsAreNeurons(inputsAsNeurons) { setSize(l0, l1, l2); setSymmetric(true); };
  ~Topology(){};
  
  // Convenience methods
  void setSize(int l0, int l1, int l2) { size[0] = l0; size[1] = l1; size[2] = l2; };
  void setSymmetric(bool s) { symmetric = s; };
  void setInputsAreNeurons(bool b) { inputsAreNeurons = b; };
  
  int getSize() const { int s = size[1] + size[2]; return (inputsAreNeurons ? s + size[0] : s); };
  bool isSymmetric() const { return symmetric; };
  bool getInputsAreNeurons() const { return inputsAreNeurons; };
  bool getOutputsAreLaterallyConnected() const { return outputsLaterallyConnected; };
  
  // Returns number of parameters required to encode the given topology
  int getNumInputs() const { return size[kLyr_Input]; };
  int getNumHidden() const { return size[kLyr_Hidden]; };
  int getNumOutputs() const { return size[kLyr_Output]; };
  int getNumParameters() const;
  
  int getNumUniqueNeurons(LayerName layer) const;
  
  void toXml(ci::XmlTree& xml) const;
  void fromXml(const ci::XmlTree& xml);
  
  bool decode(CTRNN& ctrnn, const double* params, const NetLimits& limits) const;
  bool encode(CTRNN& ctrnn, double* params, const NetLimits& limits) const;
  
protected:
  
  inline double cutWeight(double w) const { return std::abs(w) >= weightCutoff ? w : 0; };
  
  int size[3];
  bool symmetric;
  bool inputsAreNeurons;
  bool outputsLaterallyConnected;
  double weightCutoff;
};  
  
} // namespace

#endif