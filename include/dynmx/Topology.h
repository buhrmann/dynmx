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
  
  // Define a type of pointer to member(!) update function
  typedef void (CTRNN::*Setter1Func)(int, double);
  typedef void (CTRNN::*Setter2Func)(int, int, double);
  typedef double (CTRNN::*Getter1Func)(int) const;
  typedef double (CTRNN::*Getter2Func)(int, int) const;
  
  Topology();
  Topology(const ci::XmlTree& xml);
  virtual ~Topology(){ destroyMatrix(); };
  
  // Convenience setters and getters
  void setSize(int l0, int l1, int l2) { destroyMatrix(); m_size[0] = l0; m_size[1] = l1; m_size[2] = l2; m_N = calcSize(); buildMatrix(); };
  void setSymmetric(bool s) { m_symmetric = s; };
  void setInputsAreNeurons(bool b) { m_inputsAreNeurons = b; };
  
  int getSize() const { return m_N; };
  bool isSymmetric() const { return m_symmetric; };
  bool getInputsAreNeurons() const { return m_inputsAreNeurons; };
  bool getOutputsLateral() const { return m_outputsLateral; };
  bool getOutputsSelf() const { return m_outputsSelf; };
  bool getHiddenLateral() const { return m_hiddenLateral; };
  bool getHiddenSelf() const { return m_hiddenSelf; };
  
  // Returns number of parameters required to encode the given topology
  int getNumInputs() const { return m_size[kLyr_Input]; };
  int getNumHidden() const { return m_size[kLyr_Hidden]; };
  int getNumOutputs() const { return m_size[kLyr_Output]; };
  int getNumUniqueNeurons(LayerName layer) const;
  int getNumUniqueConnections() const { return m_numConn; };
  void getLayerFirstLast(LayerName l, int& first, int& last, bool unique=false) const;
  int getLayerFirst(LayerName l) const;
  
  virtual void toXml(ci::XmlTree& xml) const;
  virtual void fromXml(const ci::XmlTree& xml);
  
  // These are crucial for GA
  virtual int getNumParameters() const { return m_numParams; };
  virtual bool decode(CTRNN& ctrnn, const double* params) const;
  virtual bool encode(CTRNN& ctrnn, double* params) const;
  
  bool isHidden(int n) const { return (n >= m_size[kLyr_Input]) && (n < (m_size[kLyr_Input] + m_size[kLyr_Hidden])); };
  bool isOutput(int n) const { return n >= (m_size[kLyr_Input] + m_size[kLyr_Hidden]); };
  bool connected(int from, int to) const { return m_connections[from][to]; };
  
  void randomiseWeights(CTRNN* net, float min, float max);
  void randomiseWeights(CTRNN* net) { randomiseWeights(net, m_limits.weight.min, m_limits.weight.max);} ;
  
protected:
  
  void decodeLayerParams(CTRNN& net, LayerName layer, const double* params, int& I, const Range& r, Setter1Func f) const;
  void encodeLayerParams(CTRNN& net, LayerName layer, double* params, int& I, const Range& r, Getter1Func f) const;
  void decodeLayerConnections(CTRNN& net, LayerName pre, LayerName post, const double* params, int& I, const Range& r, Setter2Func f, bool slf=true) const;
  void encodeLayerConnections(CTRNN& net, LayerName pre, LayerName post, double* params, int& I, const Range& r, Getter2Func f, bool slf=true) const;
  
  inline double cutWeight(double w) const { return std::abs(w) >= m_weightCutoff ? w : 0; };
  int calcSize() const { int s = m_size[1] + m_size[2]; return (m_inputsAreNeurons ? s + m_size[0] : s); };
  bool calcConn(int from, int to) const;
  int calcNumUniqueConnections();
  int calcNumParameters();
  void destroyMatrix();
  void buildMatrix();
  
  int m_N;
  int m_size[3];
  bool m_symmetric;
  bool m_inputsAreNeurons;
  bool m_outputsLateral;
  bool m_hiddenLateral;
  bool m_hiddenSelf;
  bool m_outputsSelf;
  double m_weightCutoff;
  bool **m_connections; // cache connectivity for quicker checks
  
  int m_numConn;
  int m_numParams;
  
  NetLimits m_limits;
};  
  
} // namespace

#endif