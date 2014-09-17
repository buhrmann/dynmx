//
//  AdapNN.h
//  dynmx
//
//  Created by Thomas Buhrmann on 02/09/14.
//
//

#ifndef dynmx_AdapNN_h
#define dynmx_AdapNN_h

#include "CTRNN.h"
#include "AdapTop.h"

namespace dmx
{

//----------------------------------------------------------------------------------------------------------------------
// The Adaptive CTRNN class declaration
//----------------------------------------------------------------------------------------------------------------------
class AdapNN : public CTRNN
{
  
public:
  
  AdapNN(Topology* top);
  virtual ~AdapNN();
  
  virtual void updateDynamic(double dt);
  
  // Network
  void setReward(float r);
  void setNoiseUniform(bool u) { m_noiseUniform = u; };
  void useAntiHebbSwitch(bool b) { m_useAntiHebbSwitch = b; };
  void setRewardMeanFilter(float d) { m_rdecay = d; };
  void setNoiseVar(float v) { m_noiseVar = v; };
  void setPresynMeanFactor(float f) { m_preMeanSubFac = f; };
  
  float getRewardMeanFilter() const { return m_rdecay; };
  float getNoiseVar() const { return m_noiseVar; };
  
  // Synpases
  void setLearningRates(double r) { fillArray2d<float>(m_lrate, size, size, r); };
  void setLearningRate(int i, int j, double r) { m_lrate[i][j] = r; };
  void setWeightDecays(double d) { fillArray2d<float>(m_wdecay, size, size, d); };
  void setWeightDecay(int i, int j, double w) { m_wdecay[i][j] = w; };
  
  double getLearningRate(int i, int j) const { return m_lrate[i][j]; };
  double getWeightDecay(int i, int j) const { return m_wdecay[i][j]; };
  
  // Neurons
  void setNeuralMeanFilters(double d) { std::fill(m_ndecay, m_ndecay + size, d); };
  void setNeuralMeanFilter(int i, double d) { m_ndecay[i] = d; };
  
  double getNeuralMeanFilter(int i) const { return m_ndecay[i]; };
    
  bool adaptive(int from, int to) { return (from != to) && m_topology->connected(from, to); };
  
  void toXml(ci::XmlTree& xml);
  void fromXml(const ci::XmlTree& xml);
  
protected:
  
  // Network-specific
  bool m_noiseUniform;
  bool m_useAntiHebbSwitch;
  
  float m_R;
  float m_meanR;
  float m_rdecay;
  float m_noiseVar;
  float m_preMeanSubFac;
  
  // Neuron-specific
  float* m_ndecay;
  double* m_meanOut; // running average of neural activity
  
  // Synapse-specific
  float** m_wdecay;
  float** m_lrate;
  
  Topology* m_topology;
};

}
#endif
