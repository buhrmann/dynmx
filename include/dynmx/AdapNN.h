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
#include "Topology.h"

namespace dmx
{

//----------------------------------------------------------------------------------------------------------------------
// The Adaptive CTRNN class declaration
//----------------------------------------------------------------------------------------------------------------------
class AdapNN : public CTRNN
{
  
public:
  
  enum SynScaling
  {
    kSynSc_Oja = 0,
    kSynSc_Turr,
    kSynSc_HomeoReg,
    kSynSc_None,
    kSynSc_Num
  };
  
  static std::string s_synScalingNames [kSynSc_Num];

  static SynScaling getScaling(std::string name) {
    return static_cast<SynScaling>(std::distance(s_synScalingNames,
                                                 std::find(s_synScalingNames, s_synScalingNames + kSynSc_None, name)));
  }

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
  void setSynapticScaling(SynScaling s) { m_scaling = s; };
  
  float getReward() { return m_reward; };
  float getRewardMeanFilter() const { return m_rdecay; };
  float getNoiseVar() const { return m_noiseVar; };
  
  // Synpases
  void setLearningRates(double r) { fillArray2d<double>(m_lrate, size, size, r); };
  void setLearningRate(int i, int j, double r) { m_lrate[i][j] = r; };
  void setWeightDecays(double d) { fillArray2d<double>(m_wdecay, size, size, d); };
  void setWeightDecay(int i, int j, double w) { m_wdecay[i][j] = w; };
  
  double getLearningRate(int i, int j) const { return m_lrate[i][j]; };
  double getWeightDecay(int i, int j) const { return m_wdecay[i][j]; };
  
  const double* const* getWeightChanges() const { return m_wdt; };
  const double* const* getLearningRates() const { return m_lrate; };
  const double* const* getWeightDecays() const { return m_wdecay; };
  
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
  
  SynScaling m_scaling;
  
  float m_R;
  float m_meanR;
  float m_reward;
  float m_rdecay;
  float m_noiseVar;
  float m_preMeanSubFac;
  
  // Neuron-specific
  float* m_ndecay;
  double* m_meanOut; // running average of neural activity
  
  // Synapse-specific
  double** m_wdecay;
  double** m_lrate;
  double** m_wdt;
  
  Topology* m_topology;
};

}
#endif
