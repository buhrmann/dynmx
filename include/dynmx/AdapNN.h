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
    kSynSc_HomeoPre,
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
  
  virtual void reset();
  
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
  
  void usesReward(bool b) { m_useReward = b; };
  bool usesReward() const { return m_useReward; };
  
  // Synpases
  void setLearningRates(double r) { fillArray2d<double>(m_lrate, size, size, r); };
  void setLearningRate(int i, int j, double r) { m_lrate[i][j] = r; };
  void setWeightDecays(double d) { fillArray2d<double>(m_wdecay, size, size, d); };
  void setWeightDecay(int i, int j, double w) { m_wdecay[i][j] = w; };
  void setSynapticScales(double s) { fillArray2d<double>(m_synscale, size, size, s); };
  void setSynapticScale(int i, int j, double s) { m_synscale[i][j] = s; };
  
  void setPreFacs(double d) { fillArray2d<double>(m_preFac, size, size, d); };
  void setPreFac(int i, int j, double d) { m_preFac[i][j] = d; };
  void setPostFacs(double d) { fillArray2d<double>(m_postFac, size, size, d); };
  void setPostFac(int i, int j, double d) { m_postFac[i][j] = d; };
  void setPrePostFac(int i, int j, double preFac, double postFac) { m_preFac[i][j] = preFac; m_postFac[i][j] = postFac; };

  double getLearningRate(int i, int j) const { return m_lrate[i][j]; };
  double getWeightDecay(int i, int j) const { return m_wdecay[i][j]; };
  double getSynapticScale(int i, int j) const { return m_synscale[i][j]; };
  double getPreFac(int i, int j) const  { return m_preFac[i][j]; };
  double getPostFac(int i, int j) const  { return m_postFac[i][j]; };
  
  const double* const* getWeightChanges() const { return m_dw; };
  const double* const* getLearningRates() const { return m_lrate; };
  const double* const* getWeightDecays() const { return m_wdecay; };
  const double* const* getSynapticScales() const { return m_synscale; };
  
  // Neurons
  void setNeuralMeanFilters(double d) { std::fill(m_ndecay, m_ndecay + size, d); };
  void setNeuralMeanFilter(int i, double d) { m_ndecay[i] = d; };
  double getNeuralMeanFilter(int i) const { return m_ndecay[i]; };
  double getMeanOutput(int i) const { return m_meanAct[i]; };

  
  bool adaptive(int from, int to) { return (from != to) && m_topology->connected(from, to); };
  
  void toXml(ci::XmlTree& xml);
  void fromXml(const ci::XmlTree& xml);
  
protected:
  
  void newWeightPerturbation(float var);
  void newNodePerturbation();
  virtual void updateHebb(double dt);
  virtual void updateRHebb(double dt);
  virtual void updateHebbStab(double dt);
  virtual void updateWeightPerturb(double dt);
  virtual void updateWeightPerturbElig(double dt);
  virtual void updateNodePerturb(double dt);
  virtual void updateNodePerturbElig(double dt);
  virtual void updateNodePerturbCont(double dt);
  
  virtual float synapticScaling(int pre, int post);
  virtual void hardLimitDw(int pre, int post);
  void validate(int i, int j);
  inline bool isNum(double n) { return (n==n) && std::isfinite(n); };
  
  // Network-specific
  bool m_noiseUniform;
  bool m_useAntiHebbSwitch;
  
  SynScaling m_scaling;
  
  bool m_useReward;
  float m_R;
  float m_meanR;
  float m_reward;
  float m_rdecay;
  float m_noiseVar;
  float m_preMeanSubFac;
  double m_wmax;
  
  // Neuron-specific
  double* m_ndecay;
  double* m_meanAct; // running average of neural activity
  
  // Synapse-specific
  double** m_wdecay;
  double** m_lrate;
  double** m_synscale;
  double** m_preFac;
  double** m_postFac;
  double** m_dw;
  
  Topology* m_topology;
  
  std::string m_mode;
  
  // Weight perturbation
  float m_time;
  double m_R0;
  double m_RM;
  double** m_wold;
  int m_epoch;
  
  // Node perturbation
  double* m_bold;
  double* m_db;
};

}
#endif
