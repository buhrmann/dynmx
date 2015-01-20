//
//  AdapTop.cpp
//  dynmx
//
//  Created by Thomas Buhrmann on 09/09/14.
//
//

#include "AdapTop.h"
#include "AdapNN.h"

namespace dmx
{
  
//----------------------------------------------------------------------------------------------------------------------
// AdapNetLimits
//----------------------------------------------------------------------------------------------------------------------
void AdapNetLimits::toXml(ci::XmlTree& xml) const
{
  meanDecay.toXml(xml, "MeanDecay");
  weightDecay.toXml(xml, "WeightDecay");  
  noiseVar.toXml(xml, "NoiseVar");
  learnRate.toXml(xml, "LearnRate");
  synScale.toXml(xml, "SynapticScale");
  prePostFac.toXml(xml, "PrePostFac");
}

//----------------------------------------------------------------------------------------------------------------------
void AdapNetLimits::fromXml(const ci::XmlTree& xml)
{
  if (xml.getTag() == "NetLimits")
  {
    meanDecay.fromXml(xml, "MeanDecay");
    weightDecay.fromXml(xml, "WeightDecay");
    noiseVar.fromXml(xml, "NoiseVar");
    learnRate.fromXml(xml, "LearnRate");
    synScale.fromXml(xml, "SynapticScale");
    prePostFac.fromXml(xml, "PrePostFac");
  }
}


//----------------------------------------------------------------------------------------------------------------------
// AdapTop
//----------------------------------------------------------------------------------------------------------------------
AdapTop::AdapTop() :
  Topology(),
  m_initialWeights(false),
  m_rulePerConnection(false)
{
};


// Base class constructor calls virtual fromXml function (i.e. below will be called automatically)
//----------------------------------------------------------------------------------------------------------------------
AdapTop::AdapTop(const ci::XmlTree& xml) :
  Topology(xml)
{
};
  
// Assumes xml argument has the "Topology" tag
//----------------------------------------------------------------------------------------------------------------------
void AdapTop::fromXml(const ci::XmlTree& xml)
{
  Topology::fromXml(xml);
  
  m_initialWeights = xml.getAttributeValue<bool>("initialWeights", true);
  m_rulePerConnection = xml.getAttributeValue<bool>("rulePerConn", true);
  m_noiseUniform = xml.getAttributeValue<bool>("noiseUniform", true);
  m_useAntiHebb = xml.getAttributeValue<bool>("antiHebb", false);
  m_actFunc = xml.getAttributeValue<std::string>("actFunc", "Sigmoid");
  m_evolveBiases = xml.getAttributeValue<bool>("evolveBiases", false);
  m_evolveTaus = xml.getAttributeValue<bool>("evolveTaus", false);
  m_evolvePrePostFac = xml.getAttributeValue<bool>("evolvePrePostFac", false);
  
  std::string scaleName = xml.getAttributeValue<std::string>("synScaling", "None");
  if (scaleName == "Evolve") {
    m_evolveSynScaleMode = true;
    m_scaling = AdapNN::kSynSc_None;
  }
  else {
    m_evolveSynScaleMode = false;
    m_scaling = AdapNN::getScaling(scaleName);
  }
  
  if(xml.hasChild("NetLimits"))
  {
    m_limits.fromXml(xml.getChild("NetLimits"));
  }
}
  
// Assumes appending to xml argument
//----------------------------------------------------------------------------------------------------------------------
void AdapTop::toXml(ci::XmlTree& xml) const
{
  // This appends a "Topology"-tagged tree to xml
  Topology::toXml(xml);
  
  // Now append to it
  ci::XmlTree& top = xml.getChild("Topology");
  top.setAttribute("adaptive", 1);
  top.setAttribute("initialWeights", m_initialWeights);
  top.setAttribute("rulePerConn", m_rulePerConnection);
  top.setAttribute("noiseUniform", m_noiseUniform);
  top.setAttribute("antiHebb", m_useAntiHebb);
  top.setAttribute("actFunc", m_actFunc);
  top.setAttribute("synScaling", AdapNN::s_synScalingNames[m_scaling]);
  top.setAttribute("evolveBiases", m_evolveBiases);
  top.setAttribute("evolveTaus", m_evolveTaus);
  top.setAttribute("evolvePrePostFac", m_evolvePrePostFac);
  
  // Add limits
  ci::XmlTree& limXml = xml.getChild("Topology/NetLimits");
  m_limits.toXml(limXml);
}

//----------------------------------------------------------------------------------------------------------------------
void AdapTop::reset(CTRNN* net)
{
  // Randomise
  const NetLimits& lm = Topology::m_limits;
  randomiseWeights(net, 0.1 * lm.weight.min, 0.1 * lm.weight.max);
  
  if(!m_evolveBiases)
  {
    if (lm.bias.min == lm.bias.max){
      net->setBiases(lm.bias.min);
    }
    else{
      net->randomizeBiases(lm.bias.min, lm.bias.max);
    }
  }
  
  if(!m_evolveTaus)
  {
    if (lm.tau.min == lm.tau.max){
      net->setTimeConstants(lm.tau.min);
    }
    else{
      net->randomizeTimeConstants(lm.tau.min, lm.tau.max);
    }
  }
}
  
//----------------------------------------------------------------------------------------------------------------------
int AdapTop::getNumAdapParams() const
{
  int synSpecParams = 3;  // lrate and wdecay and synscale
  if (m_evolvePrePostFac) {
    synSpecParams += 2;   // pre and post factors
  }
  
  int neuSpecParams = 1; // mean decay
  
  int netSpecParams = 2; // reward decay, noise variance
  if (m_evolveSynScaleMode) {
    netSpecParams += 1;
  }
  
  if(! m_rulePerConnection)
  {
    // All neurons and synapses behave the same
    return synSpecParams + neuSpecParams + netSpecParams;
  }
  else
  {
    // Neurons and synapses have separate learning rules
    // Unique adaptive synapses are all unique synapses except for self-connections if they exist (accounting for symmetry):
    int numUniqueAdapSyn = getNumUniqueConnections();
    if (m_hiddenSelf)
      numUniqueAdapSyn -= getNumUniqueNeurons(kLyr_Hidden);
    if (m_outputsSelf)
      numUniqueAdapSyn -= getNumUniqueNeurons(kLyr_Output);
    
    const int numUniqueNeurons = getNumUniqueNeurons(kLyr_Input) + getNumUniqueNeurons(kLyr_Hidden) + getNumUniqueNeurons(kLyr_Output);
    return (numUniqueAdapSyn * synSpecParams) + (numUniqueNeurons * neuSpecParams) + netSpecParams;
  }
}
  
//----------------------------------------------------------------------------------------------------------------------
int AdapTop::getNumParameters() const
{
  if (m_initialWeights)
  {
    // All original CTRNN parameters + adaptive ones
    return Topology::getNumParameters() + getNumAdapParams();
  }
  else
  {
    // Adaptive parameters
    int N = 0;
    N += getNumAdapParams();
    
    // Non-adaptive parameters
    // Input gains
    if(!m_inputsAreNeurons)
      N += getNumUniqueNeurons(kLyr_Input);

    // Biases and time constants
    if (m_evolveBiases)
      N += getNumUniqueNeurons(kLyr_Hidden) + getNumUniqueNeurons(kLyr_Output);

    if (m_evolveTaus)
      N += getNumUniqueNeurons(kLyr_Hidden) + getNumUniqueNeurons(kLyr_Output);
    
    return N;
  }
}

  
// Create AdapNN from genome params
//----------------------------------------------------------------------------------------------------------------------
bool AdapTop::decode(CTRNN& net, const double* params) const
{
  int I = 0;
  
  if(m_initialWeights)
  {
    // Superclass already does normal decoding
    bool success = Topology::decode(net, params);
    assert(success);
    I += Topology::getNumParameters();
  }
  else
  {
    // Manual decoding without weights, just input gains, biases and time constants
    // Input gains, if inputs are not neurons (in which case weights take care of individual weighting)
    if(m_inputsAreNeurons)
    {
      for(int i = 0; i < m_size[kLyr_Input]; ++i)
        net.setInputNeuron(i);
    }
    else
    {
      decodeLayerParams(net, kLyr_Input, params, I,Topology:: m_limits.gain, &CTRNN::setGain);
    }
    
    // Hidden layer: biases and time constants
    if (m_evolveBiases){
      decodeLayerParams(net, kLyr_Hidden, params, I, Topology::m_limits.bias, &CTRNN::setBias);
      decodeLayerParams(net, kLyr_Output, params, I, Topology::m_limits.bias, &CTRNN::setBias);
    }
    
    if (m_evolveTaus){
      decodeLayerParams(net, kLyr_Hidden, params, I, Topology::m_limits.tau, &CTRNN::setTimeConstant);
      decodeLayerParams(net, kLyr_Output, params, I, Topology::m_limits.tau, &CTRNN::setTimeConstant);
    }
  }
  
  // NASTY: should be done as virtual function ideally!!!
  decodeAdap(((AdapNN&)net), params, I);
  
  const int reqParams = getNumParameters();
  bool correct = I == reqParams;
  assert(correct);
  
#define CHECK_ROUNDTRIP 0
#if CHECK_ROUNDTRIP
  double encParams [I];
  encode(net, &encParams[0]);
  double tolerance = 0.0001;
  bool same = std::equal(params, params+I, encParams, [&tolerance](double a, double b) -> bool { return ((a-b) < tolerance) && ((a-b) > -tolerance); });
  assert(same);
#endif
  
  return correct;
}
  
//----------------------------------------------------------------------------------------------------------------------
void AdapTop::decodeAdap(AdapNN& net, const double* params, int& I) const
{  
  // Network-wide parameters
  // Not evolved but specifiable in Topology xml
  net.setNoiseUniform(m_noiseUniform);
  net.useAntiHebbSwitch(m_useAntiHebb);

  if(m_actFunc == "Tanh")
    net.setActivationFunction(CTRNN::kAF_Tanh);
  
  if (m_evolveSynScaleMode) {
    int mode_idx = (int) (params[I++] * AdapNN::kSynSc_Num);
    AdapNN::SynScaling s =  static_cast<AdapNN::SynScaling> (mode_idx);
    net.setSynapticScaling(s);
  }
  else {
    net.setSynapticScaling(m_scaling);
  }
  
  net.setRewardMeanFilter(m_limits.meanDecay.decode(params[I++]));
  net.setNoiseVar(m_limits.noiseVar.decode(params[I++]));
  
  if(m_rulePerConnection)
  {
    // Neuron-specific parameters: mean activation filter constant
    Setter1Func setter = static_cast<Setter1Func>(&AdapNN::setNeuralMeanFilter);
    decodeLayerParams(net, kLyr_Input, params, I, m_limits.meanDecay, setter);
    decodeLayerParams(net, kLyr_Hidden, params, I, m_limits.meanDecay, setter);
    decodeLayerParams(net, kLyr_Output, params, I, m_limits.meanDecay, setter);
    
    // Synapse-specific parameters: learning rate and weight decay
    Setter2Func wdsetter = static_cast<Setter2Func>(&AdapNN::setWeightDecay);
    Setter2Func lrsetter = static_cast<Setter2Func>(&AdapNN::setLearningRate);
    Setter2Func sssetter = static_cast<Setter2Func>(&AdapNN::setSynapticScale);
    Setter2Func presetter = static_cast<Setter2Func>(&AdapNN::setPreFac);
    Setter2Func postsetter = static_cast<Setter2Func>(&AdapNN::setPostFac);
    
    if(m_inputsAreNeurons)
    {
      decodeLayerConnections(net, kLyr_Input, kLyr_Hidden, params, I, m_limits.weightDecay, wdsetter, false);
      decodeLayerConnections(net, kLyr_Input, kLyr_Hidden, params, I, m_limits.learnRate, lrsetter, false);
      decodeLayerConnections(net, kLyr_Input, kLyr_Hidden, params, I, m_limits.synScale, sssetter, false);
      if(m_evolvePrePostFac) {
        decodeLayerConnections(net, kLyr_Input, kLyr_Hidden, params, I, m_limits.prePostFac, presetter, false);
        decodeLayerConnections(net, kLyr_Input, kLyr_Hidden, params, I, m_limits.prePostFac, postsetter, false);
      }
    }
    
    decodeLayerConnections(net, kLyr_Hidden, kLyr_Output, params, I, m_limits.weightDecay, wdsetter, false);
    decodeLayerConnections(net, kLyr_Hidden, kLyr_Output, params, I, m_limits.learnRate, lrsetter, false);
    decodeLayerConnections(net, kLyr_Hidden, kLyr_Output, params, I, m_limits.synScale, sssetter, false);
    if(m_evolvePrePostFac) {
      decodeLayerConnections(net, kLyr_Hidden, kLyr_Output, params, I, m_limits.prePostFac, presetter, false);
      decodeLayerConnections(net, kLyr_Hidden, kLyr_Output, params, I, m_limits.prePostFac, postsetter, false);
    }
    
    if(m_hiddenLateral)
    {
      decodeLayerConnections(net, kLyr_Hidden, kLyr_Hidden, params, I, m_limits.weightDecay, wdsetter, false);
      decodeLayerConnections(net, kLyr_Hidden, kLyr_Hidden, params, I, m_limits.learnRate, lrsetter, false);
      decodeLayerConnections(net, kLyr_Hidden, kLyr_Hidden, params, I, m_limits.synScale, sssetter, false);
      if(m_evolvePrePostFac) {
        decodeLayerConnections(net, kLyr_Hidden, kLyr_Hidden, params, I, m_limits.prePostFac, presetter, false);
        decodeLayerConnections(net, kLyr_Hidden, kLyr_Hidden, params, I, m_limits.prePostFac, postsetter, false);
      }
    }
    
    if(m_outputsLateral)
    {
      decodeLayerConnections(net, kLyr_Output, kLyr_Output, params, I, m_limits.weightDecay, wdsetter, false);
      decodeLayerConnections(net, kLyr_Output, kLyr_Output, params, I, m_limits.learnRate, lrsetter, false);
      decodeLayerConnections(net, kLyr_Output, kLyr_Output, params, I, m_limits.synScale, sssetter, false);
      if(m_evolvePrePostFac) {
        decodeLayerConnections(net, kLyr_Output, kLyr_Output, params, I, m_limits.prePostFac, presetter, false);
        decodeLayerConnections(net, kLyr_Output, kLyr_Output, params, I, m_limits.prePostFac, postsetter, false);
      }
    }
  }
  else
  {
    // Same rule for all connections
    // Neuron-specific parameters
    net.setNeuralMeanFilters(m_limits.meanDecay.decode(params[I++]));
    
    // Synapse-specific parameters
    float wd = m_limits.weightDecay.decode(params[I++]);
    float lr = m_limits.learnRate.decode(params[I++]);
    float ss = m_limits.synScale.decode(params[I++]);
    float preFac = m_limits.prePostFac.decode(params[I++]);
    float postFac = m_limits.prePostFac.decode(params[I++]);
    
    for(int i = 0; i < m_N; ++i)
    {
      for(int j = 0; j < m_N; ++j)
      {
        if(connected(i,j) && (i != j))
        {
          net.setWeightDecay(i, j, wd);
          net.setLearningRate(i, j, lr);
          net.setSynapticScale(i, j, ss);
          net.setPrePostFac(i, j, preFac, postFac);
        }
        else
        {
          net.setWeightDecay(i, j, 0);
          net.setLearningRate(i, j, 0);
          net.setSynapticScale(i, j, 0);
          net.setPrePostFac(i, j, 0, 0);
        }
      }
    }
  }
}

//----------------------------------------------------------------------------------------------------------------------
void AdapTop::encodeAdap(AdapNN& net, double* params, int& I) const
{
  // Network-wide parameters
  params[I++] = m_limits.meanDecay.encode(net.getRewardMeanFilter());
  params[I++] = m_limits.noiseVar.encode(net.getNoiseVar());
  
  if(m_rulePerConnection)
  {
    // Neuron-specific parameters: mean activation filter constant
    Getter1Func getter = static_cast<Getter1Func>(&AdapNN::getNeuralMeanFilter);
    encodeLayerParams(net, kLyr_Input, params, I, m_limits.meanDecay, getter);
    encodeLayerParams(net, kLyr_Hidden, params, I, m_limits.meanDecay, getter);
    encodeLayerParams(net, kLyr_Output, params, I, m_limits.meanDecay, getter);
    
    // Synapse-specific parameters: learning rate and weight decay
    Getter2Func wdgetter = static_cast<Getter2Func>(&AdapNN::getWeightDecay);
    Getter2Func lrgetter = static_cast<Getter2Func>(&AdapNN::getLearningRate);
    Getter2Func ssgetter = static_cast<Getter2Func>(&AdapNN::getSynapticScale);
    Getter2Func pregetter = static_cast<Getter2Func>(&AdapNN::getPreFac);
    Getter2Func postgetter = static_cast<Getter2Func>(&AdapNN::getPostFac);
    
    if(m_inputsAreNeurons)
    {
      encodeLayerConnections(net, kLyr_Input, kLyr_Hidden, params, I, m_limits.weightDecay, wdgetter, false);
      encodeLayerConnections(net, kLyr_Input, kLyr_Hidden, params, I, m_limits.learnRate, lrgetter, false);
      encodeLayerConnections(net, kLyr_Input, kLyr_Hidden, params, I, m_limits.synScale, ssgetter, false);
      if (m_evolvePrePostFac) {
        encodeLayerConnections(net, kLyr_Input, kLyr_Hidden, params, I, m_limits.prePostFac, pregetter, false);
        encodeLayerConnections(net, kLyr_Input, kLyr_Hidden, params, I, m_limits.prePostFac, postgetter, false);
      }
    }
    
    encodeLayerConnections(net, kLyr_Hidden, kLyr_Output, params, I, m_limits.weightDecay, wdgetter, false);
    encodeLayerConnections(net, kLyr_Hidden, kLyr_Output, params, I, m_limits.learnRate, lrgetter, false);
    encodeLayerConnections(net, kLyr_Hidden, kLyr_Output, params, I, m_limits.synScale, ssgetter, false);
    if (m_evolvePrePostFac) {
      encodeLayerConnections(net, kLyr_Hidden, kLyr_Output, params, I, m_limits.prePostFac, pregetter, false);
      encodeLayerConnections(net, kLyr_Hidden, kLyr_Output, params, I, m_limits.prePostFac, postgetter, false);
    }
    
    if(m_hiddenLateral)
    {
      encodeLayerConnections(net, kLyr_Hidden, kLyr_Hidden, params, I, m_limits.weightDecay, wdgetter, false);
      encodeLayerConnections(net, kLyr_Hidden, kLyr_Hidden, params, I, m_limits.learnRate, lrgetter, false);
      encodeLayerConnections(net, kLyr_Hidden, kLyr_Hidden, params, I, m_limits.synScale, ssgetter, false);
      if (m_evolvePrePostFac) {
        encodeLayerConnections(net, kLyr_Hidden, kLyr_Hidden, params, I, m_limits.prePostFac, pregetter, false);
        encodeLayerConnections(net, kLyr_Hidden, kLyr_Hidden, params, I, m_limits.prePostFac, postgetter, false);
      }
    }
    
    if(m_outputsLateral)
    {
      encodeLayerConnections(net, kLyr_Output, kLyr_Output, params, I, m_limits.weightDecay, wdgetter, false);
      encodeLayerConnections(net, kLyr_Output, kLyr_Output, params, I, m_limits.learnRate, lrgetter, false);
      encodeLayerConnections(net, kLyr_Output, kLyr_Output, params, I, m_limits.synScale, ssgetter, false);
      if (m_evolvePrePostFac) {
        encodeLayerConnections(net, kLyr_Output, kLyr_Output, params, I, m_limits.prePostFac, pregetter, false);
        encodeLayerConnections(net, kLyr_Output, kLyr_Output, params, I, m_limits.prePostFac, postgetter, false);
      }
      
    }
  }
  else
  {
    // Neuron-specific parameter (all identical in this case)
    params[I++] = m_limits.meanDecay.encode(net.getNeuralMeanFilter(0));
    
    // Synapse-specific parameter (also identical).
    // First adaptive synapse should be from input to first hidden (no incoming to inputs, nor self-connection).
    int firstHidden = getLayerFirst(kLyr_Hidden);
    params[I++] = m_limits.weightDecay.encode(net.getWeightDecay(0, firstHidden));
    params[I++] = m_limits.learnRate.encode(net.getLearningRate(0, firstHidden));
    params[I++] = m_limits.synScale.encode(net.getSynapticScale(0, firstHidden));
    params[I++] = m_limits.prePostFac.encode(net.getPreFac(0, firstHidden));
    params[I++] = m_limits.prePostFac.encode(net.getPostFac(0, firstHidden));
  }
}


//----------------------------------------------------------------------------------------------------------------------
bool AdapTop::encode(CTRNN& net, double* params) const
{
  int I = 0;
  
  if(m_initialWeights)
  {
    // Superclass already does normal decoding
    bool success = Topology::encode(net, params);
    assert(success);
    I += Topology::getNumParameters();
  }
  else
  {
    // Input gains, if inputs are not neurons (in which case weights take care of individual weighting)
    if(!m_inputsAreNeurons)
    {
      encodeLayerParams(net, kLyr_Input, params, I, Topology::m_limits.gain, &CTRNN::getGain);
    }
    
    // Neural parameters
    if (m_evolveBiases){
      encodeLayerParams(net, kLyr_Hidden, params, I, Topology::m_limits.bias, &CTRNN::getBias);
      encodeLayerParams(net, kLyr_Output, params, I, Topology::m_limits.bias, &CTRNN::getBias);
    }
    
    if (m_evolveTaus){
      encodeLayerParams(net, kLyr_Hidden, params, I, Topology::m_limits.tau, &CTRNN::getTimeConstant);
      encodeLayerParams(net, kLyr_Output, params, I, Topology::m_limits.tau, &CTRNN::getTimeConstant);
    }
  }
  
  // NASTY: should be done as virtual function ideally!!!
  encodeAdap(((AdapNN&)net), params, I);

  const int reqParams = getNumParameters();
  bool correct = I == reqParams;
  assert(correct);
  
  return correct;
}

}