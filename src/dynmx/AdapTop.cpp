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
  
  // Add limits
  ci::XmlTree& limXml = xml.getChild("Topology/NetLimits");
  m_limits.toXml(limXml);
}
  
//----------------------------------------------------------------------------------------------------------------------
int AdapTop::getNumAdapParams() const
{
  const int synSpecParams = 2; // lrate and wdecay
  const int neuSpecParams = 1; // mean decay
  const int netSpecParams = 2; // reward decay, noise variance
  
  if(! m_rulePerConnection)
  {
    // All neurons and synapses behave the same
    return synSpecParams + neuSpecParams + netSpecParams;
  }
  else
  {
    // Neurons and synapses have separate learning rules
    // Unique adaptive synapses are all unique synapses except for self-connections (accounting for symmetry):
    const int numUniqueAdapSyn = getNumUniqueConnections() - getNumUniqueNeurons(kLyr_Hidden) - getNumUniqueNeurons(kLyr_Output);
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
    N += 2 * (getNumUniqueNeurons(kLyr_Hidden) + getNumUniqueNeurons(kLyr_Output));
    
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
    decodeLayerParams(net, kLyr_Hidden, params, I, Topology::m_limits.bias, &CTRNN::setBias);
    decodeLayerParams(net, kLyr_Output, params, I, Topology::m_limits.bias, &CTRNN::setBias);
    decodeLayerParams(net, kLyr_Hidden, params, I, Topology::m_limits.tau, &CTRNN::setTimeConstant);
    decodeLayerParams(net, kLyr_Output, params, I, Topology::m_limits.tau, &CTRNN::setTimeConstant);
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
  net.setNoiseUniform(m_noiseUniform); // Not evolved but specifiable in Topology xml
  
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

    if(m_inputsAreNeurons)
    {
      decodeLayerConnections(net, kLyr_Input, kLyr_Hidden, params, I, m_limits.weightDecay, wdsetter, false);
      decodeLayerConnections(net, kLyr_Input, kLyr_Hidden, params, I, m_limits.learnRate, lrsetter, false);
    }
    
    decodeLayerConnections(net, kLyr_Hidden, kLyr_Output, params, I, m_limits.weightDecay, wdsetter, false);
    decodeLayerConnections(net, kLyr_Hidden, kLyr_Output, params, I, m_limits.learnRate, lrsetter, false);
    
    if(m_hiddenLaterallyConnected)
    {
      decodeLayerConnections(net, kLyr_Hidden, kLyr_Hidden, params, I, m_limits.weightDecay, wdsetter, false);
      decodeLayerConnections(net, kLyr_Hidden, kLyr_Hidden, params, I, m_limits.learnRate, lrsetter, false);
    }
    
    if(m_outputsLaterallyConnected)
    {
      decodeLayerConnections(net, kLyr_Output, kLyr_Output, params, I, m_limits.weightDecay, wdsetter, false);
      decodeLayerConnections(net, kLyr_Output, kLyr_Output, params, I, m_limits.learnRate, lrsetter, false);
    }
  }
  else
  {
    // Neuron-specific parameters
    net.setNeuralMeanFilters(m_limits.meanDecay.decode(params[I++]));
    
    // Synapse-specific parameters
    float wd = m_limits.weightDecay.decode(params[I++]);
    float lr = m_limits.learnRate.decode(params[I++]);
    for(int i = 0; i < m_N; ++i)
    {
      for(int j = 0; j < m_N; ++j)
      {
        if(connected(i,j) && (i != j))
        {
          net.setWeightDecay(i, j, wd);
          net.setLearningRate(i, j, lr);
        }
        else
        {
          net.setWeightDecay(i, j, 0);
          net.setLearningRate(i, j, 0);
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
    
    if(m_inputsAreNeurons)
    {
      encodeLayerConnections(net, kLyr_Input, kLyr_Hidden, params, I, m_limits.weightDecay, wdgetter, false);
      encodeLayerConnections(net, kLyr_Input, kLyr_Hidden, params, I, m_limits.learnRate, lrgetter, false);
    }
    
    encodeLayerConnections(net, kLyr_Hidden, kLyr_Output, params, I, m_limits.weightDecay, wdgetter, false);
    encodeLayerConnections(net, kLyr_Hidden, kLyr_Output, params, I, m_limits.learnRate, lrgetter, false);
    
    if(m_hiddenLaterallyConnected)
    {
      encodeLayerConnections(net, kLyr_Hidden, kLyr_Hidden, params, I, m_limits.weightDecay, wdgetter, false);
      encodeLayerConnections(net, kLyr_Hidden, kLyr_Hidden, params, I, m_limits.learnRate, lrgetter, false);
    }
    
    if(m_outputsLaterallyConnected)
    {
      encodeLayerConnections(net, kLyr_Output, kLyr_Output, params, I, m_limits.weightDecay, wdgetter, false);
      encodeLayerConnections(net, kLyr_Output, kLyr_Output, params, I, m_limits.learnRate, lrgetter, false);
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
    encodeLayerParams(net, kLyr_Hidden, params, I, Topology::m_limits.bias, &CTRNN::getBias);
    encodeLayerParams(net, kLyr_Output, params, I, Topology::m_limits.bias, &CTRNN::getBias);
    encodeLayerParams(net, kLyr_Hidden, params, I, Topology::m_limits.tau, &CTRNN::getTimeConstant);
    encodeLayerParams(net, kLyr_Output, params, I, Topology::m_limits.tau, &CTRNN::getTimeConstant);
  }
  
  // NASTY: should be done as virtual function ideally!!!
  encodeAdap(((AdapNN&)net), params, I);

  const int reqParams = getNumParameters();
  bool correct = I == reqParams;
  assert(correct);
  
  return correct;
}

}