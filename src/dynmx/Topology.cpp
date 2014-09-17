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
// NetLimits
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

//----------------------------------------------------------------------------------------------------------------------
// Topology
//----------------------------------------------------------------------------------------------------------------------
Topology::Topology() :
  m_N(5),
  m_connections(NULL),
  m_inputsAreNeurons(true),
  m_weightCutoff(0),
  m_symmetric(false),
  m_outputsLaterallyConnected(false),
  m_hiddenLaterallyConnected(true)
{
  setSize(1, 3, 1);
  calcNumParameters();
};

// Create from xml
//----------------------------------------------------------------------------------------------------------------------
Topology::Topology(const ci::XmlTree& xml) :
  m_connections(NULL)
{
  fromXml(xml);
}

//----------------------------------------------------------------------------------------------------------------------
void Topology::destroyMatrix()
{
  // Destroy matrix
  if(m_connections != NULL)
  {
    for(int i = 0; i < m_N; i++)
    {
      delete [] m_connections[i];
    }
    delete [] m_connections;
  }
  
  m_connections = NULL;
}
  
//----------------------------------------------------------------------------------------------------------------------
void Topology::buildMatrix()
{
  if(m_connections == NULL)
  {
    // Create matrix
    m_connections = new bool*[m_N];
    for(int i = 0; i < m_N; i++)
    {
      m_connections[i] = new bool[m_N];
      for(int j = 0; j < m_N; j++)
      {
        m_connections[i][j] = calcConn(i, j);
      }
    }
  }
}

// Returns the number of neurons in a layer that are uniquely encoded, taking into account symmetries and evenness
//----------------------------------------------------------------------------------------------------------------------
int Topology::getNumUniqueNeurons(LayerName layer) const
{
  if(m_symmetric)
  {
    if(even(m_size[layer]))
      return m_size[layer] / 2;       // e.g. 2 if 4 neurons in layer
    else
      return 1 + (m_size[layer] / 2); // e.g. 3 if 5 neurons in layer
  }
  else 
  {
    return m_size[layer];
  }
}

//----------------------------------------------------------------------------------------------------------------------
int Topology::getLayerFirst(LayerName l) const
{
  if (l == kLyr_Input)
  {
    return 0;
  }
  else
  {
    const int numInputs = m_inputsAreNeurons ? m_size[kLyr_Input] : 0;
    if (l == kLyr_Hidden)
      return numInputs;
    else
      return numInputs + m_size[kLyr_Hidden];
  }
}

//----------------------------------------------------------------------------------------------------------------------
void Topology::getLayerFirstLast(LayerName l, int& first, int& last, bool unique) const
{
  first = getLayerFirst(l);
  last = first + (unique ? getNumUniqueNeurons(l) : m_size[l]) - 1;
}

//----------------------------------------------------------------------------------------------------------------------
int Topology::calcNumUniqueConnections()
{
  m_numConn = 0;
  const int numUniqueHidden = getNumUniqueNeurons(kLyr_Hidden);
  const int numUniqueOutputs = getNumUniqueNeurons(kLyr_Output);
  
  // Inputs to hidden
  if(m_inputsAreNeurons)
    m_numConn += numUniqueHidden * m_size[kLyr_Input];
  
  // Hidden intralayer weights
  if(m_hiddenLaterallyConnected)
    m_numConn += numUniqueHidden * m_size[kLyr_Hidden];
  
  // Hidden layer to output
  m_numConn += numUniqueOutputs * m_size[kLyr_Hidden];
  
  // Output intralayer weights
  if(m_outputsLaterallyConnected)
    m_numConn += numUniqueOutputs * m_size[kLyr_Output];
  
  return m_numConn;
}
  
//----------------------------------------------------------------------------------------------------------------------
int Topology::calcNumParameters()
{
  m_numParams = 0;

  // Input layer gains
  if(!m_inputsAreNeurons)
    m_numParams += getNumUniqueNeurons(kLyr_Input);

  // Biases and time constants
  m_numParams += 2 * (getNumUniqueNeurons(kLyr_Hidden) + getNumUniqueNeurons(kLyr_Output));
  
  // Connections
  m_numParams += calcNumUniqueConnections();
  return m_numParams;
}
  
//----------------------------------------------------------------------------------------------------------------------
void Topology::decodeLayerParams(CTRNN& net, LayerName layer, const double* params, int& I, const Range& r, Setter1Func f) const
{
  int first, last;
  getLayerFirstLast(layer, first, last, false);
  const int numUnique = getNumUniqueNeurons(layer);
  
  for(int i = 0; i < numUnique; ++i)
  {
    double p = r.decode(params[I++]);
    (net.*f)(first + i, p);
    
    if(m_symmetric)
      (net.*f)(last - i, p);
  }
}
 
//----------------------------------------------------------------------------------------------------------------------
void Topology::encodeLayerParams(CTRNN& net, LayerName layer, double* params, int& I, const Range& r, Getter1Func f) const
{
  int first, lastUnique;
  getLayerFirstLast(layer, first, lastUnique, true);

  for(int i = first; i <= lastUnique; ++i)
  {
    params[I++] = r.encode((net.*f)(i));
  }
}
  
// Decodes weights from layer pre to layer post (can be same layer)
//----------------------------------------------------------------------------------------------------------------------
void Topology::decodeLayerConnections(CTRNN& net, LayerName layerPre, LayerName layerPost, const double* params, int& I,
                                     const Range& r, Setter2Func f, bool self) const
{
  int firstPre, lastPre;
  int firstPost, lastPost;
  getLayerFirstLast(layerPre, firstPre, lastPre, false);
  getLayerFirstLast(layerPost, firstPost, lastPost, false);
  const int numUniquePost = getNumUniqueNeurons(layerPost);
  
  int from, to;
  for(int i = 0; i < numUniquePost; ++i)
  {
    for(int j = 0; j < m_size[layerPre]; ++j)
    {
      from = firstPre + j;
      to = firstPost + i;
      
      // Check how to handle self-connections
      if (self || (from != to))
      {
        double p = r.decode(params[I++]);
        (net.*f)(from, to, p);
        
        if(m_symmetric)
        {
          // Ensure middle neurons, if odd number, isn't decoded twice
          int toSym = lastPost - i;
          if(toSym != to)
          {
            int fromSym = lastPre - j;
            (net.*f)(fromSym, toSym, p);
          }
        } // symmetric
      } // self-connection
    }
  }
}
  
//----------------------------------------------------------------------------------------------------------------------
void Topology::encodeLayerConnections(CTRNN& net, LayerName pre, LayerName post, double* params, int& I, const Range& r,
                                      Getter2Func f, bool self) const
{
  int numUniquePost = getNumUniqueNeurons(post);
  int numPre = m_size[pre];
  int firstPre = getLayerFirst(pre);
  int firstPost = getLayerFirst(post);
  
  for(int i = 0; i < numUniquePost; ++i)
  {
    for(int j = 0; j < numPre; ++j)
    {
      int from = firstPre + j;
      int to = firstPost + i;
      if (self || (from != to))
        params[I++] = r.encode((net.*f)(from, to));
    }
  }
}
  
//----------------------------------------------------------------------------------------------------------------------  
// Input neurons are placeholder neurons only and are arranged in a layer different from real neurons.
// Real neurons are are arranged in a bilaterally symmetric layer. I.e. first and last neuron are identical, as well
// as the second and second to last etc...
//----------------------------------------------------------------------------------------------------------------------
bool Topology::decode(CTRNN& ctrnn, const double* params) const
{
  int I = 0;
  
  // Input gains, if inputs are not neurons (in which case weights take care of individual weighting)
  if(m_inputsAreNeurons)
  {
    for(int i = 0; i < m_size[kLyr_Input]; ++i)
      ctrnn.setInputNeuron(i);
  }
  else
  {
    decodeLayerParams(ctrnn, kLyr_Input, params, I, m_limits.gain, &CTRNN::setGain);
  }
  
  // Neural parameters
  decodeLayerParams(ctrnn, kLyr_Hidden, params, I, m_limits.bias, &CTRNN::setBias);
  decodeLayerParams(ctrnn, kLyr_Output, params, I, m_limits.bias, &CTRNN::setBias);
  decodeLayerParams(ctrnn, kLyr_Hidden, params, I, m_limits.tau, &CTRNN::setTimeConstant);
  decodeLayerParams(ctrnn, kLyr_Output, params, I, m_limits.tau, &CTRNN::setTimeConstant);
  
  // Connections
  if(m_inputsAreNeurons)
    decodeLayerConnections(ctrnn, kLyr_Input, kLyr_Hidden, params, I, m_limits.weight, &CTRNN::setWeight);
  
  decodeLayerConnections(ctrnn, kLyr_Hidden, kLyr_Output, params, I, m_limits.weight, &CTRNN::setWeight);
  
  if(m_hiddenLaterallyConnected)
    decodeLayerConnections(ctrnn, kLyr_Hidden, kLyr_Hidden, params, I, m_limits.weight, &CTRNN::setWeight);
  
  if(m_outputsLaterallyConnected)
    decodeLayerConnections(ctrnn, kLyr_Output, kLyr_Output, params, I, m_limits.weight, &CTRNN::setWeight);
  
  
  // Check we decoded correctly!
  int numReqParams = Topology::getNumParameters();
  bool correct = I == numReqParams;
  assert(correct);
  
  return correct;
}

//----------------------------------------------------------------------------------------------------------------------
// Input neurons are placeholder neurons only and are arranged in a layer different from real neurons.
// Each real neuron receives connections from all input neurons.
// Real neurons are are arranged in a bilaterally symmetric layer. I.e. first and last neuron are identical, as well
// as the second and second to last etc...
//----------------------------------------------------------------------------------------------------------------------
bool Topology::encode(CTRNN& ctrnn, double* params) const
{
  int I = 0;
  
  // Input gains, if inputs are not neurons (in which case weights take care of individual weighting)
  if(!m_inputsAreNeurons)
  {
    encodeLayerParams(ctrnn, kLyr_Input, params, I, m_limits.gain, &CTRNN::getGain);
  }
  
  // Neural parameters
  encodeLayerParams(ctrnn, kLyr_Hidden, params, I, m_limits.bias, &CTRNN::getBias);
  encodeLayerParams(ctrnn, kLyr_Output, params, I, m_limits.bias, &CTRNN::getBias);
  encodeLayerParams(ctrnn, kLyr_Hidden, params, I, m_limits.tau, &CTRNN::getTimeConstant);
  encodeLayerParams(ctrnn, kLyr_Output, params, I, m_limits.tau, &CTRNN::getTimeConstant);
  
  // Connections
  if(m_inputsAreNeurons)
    encodeLayerConnections(ctrnn, kLyr_Input, kLyr_Hidden, params, I, m_limits.weight, &CTRNN::getWeight);
  
  encodeLayerConnections(ctrnn, kLyr_Hidden, kLyr_Output, params, I, m_limits.weight, &CTRNN::getWeight);
  
  if(m_hiddenLaterallyConnected)
    encodeLayerConnections(ctrnn, kLyr_Hidden, kLyr_Hidden, params, I, m_limits.weight, &CTRNN::getWeight);
  
  if(m_outputsLaterallyConnected)
    encodeLayerConnections(ctrnn, kLyr_Output, kLyr_Output, params, I, m_limits.weight, &CTRNN::getWeight);
  
  // Check we decoded correctly!
  int numReqParams = Topology::getNumParameters();
  bool correct = I == numReqParams;
  assert(correct);
  
  return correct;
}

//----------------------------------------------------------------------------------------------------------------------
bool Topology::calcConn(int from, int to) const
{
  // Only non-inputs reveive incomming connections
  if (to >= m_size[kLyr_Input])
  {
    if (isHidden(to))
    {
      // Receiver is hidden
      const bool fromIsInput = from < m_size[kLyr_Input];
      if(fromIsInput || (isHidden(from) && m_hiddenLaterallyConnected))
        return true;
    }
    else if (isHidden(from) || (isOutput(from) && m_outputsLaterallyConnected))
    {
      // Receiver is output and origin hidden or other output (if lateral allowed)
      return true;
    }
  }
  
  return false;
}
  
//----------------------------------------------------------------------------------------------------------------------
void Topology::randomiseWeights(CTRNN* net, float min, float max)
{
  for (int i = 0; i < m_N; ++i)
  {
    for (int j = 0; j < m_N; ++j)
    {
      if(m_connections[j][i])
      {
        net->setWeight(j, i, UniformRandom(min, max));
      }
    }
  }
}

//----------------------------------------------------------------------------------------------------------------------
void Topology::toXml(ci::XmlTree& xml) const
{
  ci::XmlTree topXml ("Topology", "");
  topXml.setAttribute("symmetric", m_symmetric);
  topXml.setAttribute("weightCutoff", m_weightCutoff);
  
  ci::XmlTree inputs ("Inputs", toString(m_size[0]));
  inputs.setAttribute("asNeurons", m_inputsAreNeurons);
  topXml.push_back(inputs);
  
  ci::XmlTree hidden ("Hidden", toString(m_size[1]));
  hidden.setAttribute("laterallyConnected", m_hiddenLaterallyConnected);
  topXml.push_back(hidden);
  
  ci::XmlTree outputs ("Outputs", toString(m_size[2]));
  outputs.setAttribute("laterallyConnected", m_outputsLaterallyConnected);
  topXml.push_back(outputs);
  
  m_limits.toXml(topXml); 
  
  xml.push_back(topXml);
}
  
//----------------------------------------------------------------------------------------------------------------------  
void Topology::fromXml(const ci::XmlTree& xml)
{
  assert(xml.getTag() == "Topology");
  
  destroyMatrix();
  
  m_symmetric = xml.getAttributeValue<bool>("symmetric");
  m_weightCutoff = xml.getAttributeValue<double>("weightCutoff", 0.0);
  
  m_size[0] = xml.getChild("Inputs").getValue<int>();
  m_size[1] = xml.getChild("Hidden").getValue<int>();
  m_size[2] = xml.getChild("Outputs").getValue<int>();
  
  m_inputsAreNeurons = xml.getChild("Inputs").getAttributeValue<bool>("asNeurons");
  m_outputsLaterallyConnected = xml.getChild("Outputs").getAttributeValue<bool>("laterallyConnected", true);
  m_hiddenLaterallyConnected = xml.getChild("Hidden").getAttributeValue<bool>("laterallyConnected", true);
  
  m_N = calcSize();
  
  buildMatrix();
  
  if(xml.hasChild("NetLimits"))
  {
    m_limits.fromXml(xml.getChild("NetLimits"));
  }
  
  calcNumParameters();
}

} // namespace