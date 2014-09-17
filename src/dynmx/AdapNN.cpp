//
//  AdapNN.cpp
//  dynmx
//
//  Created by Thomas Buhrmann on 02/09/14.
//
//

#include <stdio.h>

#include "AdapNN.h"

namespace dmx
{
  
// By default non-adaptive
#define DEFAULT_NDECAY 0.0
#define DEFAULT_RDECAY 0.0
#define DEFAULT_NOISEVAR 0.0
#define DEFAULT_WDECAY 0.0
#define DEFAULT_LRATE 0.0
  
//----------------------------------------------------------------------------------------------------------------------
AdapNN::AdapNN(Topology* top) :
  m_topology(top),
  CTRNN(top->getSize())
{
  // Dynamically create arrays
  m_meanOut = new double[size];
  
  // Initialize
  std::fill(m_meanOut, m_meanOut + size, 0.0);
  
  // Reset states
  m_R = 0;
  m_meanR = 0;
  
  // Initialize parameters to sensible defaults
  // Network-specific:
  m_noiseUniform = true;
  m_useAntiHebbSwitch = false;
  m_rdecay = DEFAULT_RDECAY;
  m_noiseVar = DEFAULT_NOISEVAR;
  m_preMeanSubFac = 1.0f;
  
  // Neuron-specific:
  m_ndecay = new float[size];
  setNeuralMeanFilters(DEFAULT_NDECAY);
  
  // Synapse-specific
  m_wdecay = createArray2d<float>(size, size, DEFAULT_WDECAY);
  m_lrate = createArray2d<float>(size, size, DEFAULT_LRATE);
  
}
  
//----------------------------------------------------------------------------------------------------------------------
AdapNN::~AdapNN()
{
  // Destroy arrays
  delete [] m_meanOut;
}
  
//----------------------------------------------------------------------------------------------------------------------
void AdapNN::setReward(float r)
{
  m_R = r;
  m_meanR = (m_rdecay * m_meanR) + (1 - m_rdecay) * m_R;
}
  
//----------------------------------------------------------------------------------------------------------------------
void AdapNN::updateDynamic(double dt)
{
  // Update neural states and output
  // Needs noise! Relies on externalinputs being reset afterwards!!
  for (int i = m_topology->getNumInputs(); i < size; ++i)
    externalinputs[i] += m_noiseUniform ? UniformRandom(- m_noiseVar, m_noiseVar) : GaussianRandom(0, m_noiseVar);
    
  CTRNN::updateDynamic(dt);
  
  // Update eligibility traces (mean neural firing rates)
  for (int i = 0; i < size; i++)
  {
    m_meanOut[i] = (m_ndecay[i] * m_meanOut[i]) + (1 - m_ndecay[i]) * (outputs[i]);
  }
  
  // Update weights: wji is the connection from j to i, i.e. j is pre-synaptic neuron
  if (m_lrate != 0)
  {
    for (int i = m_topology->getNumInputs(); i < size; i++)
    {
      for (int j = 0; j < size; j++)
      {
        if(adaptive(j, i))
        {
          float reward = m_R - m_meanR;
          float eji = (outputs[j] - m_preMeanSubFac * m_meanOut[j]) * (outputs[i] - m_meanOut[i]);
          if (m_useAntiHebbSwitch)
            eji *= sign(reward);
          
          float dwji = m_lrate[j][i] * reward * eji;
          dwji -= m_wdecay[j][i] * weights[j][i];
          weights[j][i] += dwji;
        }
      }
    } // end weights loop
  }
  
  // Reset external inputs so they don't grow infinitely due to noise
  for (int i = 0; i < size; ++i)
    externalinputs[i] = 0;
}
  
//----------------------------------------------------------------------------------------------------------------------
void AdapNN::toXml(ci::XmlTree& xml)
{
  // Modify the xml that's being appended by base class
  CTRNN::toXml(xml);
  
  ci::XmlTree& nn = xml.getChild("CTRNN");
  
  nn.setAttribute("adaptive", 1);
  nn.setAttribute("noiseUniform", m_noiseUniform);
  nn.setAttribute("noiseVar", m_noiseVar);
  nn.setAttribute("hebbSwitch", m_useAntiHebbSwitch);
  nn.setAttribute("preMeanFac", m_preMeanSubFac);
  nn.setAttribute("rewardDecay", m_rdecay);
  
  ci::XmlTree::Iter neuron = nn.begin("Neuron");
  for (; neuron != nn.end(); ++neuron)
  {
    ci::XmlTree& n = *neuron;
    int i = n.getAttributeValue<int>("Index");
    n.setAttribute("meanDecay", m_ndecay[i]);
    
    ci::XmlTree::Iter weight = n.begin("Weight");
    for (; weight != neuron->end(); ++weight)
    {
      ci::XmlTree& w = *weight;
      int from = w.getAttributeValue<int>("From");
      w.setAttribute("learnRate", m_lrate[from][i]);
      w.setAttribute("decay", m_wdecay[from][i]);
    }
  }
  
}

//----------------------------------------------------------------------------------------------------------------------
void AdapNN::fromXml(const ci::XmlTree& xml)
{
  CTRNN::fromXml(xml);
  
  if(xml.getTag() == "CTRNN")
  {
    m_noiseUniform = xml.getAttributeValue<bool>("noiseUniform", true);
    m_noiseVar = xml.getAttributeValue<float>("noiseVar", 0);
    m_useAntiHebbSwitch = xml.getAttributeValue<bool>("hebbSwitch", false);
    m_preMeanSubFac = xml.getAttributeValue<float>("preMeanFac", 1);
    m_rdecay = xml.getAttributeValue<float>("rewardDecay", 0);
    
    ci::XmlTree::ConstIter neuron = xml.begin("Neuron");
    for (; neuron != xml.end(); ++neuron)
    {
      const ci::XmlTree& n = *neuron;
      int i = n.getAttributeValue<int>("Index");
      m_ndecay[i] = n.getAttributeValue<double>("meanDecay");
      
      ci::XmlTree::ConstIter weight = n.begin("Weight");
      for (; weight != n.end(); ++weight)
      {
        int from = weight->getAttributeValue<int>("From");
        m_lrate[from][i] = weight->getAttributeValue<float>("learnRate", 0.0);
        m_wdecay[from][i] = weight->getAttributeValue<float>("decay", 0.0);
      }
    }    
  }
}
  
};
