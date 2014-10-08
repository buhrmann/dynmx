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

std::string AdapNN::s_synScalingNames [kSynSc_Num] = {"Oja", "Turr", "HomeoReg", "None"};
  
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
  m_reward = 0;
  
  // Initialize parameters to sensible defaults
  // Network-specific:
  m_noiseUniform = true;
  m_useAntiHebbSwitch = false;
  m_rdecay = DEFAULT_RDECAY;
  m_noiseVar = DEFAULT_NOISEVAR;
  m_preMeanSubFac = 0.0f;
  
  m_scaling = kSynSc_HomeoReg;
  
  // Neuron-specific:
  m_ndecay = new float[size];
  setNeuralMeanFilters(DEFAULT_NDECAY);
  
  // Synapse-specific
  m_wdecay = createArray2d<double>(size, size, DEFAULT_WDECAY);
  m_lrate = createArray2d<double>(size, size, DEFAULT_LRATE);
  m_wdt = createArray2d<double>(size, size, 0.0);
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
  m_reward = m_R - m_meanR;
  
  m_reward = std::max(0.0f, m_reward);
}
  
//----------------------------------------------------------------------------------------------------------------------
void AdapNN::updateDynamic(double dt)
{
  // Update neural states and output with noise
  for (int i = m_topology->getNumInputs(); i < size; ++i)
  {
    float noiseVar = m_noiseVar/2.0 * outputs[i]/2.0;
    externalinputs[i] = m_noiseUniform ? UniformRandom(- noiseVar, noiseVar) : GaussianRandom(0, noiseVar);
  }
  
  CTRNN::updateDynamic(dt);
  
  // Update eligibility traces (mean neural firing rates)
  for (int i = 0; i < size; i++)
  {
    m_meanOut[i] = (m_ndecay[i] * m_meanOut[i]) + (1 - m_ndecay[i]) * (outputs[i]);
  }
  
  // Update weights: wji is the connection from j to i, i.e. j is pre-synaptic neuron
  for (int i = m_topology->getNumInputs(); i < size; i++)
  {
    for (int j = 0; j < size; j++)
    {
      if(adaptive(j, i))
      {
        float eji = (outputs[j] - m_preMeanSubFac * m_meanOut[j]) * (outputs[i] - m_meanOut[i]);
        //float eji = outputs[j] * outputs[i];
        
        if (m_useAntiHebbSwitch)
          eji *= sign(m_reward);
        
        m_wdt[j][i] = m_lrate[j][i] * m_reward * eji;
        
        // Scaling
        float scalar = 0;
        if (kSynSc_Oja == m_scaling)
        {
          scalar = -weights[j][i] * sqr(m_meanOut[i]);
        }
        else if (kSynSc_Turr == m_scaling)
        {
          scalar = (0.5 - m_meanOut[i]) * sign(weights[j][i]) * pow(weights[j][i], 1);
        }
        else if (kSynSc_HomeoReg == m_scaling)
        {
          if(m_meanOut[i] < 0.25)
            scalar = 16 * sqr(m_meanOut[i] - 0.25) * sqr(weights[j][i]);
          else if (m_meanOut[i] > 0.75)
            scalar = -16 * sqr(m_meanOut[i] - 0.75) * sqr(weights[j][i]);
          else
            scalar = 0;
        }

        m_wdt[j][i] += m_lrate[j][i] * m_wdecay[j][i] * scalar;
        
        // Limit
        float wmax = 10;
        float l = (wmax - fabs(weights[j][i])) / wmax;
        m_wdt[j][i] *= l;
        
        weights[j][i] += m_wdt[j][i];
      }
    }
  } // end weights loop
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
