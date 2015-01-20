//
//  AdapNN.cpp
//  dynmx
//
//  Created by Thomas Buhrmann on 02/09/14.
//
//

#include <stdio.h>
#include <cmath>

#include "AdapNN.h"

namespace dmx
{
  
// By default non-adaptive
#define DEFAULT_NDECAY 0.0
#define DEFAULT_RDECAY 0.0
#define DEFAULT_NOISEVAR 0.0
#define DEFAULT_WDECAY 0.0
#define DEFAULT_LRATE 0.0
#define DEFAULT_SYNSCALE 0.0
  
std::string AdapNN::s_synScalingNames [kSynSc_Num] = {"Oja", "Turr", "HomeoReg", "HomeoPre", "None"};
  
//----------------------------------------------------------------------------------------------------------------------
AdapNN::AdapNN(Topology* top) :
  m_topology(top),
  CTRNN(top->getSize())
{
  // Dynamically create arrays
  m_meanAct = new double[size];
  std::fill(m_meanAct, m_meanAct + size, 0.0);
  
  // Reset states
  m_R = 0;
  m_meanR = 0;
  m_reward = 0;
  m_useReward = false;
  
  // Initialize parameters to sensible defaults
  // Network-specific:
  m_noiseUniform = true;
  m_useAntiHebbSwitch = false;
  m_rdecay = DEFAULT_RDECAY;
  m_noiseVar = DEFAULT_NOISEVAR;
  m_preMeanSubFac = 0.0f;
  
  m_wmax = top->getLimits().weight.max;
  
  m_scaling = kSynSc_HomeoPre;
  
  // Neuron-specific
  m_ndecay = new double[size];
  setNeuralMeanFilters(DEFAULT_NDECAY);
  
  // Synapse-specific
  m_wdecay = createArray2d<double>(size, size, DEFAULT_WDECAY);
  m_lrate = createArray2d<double>(size, size, DEFAULT_LRATE);
  m_synscale = createArray2d<double>(size, size, DEFAULT_SYNSCALE);
  m_preFac = createArray2d<double>(size, size, 0);
  m_postFac = createArray2d<double>(size, size, 0);
  m_dw = createArray2d<double>(size, size, 0.0);
  
  
  // Weight perturbation
  m_wold = createArray2d<double>(size, size, 0.0);
  
  // Node perturbation
  m_bold = new double[size];
  m_db = new double[size];
  
  //m_mode = "NodePerturbationElig";
  m_mode = "HebbStab";

  reset();
}
  
//----------------------------------------------------------------------------------------------------------------------
AdapNN::~AdapNN()
{
  // Destroy arrays
  delete [] m_meanAct;
}

//----------------------------------------------------------------------------------------------------------------------
void AdapNN::reset()
{
  m_time = 0;
  m_R = m_RM = m_R0 = 0;
  m_epoch = 0;
  
  // Reset mean activities
  if(m_mode == "RHebb")
    std::fill(m_meanAct, m_meanAct + size, 0.0); // Operates on activity (state)
  else
    std::fill(m_meanAct, m_meanAct + size, 0.5); // Operates on outputs (firing rate)
  
  // Allow topology to randomise non-evolved parameters
  m_topology->reset(this);
  
  if (m_mode == "NodePerturbation" || m_mode == "NodePerturbationCont")
  {
    for (int i = m_topology->getNumInputs(); i < size; i++){
      biases[i] = 0;
      externalinputs[i] = 0; // TODO: Randomize!
      m_bold[i] = externalinputs[i];
    }
    newNodePerturbation();
  }
  else if (m_mode == "NodePerturbationElig"){
    for (int i = 0; i < size; i++)
      biases[i] = 0;
  }
  else if (m_mode == "WeightPerturbation"){
    newWeightPerturbation(0.002);
  }
  else if (m_mode == "WeightPerturbationElig"){
    for (int i = 0; i < size; i++)
      for (int j = 0; j < size; j++)
        m_wold[j][i] = weights[j][i];
  }
}
  
//----------------------------------------------------------------------------------------------------------------------
void AdapNN::setReward(float r)
{
  m_R = r;
  m_meanR = (m_rdecay * m_meanR) + (1 - m_rdecay) * m_R;
  m_reward = m_R - m_meanR;
  
  //m_reward = std::max(0.0f, m_reward);
}

//----------------------------------------------------------------------------------------------------------------------
void AdapNN::newNodePerturbation()
{
  const float var = 0.02f;
  const int numIn = m_topology->getNumInputs();
  
  // Create new perturbation
  for (int i = numIn; i < size; i++)
      externalinputs[i] = GaussianRandom(0, var);
}
  
  
//----------------------------------------------------------------------------------------------------------------------
void AdapNN::newWeightPerturbation(float var)
{
  // Cache original weights
  for (int i = 0; i < size; i++)
    for (int j = 0; j < size; j++)
      m_wold[j][i] = weights[j][i];
  
  // Create new perturbation
  for (int i = 0; i < size; i++)
    for (int j = 0; j < size; j++)
      m_dw[j][i] = GaussianRandom(0, var);
  
  // Make perturbation current
  for (int i = 0; i < size; i++)
    for (int j = 0; j < size; j++)
      if(m_topology->connected(j, i)){
        weights[j][i] += m_dw[j][i];
        weights[j][i] = m_topology->getLimits().weight.clamp(weights[j][i]);
      }
}

//----------------------------------------------------------------------------------------------------------------------
void AdapNN::updateDynamic(double dt)
{
  if (m_mode == "Hebb")
    updateHebb(dt);
  else if (m_mode == "RHebb")
    updateRHebb(dt);
  else if (m_mode == "HebbStab")
    updateHebbStab(dt);
  else if(m_mode == "WeightPerturbation")
    updateWeightPerturb(dt);
  else if(m_mode == "WeightPerturbationElig")
    updateWeightPerturbElig(dt);
  else if (m_mode == "NodePerturbation")
    updateNodePerturb(dt);
  else if (m_mode == "NodePerturbationElig")
    updateNodePerturbElig(dt);
  else if (m_mode == "NodePerturbationCont")
    updateNodePerturbCont(dt);
  else
    updateHebb(dt);
}

  
//----------------------------------------------------------------------------------------------------------------------
float AdapNN::synapticScaling(int pre, int post)
{
  float scalar = 0;
  if (kSynSc_Oja == m_scaling)
  {
    scalar = -weights[pre][post] * sqr(m_meanAct[post]);
  }
  else if (kSynSc_Turr == m_scaling)
  {
    scalar = (0.5 - m_meanAct[post]) * sign(weights[pre][post]) * pow(weights[pre][post], 1);
  }
  else if (kSynSc_HomeoReg == m_scaling)
  {
    if(m_meanAct[post] < 0.25)
      scalar = 16 * sqr(m_meanAct[post] - 0.25) * sqr(weights[pre][post]);
    else if (m_meanAct[post] > 0.75)
      scalar = -16 * sqr(m_meanAct[post] - 0.75) * sqr(weights[pre][post]);
    else
      scalar = 0;
  }
  else if (kSynSc_HomeoPre == m_scaling)
  {
    if(m_meanAct[post] < 0.25)
      scalar = 16 * sqr(m_meanAct[post] - 0.25) * sqr(weights[pre][post]) * m_meanAct[pre];
    else if (m_meanAct[post] > 0.75)
      scalar = -16 * sqr(m_meanAct[post] - 0.75) * sqr(weights[pre][post]) * m_meanAct[pre];
    else
      scalar = 0;
  }
  
  scalar *= m_synscale[pre][post];
  return scalar;
}

  
//----------------------------------------------------------------------------------------------------------------------
void AdapNN::hardLimitDw(int pre, int post)
{
  float l = (m_wmax - fabs(weights[pre][post])) / m_wmax;
  m_dw[pre][post] *= l;
  
  weights[pre][post] += m_dw[pre][post];
  weights[pre][post] = clamp(weights[pre][post], -m_wmax, m_wmax);
}
  

//----------------------------------------------------------------------------------------------------------------------
void AdapNN::validate(int i, int j)
{
  assert(isNum(m_dw[i][j]));
  assert(isNum(weights[i][j]));
}

//----------------------------------------------------------------------------------------------------------------------
void AdapNN::updateHebbStab(double dt)
{
  // Noisy update
  const int numInputs = m_topology->getNumInputs();
  for (int i = numInputs; i < size; i++)
    externalinputs[i] = m_noiseUniform ? UniformRandom(-m_noiseVar, m_noiseVar) : GaussianRandom(0, m_noiseVar);
  
  CTRNN::updateDynamic(dt);
  
  // Update mean neural activity
  for (int i = 0; i < size; i++)
    m_meanAct[i] = (m_ndecay[i] * m_meanAct[i]) + (1 - m_ndecay[i]) * (outputs[i]);
  
  // Learn
  for (int post = 0; post < size; post++)
    for (int pre = 0; pre < size; pre++)
      if(m_topology->connected(pre, post))
      {
        // Reward stability
        float eji = m_lrate[pre][post] * (outputs[pre] - m_preMeanSubFac*m_meanAct[pre]) * (outputs[post] - m_meanAct[post]);
        eji += m_lrate[pre][post] * ((m_preFac[pre][post] * outputs[pre]) + (m_postFac[pre][post] * outputs[post]));
        //eji *= fabs(weights[pre][post]);
        
        if(m_useReward)
          eji *= sign(m_reward);
        
        m_dw[pre][post] = eji;
        
        float synscale = synapticScaling(pre, post);
        m_dw[pre][post] += synscale;
        
        // If in homeostatic limits
        if(synscale == 0)
          m_dw[pre][post] -= m_wdecay[pre][post] * weights[pre][post];
        
        hardLimitDw(pre, post);
        validate(pre, post);
      }
}

// Operates on means of state y, rather than output, to approximate the noise !
//----------------------------------------------------------------------------------------------------------------------
void AdapNN::updateRHebb(double dt)
{
  // Update neural states and output with noise
  const int numInputs = m_topology->getNumInputs();
  for (int i = numInputs; i < size; ++i)
    externalinputs[i] = m_noiseUniform ? UniformRandom(- m_noiseVar, m_noiseVar) : GaussianRandom(0, m_noiseVar);
  
  CTRNN::updateDynamic(dt);
  
  // Update mean neural activity
  for (int i = 0; i < size; i++)
    m_meanAct[i] = (m_ndecay[i] * m_meanAct[i]) + (1 - m_ndecay[i]) * (states[i]);
  
  // Learn
  for (int post = numInputs; post < size; post++)
    for (int pre = 0; pre < size; pre++)
      if(adaptive(pre, post))
      {
        float eji = states[pre] * (states[post] - m_meanAct[post]);
        m_dw[pre][post] = m_lrate[pre][post] * sign(m_reward) * eji;
        
        float synscale = synapticScaling(pre, post);
        m_dw[pre][post] += synscale;
        
        hardLimitDw(pre, post);
        validate(pre, post);
      }
  
  const float t_epoch = 100.0f;
  m_time += dt;
  if(m_time >= t_epoch)
  {
    std::cout << "Reward " << m_epoch << ": " << m_meanR << " (inc: " << m_reward << ")" << std::endl;
    m_time = 0;
    m_epoch++;
  }
}


//----------------------------------------------------------------------------------------------------------------------
void AdapNN::updateHebb(double dt)
{
  // Update neural states and output with noise
  const int numInputs = m_topology->getNumInputs();
  for (int i = numInputs; i < size; ++i)
    externalinputs[i] = m_noiseUniform ? UniformRandom(- m_noiseVar, m_noiseVar) : GaussianRandom(0, m_noiseVar);
  
  CTRNN::updateDynamic(dt);
  
  // Update mean neural activity
  for (int i = 0; i < size; i++)
    m_meanAct[i] = (m_ndecay[i] * m_meanAct[i]) + (1 - m_ndecay[i]) * (outputs[i]);
  
  // Learn
  for (int post = numInputs; post < size; post++)
    for (int pre = 0; pre < size; pre++)
      if(adaptive(pre, post))
      {
        // Hebbian component
        float eji = (outputs[pre] - m_preMeanSubFac * m_meanAct[pre]) * (outputs[post] - m_meanAct[post]);
        m_dw[pre][post] = m_lrate[pre][post] * m_reward * eji;
        
        float synscale = synapticScaling(pre, post);
        m_dw[pre][post] += synscale;
        
        hardLimitDw(pre, post);
        validate(pre, post);
      }
}
  
//----------------------------------------------------------------------------------------------------------------------
void AdapNN::updateNodePerturbCont(double dt)
{
  const float t_epoch = 100.0f;
  const float var = 0.002f;

  // Run noisy network and integrate noise
  int numPerts = 1;
  int mod = 10 * fmod(m_time, t_epoch / numPerts);
  if(mod == 0){
    //std::cout << "New perturbation at time " << m_time << std::endl;
    for (int i = m_topology->getNumInputs(); i < size; i++){
      externalinputs[i] = GaussianRandom(0, var);
      m_db[i] += externalinputs[i] / numPerts;
    }
  }
  
  CTRNN::updateDynamic(dt);
  m_RM += dt * m_R;
  
  // Update mean activities
  for (int i = 0; i < size; i++)
    m_meanAct[i] += dt * states[i] / t_epoch;
  
  m_time += dt;
  if(m_time >= t_epoch)
  {
    m_RM /= t_epoch;
    float dR = m_RM - m_R0; // increase in reward
    std::cout << "Epoch reward " << m_epoch << ": " << m_RM << " (inc: " << dR << ")"  << " | " << m_lrate[0][0] << std::endl;
    
    dR = (dR > 0) ? 1 : (dR < 0) ? - 1 : 0;
    
    // Learn
    for (int i = 0; i < size; i++)
      for (int j = 0; j < size; j++)
        if(m_topology->connected(j, i))
          weights[j][i] += 1 * m_lrate[j][i] * dR * m_db[i] * m_meanAct[j];
    
    if(m_RM > 0.5 && m_R0 > 0.5)
      setLearningRates(0.999 * m_lrate[0][0]);
    
    m_R0 = m_RM;
    m_RM = 0;
    std::fill(m_meanAct, m_meanAct+size, 0);
    std::fill(m_db, m_db + size, 0);
    m_time = 0;
    m_epoch++;
  }
}
  
  
//----------------------------------------------------------------------------------------------------------------------
void AdapNN::updateNodePerturb(double dt)
{
  const float t_epoch = 100.0f;
  const float var = 0.002f;
  
  CTRNN::updateDynamic(dt);
  m_RM += dt * m_R;
  
  // Update mean activities
  for (int i = 0; i < size; i++)
  {
    //m_meanAct[i] = (ndecay * m_meanAct[i]) + (1 - ndecay) * (states[i]);
    m_meanAct[i] += dt * states[i] / t_epoch;
  }
  
  m_time += dt;
  if(m_time >= t_epoch)
  {
    m_RM /= t_epoch;
    float dR = m_RM - m_R0; // increase in reward
    std::cout << "Epoch reward " << m_epoch << ": " << m_RM << " (inc: " << dR << ")"  << " | " << m_lrate[0][0] << std::endl;
    
    dR = (dR > 0) ? 1 : (dR < 0) ? - 1 : 0;
    
    // Learn
    for (int i = 0; i < size; i++)
      for (int j = 0; j < size; j++)
        if(m_topology->connected(j, i))
          weights[j][i] += 1 * m_lrate[j][i] * dR * externalinputs[i] * (m_meanAct[j]);
    
    // Create new perturbation
    for (int i = m_topology->getNumInputs(); i < size; i++)
      externalinputs[i] = GaussianRandom(0, var);
    
    if(m_RM > 0.5 && m_R0 > 0.5)
      setLearningRates(0.999 * m_lrate[0][0]);
    
    m_R0 = m_RM;
    std::fill(m_meanAct, m_meanAct+size, 0);
    m_RM = 0;
    m_time = 0;
    m_epoch++;
  }
}

  
//----------------------------------------------------------------------------------------------------------------------
void AdapNN::updateNodePerturbElig(double dt)
{
  const float t_epoch = 100.0f;
  const float var = 0.01f;
  
  // Add noise to the biases and cache noise terms for integration over epoch
  const int numInputs = m_topology->getNumInputs();
  for (int i = numInputs; i < size; i++){
    m_db[i] = GaussianRandom(0, var);
    externalinputs[i] = m_db[i];
  }
  
  CTRNN::updateDynamic(dt);
  m_RM += dt * m_R;
  
  // Update eligibility traces
  for (int i = 0; i < size; i++)
    for (int j = 0; j < size; j++)
      if (m_topology->connected(j, i)) {
        m_dw[j][i] += (m_db[i] * states[j]);
      }
  
  m_time += dt;
  if(m_time >= t_epoch)
  {
    m_RM /= t_epoch;
    
    float dR = m_RM - m_R0; // increase in reward
    
    //if(m_RM > 0.7)
    //  m_nu *= 0.99;
    
    std::cout << "Epoch" << m_epoch << ": " << m_RM << " (inc: " << dR << ")" << " | " << m_lrate[0][0] << std::endl;
    if (false){
      printArray(biases, size);
      printArray2d(m_dw, size, size);
    }
    
    //dR = (dR > 0) ? 1 : (dR < 0) ? -1 : 0;
    
    // Learn
    for (int i = 0; i < size; i++)
      for (int j = 0; j < size; j++)
        if (m_topology->connected(j, i)) {
          weights[j][i] += sign(weights[j][i]) * m_lrate[j][i] * 1 * dR * m_dw[j][i];
        }
    
    // Reset eligibility traces
    fillArray2d(m_dw, size, size, 0.0);
    
    m_R0 = m_RM; // reset initial reward
    m_RM = 0;
    m_time = 0;
    m_epoch++;
  }
}
  
  
//----------------------------------------------------------------------------------------------------------------------
void AdapNN::updateWeightPerturbElig(double dt)
{
  const float t_epoch = 100.0f;
  const float var = 0.002f;
  
  // Add noise to the biases and cache noise terms for integration over epoch
  for (int i = 0; i < size; i++)
    for (int j = 0; j < size; j++)
      if (m_topology->connected(j, i))
      {
        double noise = GaussianRandom(0, var);
        weights[j][i] += noise;
        m_dw[j][i] += noise;
      }

  
  CTRNN::updateDynamic(dt);
  m_RM += dt * m_R;
  
  m_time += dt;
  if(m_time >= t_epoch)
  {
    m_RM /= t_epoch;
    
    float dR = m_RM - m_R0; // increase in reward
    if(m_RM > 0.7 && m_R0 > 0.7)
      setLearningRates(0.99 * m_lrate[0][0]);
    
    std::cout << "Epoch " << m_epoch << ": " << m_RM << " (" << dR << " )" << " | " << m_lrate[0][0] << std::endl;
    //printArray2d(m_wdt, size, size);
    
    dR = (dR > 0) ? 1 : (dR < 0) ? - 1 : 0;
    
    // Learn
    for (int i = 0; i < size; i++)
      for (int j = 0; j < size; j++)
        if (m_topology->connected(j, i))
        {
          weights[j][i] = m_wold[j][i] + (m_lrate[j][i] * dR * m_dw[j][i]);
        }
    
    // Reset eligibility traces
    fillArray2d(m_dw, size, size, 0.0);
    
    // Store initial weights
    for (int i = 0; i < size; i++)
      for (int j = 0; j < size; j++)
        m_wold[j][i] = weights[j][i];
    
    m_R0 = m_RM; // reset initial reward
    m_RM = 0;
    m_time = 0;
    m_epoch++;
  }
}
  
//----------------------------------------------------------------------------------------------------------------------
void AdapNN::updateWeightPerturb(double dt)
{
  const float var = 0.5f;
  const float t_epoch = 100.0f; // sec.
  const float ndecay = 0.99f;
  const float synsc = 1.f;
  
  CTRNN::updateDynamic(dt);
  m_RM += dt * m_R;
  
  for (int i = 0; i < size; i++)
    m_meanAct[i] = (ndecay * m_meanAct[i]) + (1 - ndecay) * (outputs[i]);
    
  m_time += dt;
  if(m_time >= t_epoch)
  {
    m_RM /= t_epoch;
    float dR = m_RM - m_R0; // increase in reward
    if(m_RM > 0.25 && m_R0 > 0.25)
      setLearningRates(0.999 * m_lrate[0][0]);
    
    std::cout << "Epoch reward " << m_epoch << ": " << m_RM << " (inc: " << dR << ")" << " | " << m_lrate[0][0] << std::endl;
    dR = (dR > 0) ? 1 : (dR < 0) ? - 1 : 0;
    
    // Learn
    double wmax = 10;
    for (int i = m_topology->getNumInputs(); i < size; i++)
      for (int j = 0; j < size; j++)
        if(m_topology->connected(j, i))
        {
          float dw = m_lrate[j][i] * dR * m_dw[j][i];
          
          // Synaptic scaling
          const bool scale = true;
          if(scale){
            float scalar = 0;
            if(m_meanAct[i] < 0.25)
              scalar = 16 * sqr(m_meanAct[i] - 0.25) * fabs(weights[j][i]);
            else if (m_meanAct[i] > 0.75)
              scalar = -16 * sqr(m_meanAct[i] - 0.75) * fabs(weights[j][i]);
            else
              scalar = 0;
            
            dw += m_lrate[j][i] * synsc * scalar;
            assert(dw < 10);
            assert(dw == dw);
            assert(std::isfinite(dw));
            
            // Limit
            float l = (wmax - fabs(weights[j][i])) / wmax;
            assert(l >= 0 && l <= 1);
            dw *= l;
            assert(dw == dw);
            assert(std::isfinite(dw));
          }
          
          weights[j][i] = m_wold[j][i] + dw;
          weights[j][i] = clamp(weights[j][i], -wmax, wmax);
        }
    
    // Try new solution
    newWeightPerturbation(var);
    
    m_R0 = m_RM; // reset initial reward
    m_RM = 0;
    m_time = 0;
    m_epoch++;
  }
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
      w.setAttribute("synScale", m_synscale[from][i]);
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
        m_synscale[from][i] = weight->getAttributeValue<float>("synScale", 0.0);
      }
    }    
  }
}
  
};
