/*
 *  SMCAgentLineDis.h
 *  cinder_dynmx
 *
 *  Created by Thomas Buhrmann on 3/12/12.
 *  Copyright 2012 __MyCompanyName__. All rights reserved.
 *
 */


#ifndef _DMX_SMC_AGENT_GAUSSIAN_
#define _DMX_SMC_AGENT_GAUSSIAN_

#include "SMCAgentEvo.h"
#include <iostream>
#include <fstream>

namespace dmx
{
  
#define GENERALIZATION_WIDTH_MIN 0.01 // 0.01
#define GENERALIZATION_WIDTH_MAX 0.12
#define GENERALIZATION_HEIGHT_MIN 0.01 // 0.01
#define GENERALIZATION_HEIGHT_MAX 0.25
#define GENERALIZATION_POS_MIN -0.4 
#define GENERALIZATION_POS_MAX -0.3

//----------------------------------------------------------------------------------------------------------------------
// A minimal agent scanning the encironment with a distance sensor
//----------------------------------------------------------------------------------------------------------------------
class SMCAgentGaussian : public SMCAgentEvo
{
  enum SMCEvalMode
  {
    kEvalMode_Evolution = 0,
    kEvalMode_Generalization,
    kEvalMode_Trajectories,
    kNumEvalModes
  };
  
public:  
  SMCAgentGaussian() { init(); };
  virtual void nextTrial(int trial);
  virtual float getFitness();
  virtual void endOfEvaluation(float fit);
  virtual void init();
  virtual void reset();
  
protected:  
  virtual void updateFitness();  
  
  int m_evalMode;
  float m_heightAvg;
  float m_heightVar;
  float m_heightVarMax;
  ci::Vec2f m_posVarMax;
  
  // Used in generalization runs
  float m_width;
  float m_height;
  float m_startPos;
  float m_accFitness;
  
  std::ofstream m_file;
};  

//----------------------------------------------------------------------------------------------------------------------    
// Inlines  
//----------------------------------------------------------------------------------------------------------------------  
inline void SMCAgentGaussian::init()
{
  m_heightAvg = 0.13;
  m_heightVar = 0.5;
  m_heightVarMax = 0.5;
  
  m_posVarMax.set(0.0, 0.4);
  
  m_evalMode = kEvalMode_Evolution;
  
  if(SETTINGS->getChild("Config/GA/Eval").getAttributeValue<bool>("Run"))
  {
    m_heightVar = m_heightVarMax;
    const std::vector<Positionable*>& objects = m_agent->getEnvironment().getObjects();    
    ((Gaussian*)objects[0])->setPositionVariance(m_posVarMax);
    
    const std::string& modeName = SETTINGS->getChild("Config/GA/Eval").getAttributeValue<std::string>("mode", "Evolution");
    if(modeName == "Evolution")
    {
      m_evalMode = kEvalMode_Evolution;
    }
    else if(modeName == "Generalization")
    {
      m_evalMode = kEvalMode_Generalization;
    }
    else if(modeName == "Trajectories")
    {
      m_evalMode = kEvalMode_Trajectories;
    }
    
    
    if (m_evalMode == kEvalMode_Generalization)
    {
      m_width = GENERALIZATION_WIDTH_MIN;
      m_height = GENERALIZATION_HEIGHT_MIN;
      m_startPos = GENERALIZATION_POS_MIN; 
      m_accFitness = 0.0;
      ((Gaussian*)objects[1])->setVisibility(false);
      //((Gaussian*)objects[0])->setPosition(ci::Vec2f(0.4, m_startPos));
      ((Gaussian*)objects[0])->setPosition(ci::Vec2f(0.4, 0.0));
      ((Gaussian*)objects[0])->setH(m_height);
      ((Gaussian*)objects[0])->setW(m_width);      
      
      m_agent->setPositionWraps(false);
      m_agent->setPosition(ci::Vec2f(0.0, m_startPos));

      const std::string fnm = DATA_DIR + "Generalization.txt";
      m_file.open (fnm.c_str()); 
    }    
    else if(m_evalMode == kEvalMode_Trajectories)
    {
      m_width = 0.03;
      m_height = 0.13;
      m_startPos = GENERALIZATION_POS_MIN; 
      m_accFitness = 0.0;
      ((Gaussian*)objects[1])->setVisibility(false);
      ((Gaussian*)objects[0])->setH(m_height);
      ((Gaussian*)objects[0])->setW(m_width);      
    }
    else if(m_evalMode == kEvalMode_Evolution)
    {
      nextTrial(0);
    }
  }
  else if(m_evalMode == kEvalMode_Evolution)
  {
    nextTrial(0);
  }
}
  
//----------------------------------------------------------------------------------------------------------------------  
inline void SMCAgentGaussian::reset()
{
  SMCAgentEvo::reset();
  
  if(m_evalMode == kEvalMode_Trajectories || m_evalMode == kEvalMode_Generalization)
  {
    m_agent->setPosition(ci::Vec2f(0.0, m_startPos));  
  }
}

//----------------------------------------------------------------------------------------------------------------------    
inline void SMCAgentGaussian::nextTrial(int trial)
{
  const std::vector<Positionable*>& objects = m_agent->getEnvironment().getObjects();

  if(m_evalMode == kEvalMode_Evolution)
  {
    // Random positions and heights
    float randomHeight = m_heightAvg + (m_heightVar * m_heightAvg * UniformRandom(-1.0, 1.0));  
    ((Gaussian*)objects[0])->setH(randomHeight);    
    ((Gaussian*)objects[1])->setH(randomHeight);  

    // randomly position first object on one side
    int randomSide = UniformRandom(0,1) > 0.5 ? 1 : -1;
    float posVar = ((Gaussian*)objects[0])->getPositionVar().y;
    const float randPos = randomSide * UniformRandom(0.1, posVar);
    ((Gaussian*)objects[0])->setPosition(ci::Vec2f(0.4, randPos));
    
    // then make sure other object is on other side
    const float randPos2 = -randomSide * UniformRandom(0.1, posVar);  
    ((Gaussian*)objects[1])->setPosition(ci::Vec2f(0.4, randPos2));
  }
  else if (m_evalMode == kEvalMode_Generalization)
  {
    // Fixed order of width and height, 100.000 trials 
    m_startPos += (GENERALIZATION_POS_MAX - GENERALIZATION_POS_MIN) / 10.0;
    
    if(trial % 10 == 0)
    {
      // Average fitness over starting positions
      float avgFit = m_accFitness / 10.0;
      m_accFitness = 0.0;
      //std::cout << "W " << m_width << " H " << m_height << ": Avg fitness = " << avgFit << std::endl;
      m_file << avgFit << "\t";
      
      m_startPos = GENERALIZATION_POS_MIN;
      m_height += (GENERALIZATION_HEIGHT_MAX - GENERALIZATION_HEIGHT_MIN) / 100.0;
      
      if(trial % (10*100) == 0)
      {
        m_height = GENERALIZATION_HEIGHT_MIN;
        m_width += (GENERALIZATION_WIDTH_MAX - GENERALIZATION_WIDTH_MIN) / 100.0;
        m_file << std::endl;
      }
    }
    
    // Apply values
    //((Gaussian*)objects[0])->setPosition(ci::Vec2f(0.4, m_startPos));
    ((Gaussian*)objects[0])->setH(m_height);
    ((Gaussian*)objects[0])->setW(m_width);
  }
  else if (m_evalMode == kEvalMode_Trajectories)
  {
    // One set of catching, one set of avoiding trials, starting at different initial conditions, 20 trials total.
    m_startPos += (GENERALIZATION_POS_MAX - GENERALIZATION_POS_MIN) / 10.0;
    
    if(trial % 10 == 0)
    {
      // Average fitness over starting positions
      float avgFit = m_accFitness / 10.0;
      m_accFitness = 0.0;
      std::cout << "W " << m_width << " H " << m_height << ": Avg fitness = " << avgFit << std::endl;
      
      m_startPos = GENERALIZATION_POS_MIN;
      m_width = 0.08;
    }
    
    // Apply values
    m_agent->setPosition(ci::Vec2f(0.0, m_startPos));
    ((Gaussian*)objects[0])->setW(m_width);
  }
  

}
  
//----------------------------------------------------------------------------------------------------------------------  
inline void SMCAgentGaussian::endOfEvaluation(float fitness)
{  
}  
  
//----------------------------------------------------------------------------------------------------------------------  
inline float SMCAgentGaussian::getFitness()
{
  float fit =  SMCAgentEvo::getFitness();
  
  const bool minimizeConnectivity = false;
  if(minimizeConnectivity)
  {
    float weightSum =   m_agent->getCTRNN().getWeightSum();
    float sumMax = sqr((float)m_agent->getTopology().getSize()) * 10.0; // max weight defined in NetLimits
    float weightFit = 1.0 - (weightSum / sumMax);
    fit *= weightFit;
  }
  
  if(m_evalMode == kEvalMode_Generalization || m_evalMode == kEvalMode_Trajectories)
  {
    m_accFitness += fit;
  }
  
  return fit;
}

// Go left if object 0 is visible, otherwise right
//----------------------------------------------------------------------------------------------------------------------  
inline void SMCAgentGaussian::updateFitness()
{
  // Measure distance from desired position
  if(m_agent->getTime() >= m_fitnessEvalDelay)
  { 
    const std::vector<Positionable*>& objects = m_agent->getEnvironment().getObjects(); 
    float desPos = objects[0]->getPosition().y;    
    float actPos = clamp(m_agent->getPosition().y, -1.0f, 1.0f);
    float diff = sqr(actPos - desPos);    
    m_fitness += diff;
  } 
}  

} // namespace


#endif