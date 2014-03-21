/*
 *  SMCAgentLineDis.h
 *  cinder_dynmx
 *
 *  Created by Thomas Buhrmann on 3/12/12.
 *  Copyright 2012 __MyCompanyName__. All rights reserved.
 *
 */


#ifndef _DMX_SMC_AGENT_LINE_DIS_
#define _DMX_SMC_AGENT_LINE_DIS_

#include "SMCAgentEvo.h"

namespace dmx
{
  
//----------------------------------------------------------------------------------------------------------------------
// A minimal agent scanning the encironment with a distance sensor
//----------------------------------------------------------------------------------------------------------------------
class SMCAgentLineDis : public SMCAgentEvo
{
  
public:  
  virtual void nextTrial(int trial);

protected:  
  virtual void updateFitness(float dt);
};  

//----------------------------------------------------------------------------------------------------------------------    
// Inlines  
//----------------------------------------------------------------------------------------------------------------------  
inline void SMCAgentLineDis::nextTrial(int trial)
{
  const std::vector<Positionable*>& objects = m_agent->getEnvironment().getObjects();
  int numObjects = objects.size();
  int visObj = trial % numObjects;
  
  for(int i = 0; i < numObjects; ++i)
  {
    if(i == visObj)
    {
      objects[i]->setVisibility(true);
      objects[i]->randomisePose();
    }
    else 
    {
      objects[i]->setVisibility(false); 
    }
  }
}

// Go left if object 0 is visible, otherwise right
//----------------------------------------------------------------------------------------------------------------------  
inline void SMCAgentLineDis::updateFitness(float dt)
{
  const std::vector<Positionable*>& objects = m_agent->getEnvironment().getObjects();

  // Determine where we want agent to be for the current trial/object
  float desPos;
  if(objects[0]->isVisible())
    desPos = m_agent->getMaxPosition();
  else if(objects[1]->isVisible())
    desPos = -m_agent->getMaxPosition();
  else
    desPos = objects[2]->getPosition().y;

  // Measure distance from desired position
  if(m_agent->getTime() >= m_fitnessEvalDelay)
  { 
    float diff = sqr(m_agent->getPosition().y - desPos);
    m_fitness += diff;
  } 
}  
  
} // namespace


#endif