/*
 *  SMCAgentEvo.cpp
 *  cinder_dynmx
 *
 *  Created by Thomas Buhrmann on 12/22/11.
 *  Copyright 2011 __MyCompanyName__. All rights reserved.
 *
 */

#include "SMCAgentEvo.h"

namespace dmx
{

static ci::Vec2f SMC_POS_LEFT (0.2, 0.2);
static ci::Vec2f SMC_POS_RIGHT (0.2, -0.2);
  
//----------------------------------------------------------------------------------------------------------------------  
SMCAgentEvo::SMCAgentEvo(int numNeurons)
{
  m_agent = new SMCAgent(numNeurons);
  init();
}

//----------------------------------------------------------------------------------------------------------------------    
void SMCAgentEvo::init() 
{   
  m_agent->getEnvironment()->addCircle(Circle(SMC_POS_LEFT, 0.05f));
#if SMC_CIRC_AND_TRI
  m_agent->getEnvironment()->addTriangle(Triangle(SMC_POS_RIGHT, 0.1f));
#endif
};  

//----------------------------------------------------------------------------------------------------------------------  
void SMCAgentEvo::reset() 
{ 
  // Set agent back to zero
  m_agent->reset();   
  
  m_fitness = 0.0f;
  
  // Randomise initial state of CTRNN
  m_agent->getCTRNN()->zeroStates();
# if 0  
  float min = -10;
  float max = 10;
  m_agent->getCTRNN()->randomizeState(min, max);
#endif  
  
  // Randomise environment objects
#if 0  
  if(m_agent->getEnvironment()->getCircles()[0].getPosition() == SMC_POS_LEFT)
  {
    m_agent->getEnvironment()->getCircles()[0].setPosition(SMC_POS_RIGHT);
#if SMC_CIRC_AND_TRI    
    m_agent->getEnvironment()->getTriangles()[0].setPosition(SMC_POS_LEFT);
#endif
  }
  else 
  {
    m_agent->getEnvironment()->getCircles()[0].setPosition(SMC_POS_LEFT);
#if SMC_CIRC_AND_TRI    
    m_agent->getEnvironment()->getTriangles()[0].setPosition(SMC_POS_RIGHT);
#endif
  }  
#endif
  
  // Randomise environment
  float distMin = 4 * m_agent->getRadius();
  float distMax = 12 * m_agent->getRadius();  
  float randDist = UniformRandom(distMin, distMax);
  float randAngle = UniformRandom(0, TWO_PI);
  float randRad = UniformRandom(0.025, 0.1);
  ci::Vec2f randPos = randDist * ci::Vec2f(cos(randAngle), sin(randAngle));
  m_agent->getEnvironment()->getCircles()[0].setPosition(randPos);
  m_agent->getEnvironment()->getCircles()[0].setRadius(randRad);
}
  
//----------------------------------------------------------------------------------------------------------------------  
void SMCAgentEvo::update(float dt) 
{ 
  m_agent->update(dt);
  
  // Measure fitness
  float agentAngle = m_agent->getAngle();
#if SMC_CIRC_AND_TRI  
  float triAngle = m_agent->getAngleWithHeading(m_agent->getEnvironment()->getTriangles()[0].getPosition());
#else
  float triAngle = m_agent->getAngleWithHeading(m_agent->getEnvironment()->getCircles()[0].getPosition());
#endif

  if(m_agent->getTime() >= 8.0f)
  {
    float diff = sqr(agentAngle - triAngle);
    m_fitness += diff;
  }
}

//----------------------------------------------------------------------------------------------------------------------
int SMCAgentEvo::getNumGenes()
{
  const int numNeurons = m_agent->getCTRNN()->getSize();
  const int numInputs = 1;
  return m_agent->getCTRNN()->getNumRequiredParams(numNeurons, numInputs);
}

//----------------------------------------------------------------------------------------------------------------------
void SMCAgentEvo::decodeGenome(const double* genome)
{
  m_agent->getCTRNN()->decodeGenome(genome, 1);
}

//----------------------------------------------------------------------------------------------------------------------
float SMCAgentEvo::getFitness()
{
  return - sqrt(m_fitness);
}

} // namespace