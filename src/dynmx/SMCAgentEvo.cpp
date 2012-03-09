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
SMCAgentEvo::SMCAgentEvo()
{
  init();
}

//----------------------------------------------------------------------------------------------------------------------    
SMCAgentEvo::~SMCAgentEvo()
{
  delete m_agent;
}

//----------------------------------------------------------------------------------------------------------------------    
void SMCAgentEvo::init() 
{   
  const ci::XmlTree* settings = SETTINGS;
  if (settings->hasChild("Config/GA/Evolvable"))
  {
    // Use setting from globals file
    const ci::XmlTree& xml = settings->getChild("Config/GA/Evolvable");
    
    const int numNeurons = xml.getChild("NumNeurons").getValue<int>();    
    m_agent = new SMCAgent(numNeurons);

    const double maxSensorDist = xml.getChild("MaxSensorDist").getValue<double>();    
    m_agent->setMaxSensorDistance(maxSensorDist);
    
    const double maxSpeed = xml.getChild("MaxSpeed").getValue<double>();    
    m_agent->setMaxSpeed(maxSpeed);
    
    const double maxAngularSpeed = degreesToRadians(xml.getChild("MaxAngularSpeed").getValue<double>());
    m_agent->setMaxAngularSpeed(maxAngularSpeed);
  }

  m_agent->getEnvironment()->addCircle(Circle(SMC_POS_LEFT, 0.05f));
#if SMC_CIRC_AND_TRI
  m_agent->getEnvironment()->addTriangle(Triangle(SMC_POS_RIGHT, 0.1f));
  
  m_agent->getEnvironment()->addLine(Line(ci::Vec2f(0.4,0.3), -PI_OVER_TWO, 0.6));
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
#if SMC_CIRC_AND_TRI
  float maxOffset = 0.1;
  if(m_agent->getEnvironment()->getCircles()[0].getPosition().y > 0)
  {
    // if circle now left
    ci::Vec2f randOffset (UniformRandom(-maxOffset, maxOffset), UniformRandom(-maxOffset, maxOffset));
    m_agent->getEnvironment()->getCircles()[0].setPosition(SMC_POS_RIGHT + randOffset);
    randOffset = ci::Vec2f(UniformRandom(-maxOffset, maxOffset), UniformRandom(-maxOffset, maxOffset));    
    m_agent->getEnvironment()->getTriangles()[0].setPosition(SMC_POS_LEFT + randOffset);
  }
  else 
  {
    ci::Vec2f randOffset (UniformRandom(-maxOffset, maxOffset), UniformRandom(-maxOffset, maxOffset));
    m_agent->getEnvironment()->getCircles()[0].setPosition(SMC_POS_LEFT + randOffset);
    randOffset = ci::Vec2f(UniformRandom(-maxOffset, maxOffset), UniformRandom(-maxOffset, maxOffset));    
    m_agent->getEnvironment()->getTriangles()[0].setPosition(SMC_POS_RIGHT + randOffset);

  }  
#else
  float distMin = 4 * m_agent->getRadius();
  float distMax = 12 * m_agent->getRadius();  
  float randDist = UniformRandom(distMin, distMax);
  float randAngle = UniformRandom(0, TWO_PI);
  float randRad = UniformRandom(0.025, 0.1);
  ci::Vec2f randPos = randDist * ci::Vec2f(cos(randAngle), sin(randAngle));
  m_agent->getEnvironment()->getCircles()[0].setPosition(randPos);
  m_agent->getEnvironment()->getCircles()[0].setRadius(randRad);
#endif
}
  
//----------------------------------------------------------------------------------------------------------------------  
void SMCAgentEvo::update(float dt) 
{ 
  m_agent->update(dt);
  
  // Measure fitness: find circle 
  float agentAngle = m_agent->getAngle();
  float circleAngle = m_agent->getAngleWithHeading(m_agent->getEnvironment()->getTriangles()[0].getPosition());

  if(m_agent->getTime() >= 8.0f)
  {
    float diff = sqr(agentAngle - circleAngle);
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