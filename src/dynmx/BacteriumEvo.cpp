//
//  BacteriumEvo.cpp
//  dynmx
//
//  Created by Thomas Buhrmann on 27/02/14.
//
//

#include "BacteriumEvo.h"


namespace dmx
{
  
//----------------------------------------------------------------------------------------------------------------------
void BacteriumEvo::init()
{
  // Already called by superclass constructor
  //SMCAgentEvo::init();
  
  if (SETTINGS->hasChild("Config/GA/Evolvable"))
  {
    const ci::XmlTree& xml = SETTINGS->getChild("Config/GA/Evolvable");
    m_numPhases = xml.getChild("TrialDuration").getAttributeValue<float>("numPhases", 1);
    m_phaseDuration = m_trialDuration / m_numPhases;
  }
  
  m_randInitProp = 0.0f;
};
  
//----------------------------------------------------------------------------------------------------------------------
void BacteriumEvo::reset()
{
  // Set agent back to zero
  m_agent->reset();
  m_agent->getCTRNN().zeroStates();
  
  m_fitness = 1.0f;
  m_fitnessInst = 0.0f;
  m_phaseFit = 0.0f;

  m_phase = -1;
  nextPhase();
}

//----------------------------------------------------------------------------------------------------------------------
void BacteriumEvo::update(float dt)
{
  m_agent->update(dt);
  
  updateFitness(dt);
  
  m_phaseTime += dt;
  if (m_phaseTime >= m_phaseDuration)
    nextPhase();
}
  
//----------------------------------------------------------------------------------------------------------------------
void BacteriumEvo::nextStage(int stage)
{
  m_randInitProp = clamp(stage * 0.1f, 0.0f, 1.0f);
}

//----------------------------------------------------------------------------------------------------------------------
void BacteriumEvo::nextPhase()
{
  m_phaseTime = 0;
  
  // Process fitness for previous trial
  if(m_phase % 3 == 0)
    m_phaseFit = 1;
  else
    m_phaseFit /= m_phaseDuration;

  if(m_phase > -1)
    m_fitness *= m_phaseFit;
  
  m_phase++;
  m_phaseFit = 0;
  
  m_agent->reset(); // doesn't reset ctrnn
  
  // Setup next trial
  // Invisible food
  const std::vector<Positionable*>& objects = m_agent->getEnvironment().getObjects();
  
#define CHANGE_VISIBILITY 0
#if CHANGE_VISIBILITY
  if(m_phase % 3 == 2)
    objects[1]->setVisibility(false);
  else
    objects[1]->setVisibility(true);
#endif
  
  // Set food location
  float foodR = m_phase < 3 ? 0.2 : 0.8;
  ((Torus*)objects[1])->setRadius(foodR);

  // Set agent position and orientation
  float p = 0.5 + 0.2 * m_randInitProp * UniformRandom(-1, 1);
  float a = m_randInitProp * UniformRandom(-TWO_PI, TWO_PI);
  m_agent->setAngle(a);
  m_agent->setPosition(ci::Vec2f(0, p));
}

//----------------------------------------------------------------------------------------------------------------------
float BacteriumEvo::getFitness()
{
  return m_fitness;
};

//----------------------------------------------------------------------------------------------------------------------
void BacteriumEvo::nextTrial(int t)
{
#define TRIAL_FUNC 2
  
#if TRIAL_FUNC == 1
  trialTaxisGrWidth(t);
#elif TRIAL_FUNC == 2
  trialFoodPos(t);
#endif
}

// 4 orientations x 2 positions x 2 radii
//----------------------------------------------------------------------------------------------------------------------
void BacteriumEvo::trialTaxisGrWidth(int t)
{
  // Change gradient radius
  float r = 0.6;
  if(t >= 8)
    r = 1.2;
  
  const std::vector<Positionable*>& objects = m_agent->getEnvironment().getObjects();
  ((Gradient*)objects[0])->setRadius(r);
  ((Torus*)objects[1])->setRadius(0.5*r);
  
  // Change initial agent orientation
  int orientTrial = t % 4;
  float angle = PI_OVER_FOUR + PI_OVER_TWO * orientTrial;
  
  // Change initial agent position (0.25 or 0.75 * radius)
  int posTrial = (t / 4) % 2;
  float p = r * (0.25 + posTrial * 0.5);
  
  m_agent->setAngle(angle);
  m_agent->setPosition(ci::Vec2f(0,  p));
}

//----------------------------------------------------------------------------------------------------------------------
void BacteriumEvo::trialFoodPos(int t)
{
  
}
  
//----------------------------------------------------------------------------------------------------------------------
void BacteriumEvo::updateFitness(float dt)
{
  
  //if(m_agent->getTime() > m_fitnessEvalDelay)
  if(m_phase % 3 > 0)
  {
    const std::vector<Positionable*>& objects = m_agent->getEnvironment().getObjects();
    
    // Desired thermal level
    float foodR = ((Torus*)objects[1])->getRadius();
    float agentR = m_agent->getPosition().distance(objects[1]->getPosition());
    
    //float foodMax = ((Torus*)objects[1])->getPeak();
    
    const float initialDist = fabs(0.5f - foodR);
    const float currentDist = fabs(agentR - foodR);
    float relAgentDist = 0.5f * clamp(currentDist / initialDist, 0.0f, 2.0f); // in [0, 1]
    
    m_fitnessInst = 1.0f - relAgentDist;
    //m_fitnessInst = 1.0f - clamp(fabs(foodR - agentDist), 0.0f, 1.0f);
    //m_fitnessInst = m_agent->getTorusSensor()->getLevel() / foodMax;
    
    //float ft = sqrt(1.0f - 2.0f * fabs(m_agent->getGradientSensor()->getActivation() - 0.5f));
    //float f = m_agent->getSensedFood() * dt;
    
    // Minimise angular velocity to avoid circling on the spot
    float fv = 1.0f - (fabs(m_agent->getAngularSpeed()) / m_agent->getMaxAngularSpeed());

    // Maximise energy
    //float f = m_agent->getEnergy() * dt;
    
    //assert(ft < 1 && fv < 1);
    m_phaseFit += m_fitnessInst * fv * dt;
  }
};

}