//
//  BodyLeg.h
//  dynmx
//
//  Created by Thomas Buhrmann on 25/08/14.
//
//

#ifndef dynmx_BodyBac_h
#define dynmx_BodyBac_h

#include "Dynmx.h"
#include "Model.h"
#include "Recorder.h"

#include "cinder/Vector.h"
#include <numeric>

namespace dmx
{
  
class LegBac;
class LegBacViz;
class LegBacView;
  
//----------------------------------------------------------------------------------------------------------------------
// Simple phototactic bacterial agent
//----------------------------------------------------------------------------------------------------------------------
class BodyBac : public Model
{
  
friend LegBac;
friend LegBacViz;
friend LegBacView;
  
public:
  
  BodyBac();
  ~BodyBac() {};
  
  // Inherited from class Model
  virtual void init();
  virtual void reset();
  virtual void update(float dt);
  virtual bool hasFinished() { return m_fitnesses.size() >= m_maxNumFoods; };
  
  void randomiseFood();
  void setNet(CTRNN* net) { m_net = net; };
  void setFoodPresentation(int num, float duration) { m_maxNumFoods = num; m_foodDur = duration; };

  float getReward() const { return m_reward; };
  float getFitness() const { return m_fitness; };
  float getFinalFitness();
  
  
protected:
  
  // Parameters
  float m_radius;
  float m_sensorSig;
  float m_maxSpeed;
  float m_maxAngSpeed;
  float m_foodDur;
  int m_maxNumFoods;
  
  CTRNN* m_net;
  
  // State
  float m_time;
  float m_angle;
  float m_fitness;
  float m_reward;
  std::vector<float> m_fitnesses;
  float m_initDist;
  
  ci::Vec2f m_position;
  ci::Vec2f m_velocity;
  ci::Vec2f m_foodPos;
  
};
  
//----------------------------------------------------------------------------------------------------------------------
// Implmenetation
//----------------------------------------------------------------------------------------------------------------------
inline BodyBac::BodyBac()
{
  init();
}
  
//----------------------------------------------------------------------------------------------------------------------
inline void BodyBac::init()
{
  m_radius = 0.55;
  m_maxSpeed = 0.5f;
  m_maxAngSpeed = 1.0f;
  m_foodDur = 1.0f;
  
  const float sigma = 6;
  m_sensorSig = -1 / (2*sigma*sigma);
}

//----------------------------------------------------------------------------------------------------------------------
inline void BodyBac::randomiseFood()
{
  float fang = UniformRandom(0, TWO_PI);
  float fdist = 10.0f + UniformRandom(0, 5);
  m_foodPos = m_position + ci::Vec2f(fdist * cos(fang), fdist * sin(fang));
  m_initDist = m_position.distance(m_foodPos);
  m_time = 0;
  m_fitness = 0;
}

//----------------------------------------------------------------------------------------------------------------------
inline void BodyBac::reset()
{
  m_position.set(0,0);
  m_velocity.set(0,0);
  m_angle = 0;

  randomiseFood();

  m_reward = 0;
  m_fitness = 0;
  m_fitnesses.clear();
  m_time = 0;
}

//----------------------------------------------------------------------------------------------------------------------
inline void BodyBac::update(float dt)
{
  // Sense
  float dSq = m_position.distanceSquared(m_foodPos);
  float sensor = exp(dSq * m_sensorSig);
  
  // Update net
  m_net->setExternalInput(0, sensor);
  m_net->updateDynamic(dt);
  
  // Move
  const int N = m_net->getSize();
  float angSpeed = m_maxAngSpeed * (m_net->getOutput(N-1) - m_net->getOutput(N-2)) / m_radius;
  float fwdSpeed = m_maxSpeed * (m_net->getOutput(N-1) + m_net->getOutput(N-2)) / 2;
  
  m_angle += dt * angSpeed;
  if(m_angle >= TWO_PI || m_angle <= -TWO_PI)
    m_angle = 0;
  
  m_velocity = fwdSpeed * ci::Vec2f(cos(m_angle), sin(m_angle));
  m_position += dt * m_velocity;
  
  // Update fitness
  m_fitness += dt * sqrt(dSq);
  
  m_time += dt;

  if(m_time >= m_foodDur)
  {
    // Save fitness for this presentation of food
    float fit = (m_initDist - (m_fitness / m_time)) / m_initDist;
    fit = std::max(fit, 0.0f);
    m_fitnesses.push_back(fit);
    //std::cout << "Food# " << m_fitnesses.size() << " : " << fit << std::endl;
    
    // New presentation
    randomiseFood();
  }
  
  // Calculate instant reward
  ci::Vec2f desDir = (m_foodPos - m_position).normalized();
  float proj = m_velocity.dot(desDir);
  m_reward = clamp(proj, 0.0f, 1.0f) / m_maxSpeed;
  
}

//----------------------------------------------------------------------------------------------------------------------
inline float BodyBac::getFinalFitness()
{
  //float fit = mean(m_fitnesses);
  //float fit = m_fitnesses[m_fitnesses.size() - 1];
  float mu, sd;
  stdev(m_fitnesses, mu, sd, m_fitnesses.size()/2, 0);
  float fit = mu - 0.5*sd;
  
  return fit;
}
  
}

#endif
