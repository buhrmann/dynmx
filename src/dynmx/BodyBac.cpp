//
//  BodyBac.cpp
//  dynmx
//
//  Created by Thomas Buhrmann on 08/10/14.
//
//

#include "BodyBac.h"

#include <numeric>
#include <algorithm>

namespace dmx
{
  
//----------------------------------------------------------------------------------------------------------------------
// LightSensor implementation
//----------------------------------------------------------------------------------------------------------------------
float LightSensor::s_sigma = -1 / (2 * sqr(50.f)); // At x, sensor reaches half of its maximum
float LightSensor::s_maxSensorAng = 3 * PI_OVER_FOUR;
  
//----------------------------------------------------------------------------------------------------------------------
void LightSensor::setPosition(const ci::Vec2f& newPos, float a)
{
  m_position = newPos + ci::Vec2f(m_offset*cos(m_angle + a), m_offset*sin(m_angle + a));
  m_direction = (m_position - newPos).normalized();
}

//----------------------------------------------------------------------------------------------------------------------
float LightSensor::sense(const ci::Vec2f& lightPos)
{
  // Shadow test
  ci::Vec2f lightDir = (lightPos - m_position).normalized();
  float cosangle = lightDir.dot(m_direction);
  float angle = acos(clamp(cosangle, -1.f, 1.f));
  if (angle > s_maxSensorAng)
  {
    m_signal = 0;
    return m_signal;
  }
  
  // Distance scaling
  float dSq = m_position.distanceSquared(lightPos);
  //m_signal = m_gain / dSq;
  m_signal = m_gain * exp(dSq * s_sigma);
  
  bool angleSensitive = true;
  if (angleSensitive)
    m_signal *= 1 - (angle / s_maxSensorAng);
  
  return m_signal;
}

//----------------------------------------------------------------------------------------------------------------------
// BodyBac implementation
//----------------------------------------------------------------------------------------------------------------------
BodyBac::BodyBac()
{
  init();
}

//----------------------------------------------------------------------------------------------------------------------
void BodyBac::init()
{
  m_radius = 0.55;
  m_maxSpeed = 0.5f;
  m_maxAngSpeed = 1.0f;
  m_foodDur = 1.0f;
  m_sensorAngRange = degreesToRadians(120.0f);
  
  const int numSensors = 2;
  for (int i = 0; i < numSensors; ++i)
    m_sensors.push_back(LightSensor(0, m_radius));
  
  resetMorphology();
}

//----------------------------------------------------------------------------------------------------------------------
void BodyBac::randomiseFood()
{
  float fang = UniformRandom(0, TWO_PI);
  float fdist = 20.0f + UniformRandom(0, 20);
  m_foodPos = m_position + ci::Vec2f(fdist * cos(fang), fdist * sin(fang));
  m_initDist = m_position.distance(m_foodPos);
}
  
//----------------------------------------------------------------------------------------------------------------------
void BodyBac::resetPosition()
{
  m_position.set(0,0);
  m_velocity.set(0,0);
  m_angle = 0;
  
  for(auto& s : m_sensors)
    s.setPosition(m_position, m_angle);
}
  
//----------------------------------------------------------------------------------------------------------------------
void BodyBac::resetMorphology()
{
  // Reset sensor order
  const int numSensors = m_sensors.size();
  for (int i = 0; i < numSensors; ++i)
  {
    const float a = -m_sensorAngRange/2 + (i * (m_sensorAngRange/(numSensors - 1)));
    m_sensors[i].setAngle(a);
    m_sensors[i].setPosition(m_position, m_angle);
  }
  
  // Reset motor order
  m_motorId1 = 1;
  m_motorId2 = 2;
}

//----------------------------------------------------------------------------------------------------------------------
void BodyBac::reset()
{
  resetPosition();
  
  randomiseFood();
  
  m_reward = 0;
  m_fitness = 0;
  m_fitnesses.clear();
  m_time = 0;
}
  
//----------------------------------------------------------------------------------------------------------------------
void BodyBac::invertVision()
{
  // Reverse the order of sensors, such that when we iterate over them neurons receive left-right reversed information
  std::reverse(m_sensors.begin(), m_sensors.end());
}
  
//----------------------------------------------------------------------------------------------------------------------
void BodyBac::shiftVision(float degrees)
{
  // Reverse the order of sensors, such that when we iterate over them neurons receive left-right reversed information
  for(auto& s : m_sensors)
    s.setAngle(s.m_angle + degreesToRadians(degrees));
}
  
//----------------------------------------------------------------------------------------------------------------------
void BodyBac::invertMotors()
{
  std::swap(m_motorId1, m_motorId2);
}

//----------------------------------------------------------------------------------------------------------------------
void BodyBac::update(float dt)
{
  // Sense
  for (int i = 0; i < m_sensors.size(); ++i)
  {
    const float s = m_sensors[i].sense(m_foodPos);
    m_net->setExternalInput(i, s);
  }
  
  // Update net
  m_net->updateDynamic(dt);

  // Move
  const int N = m_net->getSize();
  const float mot1 = m_net->getOutput(N - m_motorId1);
  const float mot2 = m_net->getOutput(N - m_motorId2);
  float angSpeed = m_maxAngSpeed * (mot1 - mot2) / m_radius;
  float fwdSpeed = m_maxSpeed * (mot1 + mot2) / 2;
  
  m_angle += dt * angSpeed;
  if(m_angle >= TWO_PI || m_angle <= -TWO_PI)
    m_angle = 0;
  
  m_velocity = fwdSpeed * ci::Vec2f(cos(m_angle), sin(m_angle));
  m_position += dt * m_velocity;
  
  for (auto& s : m_sensors)
    s.setPosition(m_position, m_angle);
  
  m_time += dt;
  
  // Update fitness
  m_fitness += dt * m_position.distance(m_foodPos);
  if(m_time >= m_foodDur)
  {
    // Save fitness for this presentation of food
    float fit = 1 - ((m_fitness / m_time) / m_initDist);
    fit = std::max(fit, 0.0f);
    m_fitnesses.push_back(fit);
    //std::cout << "Food# " << m_fitnesses.size() << " : " << fit << std::endl;
    
    // New presentation
    //resetPosition();
    randomiseFood();
    m_time = 0;
    m_fitness = 0;
  }
  
  // Calculate instant reward
  ci::Vec2f desDir = (m_foodPos - m_position).normalized();
  float proj = m_velocity.dot(desDir);
  m_reward = clamp(proj, -1.0f, 1.0f) / m_maxSpeed;
  
}

//----------------------------------------------------------------------------------------------------------------------
float BodyBac::getFinalFitness()
{
  //float fit = mean(m_fitnesses);
  //float fit = m_fitnesses[m_fitnesses.size() - 1];
  float mu, sd;
  stdev(m_fitnesses, mu, sd, m_fitnesses.size()/2, 0);
  float fit = mu - 2.0*sd;
  
  return fit;
}
  
} // namespace