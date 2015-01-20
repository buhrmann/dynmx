//
//  BodyLeg.cpp
//  dynmx
//
//  Created by Thomas Buhrmann on 04/11/14.
//
//

#include "BodyLeg.h"
#include "MathUtils.h"

namespace dmx
{

//----------------------------------------------------------------------------------------------------------------------
BodyLeg::BodyLeg()
{
  init();
}

//----------------------------------------------------------------------------------------------------------------------
void BodyLeg::init()
{
  s_legLength = 15.0f;
  s_maxFootDist = 20.0f;
  s_maxLegForce = 0.05f;
  s_forwardAngleLimit = PI/6;
  s_backwardAngleLimit = -PI/6;
  s_maxVelocity = 1.2; // Was 6. Determined by forcing force=maxforce and observing velocity before angle limit reached
  s_maxTorque = 0.5;
  s_maxAngSpeed = 0.25; // Was 1. Determined by forcing force=maxforce and observing angSpeed before limit is reached
  s_maxAsymVel = 0.627;
  s_minLegSensor = -1.0;
  s_maxLegSensor = 0.524;
  
  m_sensorRange = s_maxLegSensor - s_minLegSensor;
  m_sensorOffset = s_minLegSensor / m_sensorRange;
  
  m_duration = 10.0f;
}

//----------------------------------------------------------------------------------------------------------------------
void BodyLeg::reset()
{
  m_time = 0;
  m_force = 0;
  m_position.set(0,0);
  m_velocity.set(0,0);
  m_footDown = false;
  
  m_angle = s_forwardAngleLimit; //UniformRandom(s_backwardAngleLimit, s_forwardAngleLimit);
  m_sensor = (m_angle / m_sensorRange) - m_sensorOffset; // Normalised sensor
  m_angSpeed = m_forwardForce = m_backwardForce = 0.0;
  m_joint = m_position;
  const float jointOffset = 0.0;
  m_joint.y += jointOffset;
  
  m_foot.x = m_joint.x + s_legLength * sin(m_angle);
  m_foot.y = m_joint.y + s_legLength * cos(m_angle);
  
  m_reward = 0;
  m_fitness = 0;
}

//----------------------------------------------------------------------------------------------------------------------
void BodyLeg::update(float dt)
{
  // Sense
  m_sensor = (m_angle / m_sensorRange) - m_sensorOffset;
  
  // Update net
  m_net->setExternalInput(0, m_sensor);
  m_net->setExternalInput(1, m_footDown);
  
  m_net->updateDynamic(dt);
  
  m_force = 0.0f;
  float newangle = 0.0f;
  
  const int N = m_net->getSize();
  const int idFoot = N - 1;
  const int idFwd = N - 2;
  const int idBwd = N - 3;
  
  if (m_net->getOutput(idFoot) > 0.5)
  {
    m_footDown = true;
    m_angSpeed = 0;
  }
  else
  {
    m_footDown = false;
  }
  
  m_forwardForce = m_net->getOutput(idFwd) * s_maxLegForce;
  m_backwardForce = m_net->getOutput(idBwd) * s_maxLegForce;
  
  // Compute the force applied to the body
  if (m_footDown && m_angle >= s_backwardAngleLimit && m_angle <= s_forwardAngleLimit)
  {
    m_force = m_forwardForce - m_backwardForce;
  }
  
  // Update the position of the body
  m_velocity.x += dt * m_force;
  m_velocity.x = clamp(m_velocity.x, -s_maxVelocity, s_maxVelocity);
  
  m_position += dt * m_velocity;
  m_joint += dt * m_velocity;
  
  // Update the leg geometry
  if (m_footDown)
  {
    newangle = atan2(m_foot.x - m_joint.x, m_foot.y - m_joint.y);
    m_angSpeed = (newangle - m_angle) / dt;
    m_angle = newangle;
  }
  else
  {
    m_velocity.set(0,0);
    m_angSpeed += dt * s_maxTorque * (m_backwardForce -  m_forwardForce);
    m_angSpeed = clamp(m_angSpeed, -s_maxAngSpeed, s_maxAngSpeed);
    
    m_angle += dt * m_angSpeed;
    if (m_angle < s_backwardAngleLimit)
    {
      m_angle = s_backwardAngleLimit;
      m_angSpeed = 0;
    }
    else if (m_angle > s_forwardAngleLimit)
    {
      m_angle = s_forwardAngleLimit;
      m_angSpeed = 0;
    }
    
    m_foot.x = m_joint.x + s_legLength * sin(m_angle);
    m_foot.y = m_joint.y + s_legLength * cos(m_angle);
  }
  
  // If the foot is too far back, the body becomes "unstable" and forward motion ceases
  if (m_joint.distance(m_foot) > s_maxFootDist)
    m_velocity.x = 0.0;
  
  // Update reward
  if (m_footDown)
    m_reward = m_velocity.x / s_maxVelocity;
  else
    m_reward = -m_angSpeed / s_maxAngSpeed;
  
  m_fitness += dt * m_reward;
  
  m_time += dt;    
}

//----------------------------------------------------------------------------------------------------------------------
float BodyLeg::getFinalFitness()
{
  return fabs(m_position.x);
  //return m_fitness / m_duration;
}
  
} // namespace
