//
//  Bacterium.cpp
//  dynmx
//
//  Created by Thomas Buhrmann on 24/02/14.
//
//

#include "Bacterium.h"
#include "MathUtils.h"

namespace dmx
{

//----------------------------------------------------------------------------------------------------------------------
void Bacterium::init()
{
  SMCAgent::init();
}
  
//----------------------------------------------------------------------------------------------------------------------
void Bacterium::reset()
{
  SMCAgent::reset();
}
  
//----------------------------------------------------------------------------------------------------------------------
void Bacterium::update(float dt)
{
  
  // Sense environment
  updateSensors(dt);
  
  float sensed = 0;
  if( hasGradientSensor() )
    sensed = m_gradientSensor->getLevel();
  
  m_food = 0;
  if( hasTorusSensor() )
    m_food = m_torusSensor->getLevel() * m_velocity.length() / m_maxSpeed;
  
  // Metabolise
  float tau = 0.25f;
  float dA = tau * ((-0.075*m_energy*m_energy*m_energy) + (0.5*m_food*m_energy*m_energy) - m_energy);
  m_energy += dA * dt;
  
  // Neural dynamics
  m_ctrnn->setExternalInput(0, sensed);
  
  if(m_topology.getNumInputs() > 1)
    //m_ctrnn->setExternalInput(1, getSensedEnergy());
    m_ctrnn->setExternalInput(1, m_torusSensor->getLevel());
  
  if(m_topology.getNumInputs() > 2)
    m_ctrnn->setExternalInput(2, m_gradientSensor->getDerivative());
  
  if(m_topology.getNumInputs() > 3)
    m_ctrnn->setExternalInput(3, m_energy);
  
  m_ctrnn->updateDynamic(dt);
  
  /*if(m_energy < 1)
  {
    m_ctrnn->leakWeights(0.99, 0.1);
  }*/
 
  
  // Move
  float angSpeed, speed = 0;
  if(m_topology.getNumOutputs() == 1)
  {
    const int motRot = m_topology.getSize() - 1;
    angSpeed = 2 * m_ctrnn->getOutput(motRot) - 1;
    speed = m_maxSpeed;
  }
  else
  {
    const int mot1 = m_topology.getSize() - 1;
    const int mot2 = m_topology.getSize() - 2;
    angSpeed = m_ctrnn->getOutput(mot1) - m_ctrnn->getOutput(mot2);
    speed = 0.5 * (m_ctrnn->getOutput(mot1) + m_ctrnn->getOutput(mot2));
  }

  
  float speedScalar = 1.0f;
  if(m_energy < m_energySpeedTresh)
    speedScalar = (1 / m_energySpeedTresh) * m_energy;
  
  // Rotation
  m_angularSpeed = speedScalar * m_maxAngularSpeed * angSpeed;
  m_angle += m_angularSpeed * dt;
  if(m_angleWraps)
    m_angle = wrap(m_angle, -m_maxAngle, m_maxAngle);
  else
    m_angle = clamp(m_angle, -m_maxAngle, m_maxAngle);

  // Translation
  m_velocity = speedScalar * speed * ci::Vec2f(cos(m_angle), sin(m_angle));
  m_position += m_velocity * dt;
  
  if(m_positionWraps){
    m_position.y = wrap(m_position.y, -m_maxPosition, m_maxPosition);
    m_position.x = wrap(m_position.x, -m_maxPosition, m_maxPosition);
  }
  
  m_time += dt;
  
}
  
//----------------------------------------------------------------------------------------------------------------------
void Bacterium::record(Recorder& recorder)
{
  SMCAgent::record(recorder);
  const std::vector<Positionable*>& objects = getEnvironment().getObjects();
  float foodR = ((Torus*)objects[1])->getRadius();
  recorder.push_back("foodR", foodR);
}


} // namespace