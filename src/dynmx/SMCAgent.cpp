/*
 *  SMCAgent.cpp
 *  cinder_dynmx
 *
 *  Created by Thomas Buhrmann on 12/18/11.
 *  Copyright 2011 __MyCompanyName__. All rights reserved.
 *
 */

#include "SMCAgent.h"

namespace dmx
{
  
//----------------------------------------------------------------------------------------------------------------------
SMCAgent::SMCAgent() : 
  m_position(ci::Vec2f(0.0f, 0.0f)), 
  m_angle(0.0f), 
  m_maxSpeed(5.0f), 
  m_radius(0.03f),
  m_sensedValue(0.0f) 
{
  init();
}

//----------------------------------------------------------------------------------------------------------------------
void SMCAgent::init()
{
  m_distanceSensor.setMaxDistance(0.5f);
  
  m_environment.addCircle(Circle(ci::Vec2f(0.3, 0.3), 0.05f));
  m_environment.addTriangle(Triangle(ci::Vec2f(-0.1, -0.2), ci::Vec2f(-0.2, -0.1), ci::Vec2f(-0.2, -0.2)));  
  
  m_ctrnn.setGain(0, 100.0f);
  
  reset();
}
  
//----------------------------------------------------------------------------------------------------------------------
void SMCAgent::reset()
{
  m_angle = 0.0f;
  m_position = ci::Vec2f(0.0f, 0.0f);
  updateSensor(0.0f);
  
  // Randomise environment
  float distMin = 3 * m_radius;
  float distMax = 6 * m_radius;
  
  float randDist = UniformRandom(distMin, distMax);
  float randAngle = UniformRandom(0, TWO_PI);
  ci::Vec2f randPos = randDist * ci::Vec2f(cos(randAngle), sin(randAngle));
  m_environment.getCircles()[0].position = randPos;
  
//  randDist = UniformRandom(distMin, distMax);
//  randAngle = UniformRandom(0, TWO_PI);
//  randPos = randDist * ci::Vec2f(cos(randAngle), sin(randAngle));
//  m_environment.getTriangles()[0].position = randPos;
  
  // Randomise CTRNN
  float min = -5;
  float max = 5;
  //m_ctrnn.randomizeState(min, max);
  m_ctrnn.randomizeWeights(min, max);
  m_ctrnn.randomizeBiases(min, max);
  m_ctrnn.randomizeTimeConstants(0.2, 0.5);
  //m_ctrnn.setCenterCrossing();
}  
  
//----------------------------------------------------------------------------------------------------------------------
void SMCAgent::update(float dt)
{
  // Sense
  updateSensor(dt);
  m_sensedValue = 1.0f - m_distanceSensor.getDistanceProportional();
  
  // Think
  m_ctrnn.setExternalInput(0, m_sensedValue);
  m_ctrnn.update(dt);
  
  // Act
  float motorDiff = m_ctrnn.getOutput(1) - m_ctrnn.getOutput(2);
  float vel = motorDiff * m_maxSpeed;
  m_angle += vel * dt; 
}

//----------------------------------------------------------------------------------------------------------------------
void SMCAgent::updateSensor(float dt)
{
  m_distanceSensor.setPosition(m_position);
  m_distanceSensor.setDirection(ci::Vec2f(cos(m_angle), sin(m_angle)));
  m_distanceSensor.senseEnvironment(m_environment);
}
                                
} // namespace