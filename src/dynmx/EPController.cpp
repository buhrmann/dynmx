/*
 *  EPController.cpp
 *  cinder_dynmx
 *
 *  Created by Thomas Buhrmann on 8/2/11.
 *  Copyright 2011 __MyCompanyName__. All rights reserved.
 *
 */

#include "EPController.h"
#include "Muscle.h"
#include "MathUtils.h"
#include <algorithm>

namespace dmx
{

//----------------------------------------------------------------------------------------------------------------------      
EPController::EPController(Muscle* muscle, float pGain, float vGain, float dGain, int delay) :
  m_muscle(muscle),
  m_pGain(pGain),
  m_vGain(vGain),
  m_dGain(dGain),
  m_numFramesDelay(delay)
{
}
  
//----------------------------------------------------------------------------------------------------------------------    
void EPController::init()
{
  assert(m_muscle);
  reset();
}
  
//----------------------------------------------------------------------------------------------------------------------    
void EPController::reset()
{
  // Set desired to current state by default
  m_desiredLength = m_muscle->getLength();
  m_desiredLengthPrev = m_desiredLength;
  m_desiredVelocity = m_muscle->getVelocity();
  
  m_activation = 0.0;
  
  // queues don't have a clear() function
  m_lengths = std::queue<double>();
  m_velocities = std::queue<double>();
}
  
//----------------------------------------------------------------------------------------------------------------------    
void EPController::update(float dt)
{
  double length = m_muscle->getLength();
  
  assert(length >=0 && length <= 10);
  
  if(length > 10.0)
  {
    length = 1.0;
    return;
  }
  
  double velocity = m_muscle->getVelocity();
  
  // If we're dealing with delayed feedback
  if(m_numFramesDelay > 0)
  {
    // Push new state on to delay line (pushes to the back)
    m_lengths.push(length);
    m_velocities.push(velocity); 
    
    // If we haven't got enough frames yet, do nothing
    if(m_lengths.size() < m_numFramesDelay)
    {
      m_activation = 0.0;
      return;
    }
    
    // Otherwise we've had at least the required number of delay frames, so use the stored information
    length = m_lengths.front(); 
    velocity = m_velocities.front();
    
    // Remove the oldest information from queue (pops from front)
    m_lengths.pop();
    m_velocities.pop();
  }
  
  // Regular "PD"-like components
  m_activation = m_pGain * (length - m_desiredLength) + m_dGain * velocity;
  
  // Calculate target velocity here if needed
  if(m_vGain > 0.0)
  {
    m_desiredVelocity = (m_desiredLength - m_desiredLengthPrev) / dt;
    m_activation += m_vGain * (velocity - m_desiredVelocity);
  }  
  
  // Clamp to allowed range
	m_activation = clamp(m_activation, 0.0, 1.0);
  if(m_activation > 1.0 || m_activation < 0.0)
  {
    m_activation = 0.0;
  }
  
}
  
}
