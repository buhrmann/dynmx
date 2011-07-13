/*
 *  PD.cpp
 *  cinder_dynmx
 *
 *  Created by Thomas Buhrmann on 28/06/2011.
 *  Copyright 2011 __MyCompanyName__. All rights reserved.
 *
 */

#include "PD.h"

#include <math.h>

namespace dmx
{

//----------------------------------------------------------------------------------------------------------------------
float PD::update(float target, float pos, float vel, float dt)
{
  if(m_numFramesDelay > 0)
  {
    // push new state on to delay line (pushes to the back)
    m_prevPositions.push(pos);
    m_prevVelocities.push(vel); 
    
    // if we haven't got enought frames yet, do nothing
    if(m_prevPositions.size() < m_numFramesDelay)
    {
      return 0.0f;
    }

    // otherwise we've had at least the required number of delay frames, so use the stored information
    pos = m_prevPositions.front(); 
    vel = m_prevVelocities.front();
    
    // remove the oldest information from queue (pops from front)
    m_prevPositions.pop();
    m_prevVelocities.pop();
  }
  
  m_target = target;
  
  // calculate target velocity here if needed
  float targetVel = (m_target - m_targetPrev) / dt;
  m_targetPrev = m_target;
  
  // position term
  float posErr = m_target - pos; 
  if(m_posFunction == kExponential)
  {
    posErr = pos > m_target ? - exp(-posErr) - 1.0f : exp(posErr) - 1.0f;
  }
  
  // velocity term
  float velErr = targetVel - vel;
  if(m_velFunction == kAsinh)
  {
    velErr = asinh(velErr);
  }

  return m_P * posErr + m_D * velErr;
}

} // namespace dmx