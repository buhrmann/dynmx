/*
 *  PD.h
 *  cinder_dynmx
 *
 *  Created by Thomas Buhrmann on 28/06/2011.
 *  Copyright 2011 __MyCompanyName__. All rights reserved.
 *
 */

#ifndef _DMX_PD_
#define _DMX_PD_

#include "Dynmx.h"

#include <queue>

namespace dmx
{

//----------------------------------------------------------------------------------------------------------------------
/// Simple controller
//----------------------------------------------------------------------------------------------------------------------
class PD
{
  
public:

  enum NonLinearity
  {
    kLinear = 0,
    kExponential,
    kAsinh
  };

  PD() : m_target(0.0f), m_targetPrev(0), m_P(10.0f), m_D(1.0f), m_posFunction(kLinear), m_velFunction(kLinear),
    m_numFramesDelay(0) {};    
  
  void init(){};
  void reset(){};
  
  float update(float target, float pos, float vel, float dt);
  
  float m_P;
  float m_D;
  
protected:
    
  std::queue<float> m_prevPositions;
  std::queue<float> m_prevVelocities;
  
  unsigned int 
    m_numFramesDelay,
    m_posFunction,
    m_velFunction;
  
  float
    m_target,
    m_targetPrev;
    
}; // class PD

} // namespace dmx

#endif
