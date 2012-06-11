/*
 *  Delay.h
 *  cinder_dynmx
 *
 *  Created by Thomas Buhrmann on 5/22/12.
 *  Copyright 2012 __MyCompanyName__. All rights reserved.
 *
 */

#ifndef _DMX_DELAY_
#define _DMX_DELAY_

#include "Dynmx.h"

#include <queue>

namespace dmx
{
  
//----------------------------------------------------------------------------------------------------------------------
// 
//----------------------------------------------------------------------------------------------------------------------  
class Delay
{
public:
  Delay();
  Delay(double delay, double dt, double initial=0);
  ~Delay() {};

  void reset();
  double update(double val);
  double get() { return m_history.front(); };
  
protected:
  int m_numFramesDelay;
  double m_initial;
  std::queue<double> m_history;
};
  
//----------------------------------------------------------------------------------------------------------------------
// 
//----------------------------------------------------------------------------------------------------------------------  
class Derivative
{
public:
  Derivative(double initial=0);
  ~Derivative() {};
  
  void reset(double initial=0);
  double update(double val, float dt);
  double get() { return m_dVal; };
  
protected:;
  double m_prevVal;
  double m_dVal;
};  

//----------------------------------------------------------------------------------------------------------------------    
// Inlines for Delay
//----------------------------------------------------------------------------------------------------------------------    
inline Delay::Delay()
{
  m_numFramesDelay = 0; // No delay
  m_initial = 0;
  reset();
}

//----------------------------------------------------------------------------------------------------------------------  
inline Delay::Delay(double delay, double dt, double initial)
{
  m_numFramesDelay = delay / dt;
  m_initial = initial;
  reset();
}

//----------------------------------------------------------------------------------------------------------------------    
inline void Delay::reset()
{
  m_history = std::queue<double>();
  for(int i = 0; i <= m_numFramesDelay; ++i)
    m_history.push(m_initial);
}
  
//----------------------------------------------------------------------------------------------------------------------    
inline double Delay::update(double val)
{
  // Push new state on to delay line (pushes to the back)  
  m_history.push(val);
  
  // Remove the oldest information from queue (pops from front)
  m_history.pop();
  
  // Return oldest value
  return m_history.front(); 
}
  
  
//----------------------------------------------------------------------------------------------------------------------    
// Inlines for Derivative
//----------------------------------------------------------------------------------------------------------------------    
inline Derivative::Derivative(double initial)
{
  reset(initial);
}

//----------------------------------------------------------------------------------------------------------------------    
inline void Derivative::reset(double initial)
{
  m_prevVal = initial;
  m_dVal = 0.0;  
}

//----------------------------------------------------------------------------------------------------------------------    
inline double Derivative::update(double val, float dt)
{
  m_dVal = (val - m_prevVal) / dt;
  m_prevVal = val;
  return m_dVal;
}  
  
} // namespace

#endif


