/*
 *  EPController.h
 *  cinder_dynmx
 *
 *  Created by Thomas Buhrmann on 8/2/11.
 *  Copyright 2011 __MyCompanyName__. All rights reserved.
 *
 */

#ifndef _DMX_EP_CONTROLLER_
#define _DMX_EP_CONTROLLER_

#include "Dynmx.h"
#include "Model.h"

#include <queue>

namespace dmx
{
  
class Muscle;
  
//----------------------------------------------------------------------------------------------------------------------
// A lambda-type equilibrium-point controller with optional velocity-reference term
//----------------------------------------------------------------------------------------------------------------------  
class EPController : public Model
{
  
public:
  
  EPController(Muscle* muscle, float pGain, float vGain, float dGain, int delay);
  
  // Inherited from class Model
  virtual void init();
  virtual void reset();
  virtual void update(float dt);
  
  // Setters
  void setGains(float pGain, float vGain, float dGain) { m_pGain = pGain; m_vGain = vGain; m_dGain = dGain; };
  void setDesiredLength(double length) { m_desiredLengthPrev = m_desiredLength; m_desiredLength = length; };
  
  // Getters
  double getActivation() { return m_activation; };
  Muscle* getMuscle() { return m_muscle; };
  
  float m_pGain;
  float m_vGain;
  float m_dGain;
  
protected:  
  
  Muscle* m_muscle;
  
  int m_numFramesDelay;
  
  std::queue<double> m_lengths;
  std::queue<double> m_velocities;
  
  double m_desiredLength;
  double m_desiredLengthPrev;
  double m_desiredVelocity;
  
  double m_activation;
  
}; // class EPController
  
} // namespace dmx

#endif