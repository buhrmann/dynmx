/*
 *  ArmMuscled.h
 *  cinder_dynmx
 *
 *  Created by Thomas Buhrmann on 28/06/2011.
 *  Copyright 2011 __MyCompanyName__. All rights reserved.
 *
 */

#ifndef _DMX_ARM_MUSCLED_
#define _DMX_ARM_MUSCLED_

#include "Arm.h"
#include "Muscle.h"

namespace dmx
{

//----------------------------------------------------------------------------------------------------------------------
/// Extends Arm to add control via PDs. 
//----------------------------------------------------------------------------------------------------------------------
class ArmMuscled : public Arm
{

public:

  /// Functions inherited from class Model
  virtual void init();
  virtual void reset(float elbAngle = 0.0f, float shdAngle = 0.0f);

  /// Tick arm model given desired joint angles
  void update(float timeStep); 
  
  int getNumMuscles() const { return m_muscles.size(); };
  Muscle* getMuscle(int i) { assert(i < m_muscles.size()); return m_muscles[i]; };

  float getJointRadius(int joint) const { return m_jointRadius[joint]; }; 

protected:

  // Six muscles: 2 per joint and two biarticulated.
  std::vector<Muscle*> m_muscles;

  float m_jointRadius[2];
 
}; // class ArmPD

} // namespace dmx

#endif
