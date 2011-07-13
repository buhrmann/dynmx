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

  /// 
  void init(float elbAngle = 0.0f, float shdAngle = 0.0f);

  /// Tick arm model given desired joint angles
  void updateMuscles(float timeStep); 
  
  int getNumMuscles() const { return m_muscles.size(); };
  const Muscle& getMuscle(int i) const { assert(i < m_muscles.size()); return m_muscles[i]; };


protected:

  // Six muscles: 2 per joint and two biarticulated.
  std::vector<Muscle> m_muscles;
 
}; // class ArmPD

} // namespace dmx

#endif
