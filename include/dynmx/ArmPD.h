/*
 *  ArmPD.h
 *  cinder_dynmx
 *
 *  Created by Thomas Buhrmann on 28/06/2011.
 *  Copyright 2011 __MyCompanyName__. All rights reserved.
 *
 */

#ifndef _DMX_ARM_PD_
#define _DMX_ARM_PD_

#include "Arm.h"
#include "PD.h"

namespace dmx
{

//----------------------------------------------------------------------------------------------------------------------
/// Extends Arm to add control via PDs. 
//----------------------------------------------------------------------------------------------------------------------
class ArmPD : public Arm
{

public:
  virtual ~ArmPD(){}; 
  virtual void init();
  virtual void reset();

  /// Tick arm model given desired joint angles
  void updatePD(float dt, float angleElbow, float angleShoulder);
  
  /// Tick arm model given a desired end-effector position
  void updatePosition(float timeStep, float x, float y);
  
  double getDesiredJointAngle(Joint j) { assert(j == JT_elbow || j == JT_shoulder); return m_desiredAngle[j]; };

  // Per-joint proportional-derivative controllers.
  PD m_pd[2];
  Pos m_desiredPos;  
  
protected:
  
  double m_desiredAngle[2];
 
}; // class ArmPD

} // namespace dmx

#endif