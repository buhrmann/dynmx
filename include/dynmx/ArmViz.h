/*
 *  ArmViz.h
 *  openDynmx
 *
 *  Created by Thomas Buhrmann on 20/02/2010.
 *  Copyright 2010 __MyCompanyName__. All rights reserved.
 *
 */

#ifndef _DMX_ARM_VIZ_
#define _DMX_ARM_VIZ_

#include "Scene.h"
#include "Arm.h"

namespace dmx
{

class Muscle;

//----------------------------------------------------------------------------------------------------------------------
// 3d visualisation of the Arm model
//----------------------------------------------------------------------------------------------------------------------
class ArmViz : public NodeGroup
{
  
public:
  
  ArmViz(Arm* model) : m_arm(model) { init(); };
  
  void setModel(Arm* model) { m_arm = model; };
  virtual void update();

  // Transforms a given point p from world to local space
  Vec3f toLocalSpace(Vec3f p);
  Vec3f toWorldSpace(Vec3f p);
  
protected:
  virtual void init();
  
  // pointer to model to retrieve updated coordinates from
  Arm* m_arm;
  
  // the rendered geometry
  Cylinder m_elbow;
  Cylinder m_shoulder;
#if 0  
  Box m_lowerArm;
  Box m_upperArm;
#else
  Cylinder m_lowerArm;
  Cylinder m_upperArm;
#endif
  
  
};

} // namespace dmx

#endif