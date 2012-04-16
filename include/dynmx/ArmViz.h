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
  
  void setDrawOverlays(bool b) { m_drawOverlays = b; };
  
protected:
  virtual void init();
  
  // pointer to model to retrieve updated coordinates from
  Arm* m_arm;
  
  bool m_drawOverlays;
};

} // namespace dmx

#endif