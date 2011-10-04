/*
 *  Arm3dView.h
 *  openDynmx
 *
 *  Created by Thomas Buhrmann on 20/02/2010.
 *  Copyright 2010 __MyCompanyName__. All rights reserved.
 *
 */

#ifndef _OFX_ARM3D_VIEW_
#define _OFX_ARM3D_VIEW_

#include "Scene.h"
#include "Arm.h"

namespace dmx
{

class Muscle;

//----------------------------------------------------------------------------------------------------------------------
// 3d visualisation of the Arm model
//----------------------------------------------------------------------------------------------------------------------
class Arm3dView : public NodeGroup
{
  
public:
  
  Arm3dView(Arm* model, bool hasMuscles = false) : m_arm(model), m_hasMuscles(hasMuscles) { init(); };
  
  void setModel(Arm* model) { m_arm = model; };
  virtual void update();

  // Transforms a given point p from world to local space
  Vec3f toLocalSpace(Vec3f p);
  
protected:
  virtual void init();
  
  // pointer to model to retrieve updated coordinates from
  Arm* m_arm;
  
  bool m_hasMuscles;
  
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