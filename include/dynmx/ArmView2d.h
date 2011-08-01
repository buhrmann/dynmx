/*
 *  Arm2dView.h
 *  openDynmx
 *
 *  Created by Thomas Buhrmann on 20/02/2010.
 *  Copyright 2010 __MyCompanyName__. All rights reserved.
 *
 */

#ifndef _OFX_ARM2D_VIEW_
#define _OFX_ARM2D_VIEW_

#include "Scene.h"
#include "Arm.h"

namespace dmx
{

//----------------------------------------------------------------------------------------------------------------------
// 2d visualisation of the Arm model
//----------------------------------------------------------------------------------------------------------------------
class Arm2dView : public NodeGroup
{
  
public:
  
  Arm2dView(Arm* model, bool hasMuscles = false) : m_arm(model), m_hasMuscles(hasMuscles) { init(); };
  
  void setModel(Arm* model) { m_arm = model; };
  virtual void update();
  
protected:
  virtual void init();
  
  // pointer to model to retrieve updated coordinates from
  Arm* m_arm;
  
  bool m_hasMuscles;  
  
};
  
} // namespace dmx

#endif