/*
 *  ArmMuscledViz.h
 *  openDynmx
 *
 *  Created by Thomas Buhrmann on 20/02/2010.
 *  Copyright 2010 __MyCompanyName__. All rights reserved.
 *
 */

#ifndef _DMX_ARM_MUSCLED_VIZ_
#define _DMX_ARM_MUSCLED_VIZ_

#include "Scene.h"
#include "ArmViz.h"

namespace dmx
{

class ArmMuscled;

//----------------------------------------------------------------------------------------------------------------------
// 3d visualisation of the Arm model
//----------------------------------------------------------------------------------------------------------------------
class ArmMuscledViz : public ArmViz
{
  
public:
  
  ArmMuscledViz(ArmMuscled* model) : ArmViz((Arm*)model), m_arm(model) { init(); };
  
  void setModel(ArmMuscled* model) { m_arm = model; };
  virtual void update();
  
  void setDrawDesired(bool b) { m_drawDesiredState = b; };
  
protected:
  virtual void init();
  
  // Pointer to model to retrieve updated coordinates from
  ArmMuscled* m_arm;
  
  bool m_drawDesiredState;
  
  struct ColorPalette
  {
    ci::ColorA shortMuscle;
    ci::ColorA longMuscle;
    ci::ColorA midMuscle;
    ci::ColorA boneOutline;
    ci::ColorA boneFill;
    ci::ColorA jointOutline;
    ci::ColorA jointFill;
    ci::ColorA limitsFill;
    ci::ColorA desired;
    ci::ColorA trajectory;
  };
  
  ColorPalette m_colors;
};
   
} // namespace dmx

#endif