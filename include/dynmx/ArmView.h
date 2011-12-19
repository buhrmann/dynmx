//
//  ArmView.h
//  cinder_dynmx
//
//  Created by Thomas Buhrmann on 29/03/2011.
//  Copyright 2011 __MyCompanyName__. All rights reserved.
//

#ifndef _DMX_ARM_VIEW_
#define _DMX_ARM_VIEW_

#include "View.h"
#include "Scene.h"
#include "Arm.h"
#include "Scene.h"
#include "ArmViz.h"


#include "cinder/gl/gl.h"
#include "cinder/Matrix.h"
#include "SimpleGUI.h"

namespace dmx
{

// ---------------------------------------------------------------------------------------------------------------------
class ArmView : public dmx::View
{
  
public:
  
  ArmView(dmx::Arm* arm) : m_arm(arm), m_fixedFrameRate(DEFAULT_VIEW_FRAME_RATE) {};
  
  // Inherited functions
  virtual void setupScene();
  virtual int getDesiredFrameRate() { return m_fixedFrameRate; };
  virtual void update(float dt);
  
  virtual void draw3d(){};
  virtual void draw2d(){};
  
  virtual void keyDown (ci::app::KeyEvent event);
  
  virtual void buildGui();


  dmx::Arm* m_arm; 
  dmx::ArmViz* m_armViz;
  Vec3f m_target;  
  int32_t m_fixedFrameRate;
  float m_gravity;
  bool m_trackMouse;
}; // class
  

//----------------------------------------------------------------------------------------------------------------------
// Inline implementations
//----------------------------------------------------------------------------------------------------------------------
inline void ArmView::setupScene()
{
  assert(m_arm);
  
  // 3d view
  m_armViz = new dmx::ArmViz(m_arm);
  m_armViz->rotate(ci::Vec4f(0,0,1,1), -PI_OVER_TWO);
  m_armViz->translate(ci::Vec4f(0.25, 0, 0, 1));
  m_scene3d.m_children.push_back(m_armViz);
  
  m_trackMouse = false;
}
  
//--------------------------------------------------------------------------------------------------------------------  
inline void ArmView::update(float dt)
{
  // Tracking cursor position
  if(m_trackMouse)
  {
    m_target = m_armViz->toLocalSpace(m_mouseWorld);
  }    
  
  // Visual control of gravity
  m_arm->setGravity(m_gravity);
}

//---------------------------------------------------------------------------------------------------------------------
inline void ArmView::keyDown (ci::app::KeyEvent event)
{
  switch(event.getChar())
  {
    case '=':
      m_fixedFrameRate++;
      break;
    case '-':
      m_fixedFrameRate--;
      break;
    case 't':
      m_trackMouse = !m_trackMouse;
      break;
  }
  // default behaviour in base class
  View::keyDown(event);
}

//--------------------------------------------------------------------------------------------------------------------
inline void ArmView::buildGui() 
{ 
  m_gui->addParam("FPS", &m_fixedFrameRate, -1, 300, m_fixedFrameRate);   
  
  m_gui->addPanel();
  m_gui->addLabel("Arm Controls");
  m_gui->addParam("Gravity", &m_gravity, 0.0, 9.81, m_arm->getGravity());
};
  

} // namespace dmx

#endif