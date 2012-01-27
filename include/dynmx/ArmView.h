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


  Arm* m_arm; 
  ArmViz* m_armViz;
  NodeGroup* m_uiColumn;
  NodeGroup* m_uiColumn2;
  dmx::Plot* m_armEffPlot;
  dmx::Plot* m_armElbPlot;
  dmx::Plot* m_armShdPlot;
  
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
  
  // 2d viz
  float columnWidth = 300;
  float columnMargin = 5;
  float left = columnWidth + columnMargin;
  int pointsPerPlot = SETTINGS->getChild("Config/Globals/PlotDuration").getAttributeValue<float>("Value") * 
                      SETTINGS->getChild("Config/Globals/FrameRate").getAttributeValue<int>("Value");
  
  m_uiColumn = new NodeGroup();
  m_uiColumn->setRightAligned(true);
  m_uiColumn->translate(ci::Vec4f(left, columnMargin, 0, 1));
  m_scene2d.m_children.push_back(m_uiColumn);
  
  m_uiColumn2 = new NodeGroup();
  m_uiColumn2->setRightAligned(true);
  m_uiColumn2->translate(ci::Vec4f(left+left, columnMargin, 0, 1));
  m_scene2d.m_children.push_back(m_uiColumn2);  

  // Create a plot for effector movement
  m_armEffPlot = new dmx::Plot(columnWidth, 180, 6, pointsPerPlot);
  m_armEffPlot->setTitle("Effector kinematics");
  m_armEffPlot->setLabel(0, "x");
  m_armEffPlot->setLabel(1, "y");
  
  // Create a plot for arm data (different forces acting on arm e.g.)
  m_armElbPlot = new dmx::Plot(columnWidth, 180, 6, pointsPerPlot);
  m_armElbPlot->translate(ci::Vec4f(0, 180 + 20, 0, 1));  
  m_armElbPlot->setTitle("Elbow forces");
  m_armElbPlot->setLabel(0, "applied");
  m_armElbPlot->setLabel(1, "inertia");
  m_armElbPlot->setLabel(2, "interaction");
  m_armElbPlot->setLabel(3, "damping"); 
  m_armElbPlot->setLabel(4, "total"); 
  
  m_armShdPlot = new dmx::Plot(columnWidth, 180, 6, pointsPerPlot);
  m_armShdPlot->translate(ci::Vec4f(0, 400, 0, 1));
  m_armShdPlot->setTitle("Shoulder forces");
  m_armShdPlot->setLabel(0, "applied");  
  m_armShdPlot->setLabel(1, "inertia");
  m_armShdPlot->setLabel(2, "interaction");
  m_armShdPlot->setLabel(3, "damping");
  m_armShdPlot->setLabel(4, "total");
  
  m_uiColumn2->m_children.push_back(m_armEffPlot);
  m_uiColumn2->m_children.push_back(m_armElbPlot);
  m_uiColumn2->m_children.push_back(m_armShdPlot);    
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
  
  // Update arm plot data
  const Pos& effPos = m_arm->getEffectorPos();
  m_armEffPlot->addPoint(effPos.x, 0);
  m_armEffPlot->addPoint(effPos.y, 1);
  
  // The component forces are displayed negative because that's how they contribute to acceleration
  // in the computation of arm dynamics
  const Arm::State& state = m_arm->getState();
  float applied = state.torques[JT_elbow];
  float inertia = state.inertiaAcc[JT_elbow];
  float interaction = -state.interactionAcc[JT_elbow] - state.coriolisAcc[JT_elbow];
  float damping = -state.dampingAcc[JT_elbow];
  float total = applied + interaction + damping; // + gravity
  float acc = state.accelerations[JT_elbow] * inertia; // scaling so all plots roughly in same range
  m_armElbPlot->addPoint(applied, 0);
  m_armElbPlot->addPoint(inertia, 1);
  m_armElbPlot->addPoint(interaction, 2);
  m_armElbPlot->addPoint(damping, 3);
  m_armElbPlot->addPoint(total, 4);
  m_armElbPlot->addPoint(acc, 5);


  applied = state.torques[JT_shoulder];
  inertia = state.inertiaAcc[JT_shoulder];
  interaction = -state.interactionAcc[JT_shoulder] - state.coriolisAcc[JT_shoulder];
  damping = -state.dampingAcc[JT_shoulder];
  total = applied + interaction + damping; // + gravity
  acc = state.accelerations[JT_shoulder] * inertia; // scaling so all plots roughly in same range
  m_armShdPlot->addPoint(applied, 0);  
  m_armShdPlot->addPoint(inertia, 1);
  m_armShdPlot->addPoint(interaction, 2);
  m_armShdPlot->addPoint(damping, 3);  
  m_armShdPlot->addPoint(total, 4);  
  m_armShdPlot->addPoint(acc, 5);  
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
  m_gui->addParam("FPS", &m_fixedFrameRate, -1, 600, m_fixedFrameRate);   
  
  m_gui->addPanel();
  m_gui->addLabel("Arm Controls");
  m_gui->addParam("Gravity", &m_gravity, 0.0, 9.81, m_arm->getGravity());
};
  

} // namespace dmx

#endif