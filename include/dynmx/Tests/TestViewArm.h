//
//  TestViewArm.h
//  cinder_dynmx
//
//  Created by Thomas Buhrmann on 29/03/2011.
//  Copyright 2011 __MyCompanyName__. All rights reserved.
//

#ifndef _DMX_TEST_VIEW_ARM_
#define _DMX_TEST_VIEW_ARM_

#include "View.h"
#include "Scene.h"
#include "Arm.h"
#include "ArmView.h"

#include "cinder/gl/gl.h"
#include "cinder/Matrix.h"

// ---------------------------------------------------------------------------------------------------------------------
class TestViewArm : public dmx::View
{
  
public:
  
  //--------------------------------------------------------------------------------------------------------------------
  TestViewArm() : m_arm(0) {};
  TestViewArm(Arm2d* arm) : m_arm(arm) {};
  
  // functions to be implemented by subclasses
  //--------------------------------------------------------------------------------------------------------------------
  virtual void setupScene()
  {
    assert(m_arm);
    
    m_armView = new dmx::Arm3dView(m_arm);
    //m_armView->translate(ci::Vec3f(50, 50, 0));
    m_scene3d.m_children.push_back(m_armView);
    
    m_plot = new dmx::Plot(300.0, 180, 2, 100);
    m_plot->translate(ci::Vec3f(50, 50, 0)); 
    m_scene2d.m_children.push_back(m_plot);
  };
  
  //--------------------------------------------------------------------------------------------------------------------
  virtual void draw3d()
  {
    dmx::drawBasis(1.0f);
//    ci::Matrix44f TM = ci::Matrix44f::createRotation(ci::Vec3f(1.0f, 0.0f, 0.0f), PI_OVER_FOUR);  
//    glColor4f(1,1,1,0.5);
//    glPushMatrix();
//    glMultMatrixf(TM.m);
//    dmx::drawBox(1,2,1);
//    glPopMatrix();
  };
  
  //---------------------------------------------------------------------------------------------------------------------
  virtual void draw2d()
  {
    m_plot->addPoint(m_arm->getJointAngle(JT_elbow), 0);
    m_plot->addPoint(m_arm->getJointAngle(JT_shoulder), 1);
  };
  
  //---------------------------------------------------------------------------------------------------------------------
  virtual void keyDown (ci::app::KeyEvent event)
  {
    View::keyDown(event);
  }
  
  //--------------------------------------------------------------------------------------------------------------------
  virtual void buildGui() { /*gui.addPage("Default gui page");*/ };
  
  
  Arm2d* m_arm; 
  dmx::Arm3dView* m_armView;
  dmx::Plot* m_plot;  
};

#endif