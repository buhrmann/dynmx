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
  TestViewArm() : m_arm(0), m_fixedFrameRate(100) {};
  TestViewArm(Arm2d* arm) : m_arm(arm), m_fixedFrameRate(100) {};
  
  // functions to be implemented by subclasses
  //--------------------------------------------------------------------------------------------------------------------
  virtual void setupScene()
  {
    assert(m_arm);
    
    // 3d view
    m_armView = new dmx::Arm3dView(m_arm);
    //m_armView->translate(ci::Vec4f(50, 50, 0, 1));
    m_scene3d.m_children.push_back(m_armView);
    
    // 2d view
    m_plot = new dmx::Plot(300.0, 180, 2, 100);
    m_plot->translate(ci::Vec4f(50, 50, 0, 1)); 
    m_scene2d.m_children.push_back(m_plot);
  };
  
  //--------------------------------------------------------------------------------------------------------------------
  virtual void draw3d()
  {
    m_app->setFixedTimeStep(1.0f / (float)m_fixedFrameRate);
    dmx::drawBasis(0.1f);
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
    switch(event.getChar())
    {
      case '=':
        m_fixedFrameRate++;
        break;
      case '-':
        m_fixedFrameRate--;
        break;
    }
    // default behaviour in base class
    View::keyDown(event);
  }
  
  //--------------------------------------------------------------------------------------------------------------------
  virtual void buildGui() 
  { 
    //m_hud.addParam( "FPS", &m_fixedFrameRate, "min=10 max=300 step=5"); 
    m_hud.addParam( "Elb P", &m_arm->m_pd[JT_elbow].m_P, "min=0.1 max=20.0 step=0.5" );
    m_hud.addParam( "Elb D", &m_arm->m_pd[JT_elbow].m_D, "min=0.1 max=20.0 step=0.5" );
    m_hud.addParam( "Shd P", &m_arm->m_pd[JT_shoulder].m_P, "min=0.1 max=50.0 step=0.5" );
    m_hud.addParam( "Shd D", &m_arm->m_pd[JT_shoulder].m_D, "min=0.1 max=50.0 step=0.5" );    
  };
  
  
  Arm2d* m_arm; 
  dmx::Arm3dView* m_armView;
  dmx::Plot* m_plot;  
  int32_t m_fixedFrameRate;
};

#endif