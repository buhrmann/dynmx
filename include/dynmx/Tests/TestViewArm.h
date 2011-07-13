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
#include "ArmMuscled.h"
#include "ArmView.h"

#include "cinder/gl/gl.h"
#include "cinder/Matrix.h"

// ---------------------------------------------------------------------------------------------------------------------
class TestViewArm : public dmx::View
{
  
public:
  
  //--------------------------------------------------------------------------------------------------------------------
  TestViewArm() : m_arm(0), m_hasMuscles(false), m_fixedFrameRate(100) {};
  TestViewArm(dmx::Arm* arm, bool hasMuscles = false) : m_arm(arm), m_hasMuscles(hasMuscles), m_fixedFrameRate(100) {};
  
  // functions to be implemented by subclasses
  //--------------------------------------------------------------------------------------------------------------------
  virtual void setupScene()
  {
    assert(m_arm);
    
    // 3d view
    m_armView = new dmx::Arm3dView(m_arm, m_hasMuscles);
    //m_armView->translate(ci::Vec4f(50, 50, 0, 1));
    m_scene3d.m_children.push_back(m_armView);
    
    // 2d view
    m_plot = new dmx::Plot(600.0, 180, 2, 200);
    m_plot->translate(ci::Vec4f(50, 50, 0, 1)); 
    m_scene2d.m_children.push_back(m_plot);
  };
  
  
  //--------------------------------------------------------------------------------------------------------------------
  virtual int getDesiredFrameRate()
  {
    return m_fixedFrameRate;
  }
  
  //--------------------------------------------------------------------------------------------------------------------
  virtual void draw3d()
  {
    //m_app->setFixedTimeStep(1.0f / (float)m_fixedFrameRate);
    dmx::drawBasis(0.1f);
  };
  
  //---------------------------------------------------------------------------------------------------------------------
  virtual void draw2d()
  {
    //m_plot->addPoint(m_arm->getJointAngle(dmx::JT_elbow), 0);
    //m_plot->addPoint(m_arm->getJointAngle(dmx::JT_shoulder), 1);
    if(m_hasMuscles)
    {
      double l1 = ((dmx::ArmMuscled*)m_arm)->getMuscle(0).getNormalisedLength();
      double l2 = ((dmx::ArmMuscled*)m_arm)->getMuscle(1).getNormalisedLength();
      //double l1 = ((dmx::ArmMuscled*)m_arm)->getMuscle(0).getVelocity();
      //double l2 = ((dmx::ArmMuscled*)m_arm)->getMuscle(1).getVelocity();
      m_plot->addPoint(l1, 0);
      m_plot->addPoint(l2, 1);
    }
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
    m_hud.addParam( "FPS", &m_fixedFrameRate, "min=10 max=300 step=5"); 
//    m_hud.addParam( "Elb P", &m_arm->m_pd[dmx::JT_elbow].m_P, "min=0.1 max=20.0 step=0.5" );
//    m_hud.addParam( "Elb D", &m_arm->m_pd[dmx::JT_elbow].m_D, "min=0.1 max=20.0 step=0.5" );
//    m_hud.addParam( "Shd P", &m_arm->m_pd[dmx::JT_shoulder].m_P, "min=0.1 max=50.0 step=0.5" );
//    m_hud.addParam( "Shd D", &m_arm->m_pd[dmx::JT_shoulder].m_D, "min=0.1 max=50.0 step=0.5" );    
  };
  
  bool m_hasMuscles;
  dmx::Arm* m_arm; 
  dmx::Arm3dView* m_armView;
  dmx::Plot* m_plot;  
  int32_t m_fixedFrameRate;
};

#endif