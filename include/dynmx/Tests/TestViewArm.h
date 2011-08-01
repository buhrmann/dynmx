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
#include "ArmView3d.h"

#include "cinder/gl/gl.h"
#include "cinder/Matrix.h"

#define NUM_MUSCLES 6

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
    
    for(int i = 0; i < NUM_MUSCLES; ++i)
    {
      m_excitation[i] = 0;
    }    
    
    // 3d view
    m_armView = new dmx::Arm3dView(m_arm, m_hasMuscles);
    m_armView->rotate(ci::Vec4f(0,0,1,1), -PI_OVER_TWO);
    m_armView->translate(ci::Vec4f(-0.25, 0, 0, 1));
    m_scene3d.m_children.push_back(m_armView);
    
    // 2d view
    m_plot = new dmx::Plot(600.0, 180, NUM_MUSCLES, 200);
    if(m_hasMuscles)
    {    
      for(int i = 0; i < ((dmx::ArmMuscled*)m_arm)->getNumMuscles(); ++i)
      {
        m_plot->setLabel(i, ((dmx::ArmMuscled*)m_arm)->getMuscle(i)->getName());
      }
    }
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
    //dmx::drawBasis(0.1f);
    if(m_hasMuscles)
    {    
      for(int i = 0; i < ((dmx::ArmMuscled*)m_arm)->getNumMuscles(); ++i)
      {
        ((dmx::ArmMuscled*)m_arm)->getMuscle(i)->setExcitation(m_excitation[i]);
      }
    }
  };
  
  //---------------------------------------------------------------------------------------------------------------------
  virtual void draw2d()
  {
    //m_plot->addPoint(m_arm->getJointAngle(dmx::JT_elbow), 0);
    //m_plot->addPoint(m_arm->getJointAngle(dmx::JT_shoulder), 1);    
    if(m_hasMuscles)
    {
      for(int i = 0; i < ((dmx::ArmMuscled*)m_arm)->getNumMuscles(); i++)
      {
        double val = ((dmx::ArmMuscled*)m_arm)->getMuscle(i)->getNormalisedLength();
        //double l1 = ((dmx::ArmMuscled*)m_arm)->getMuscle(0).getVelocity();        
        m_plot->addPoint(val, i);
      }
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
    m_hud.addSeparator("Muscle Controls");
    if(m_hasMuscles)
    {
      for(int i = 0; i < ((dmx::ArmMuscled*)m_arm)->getNumMuscles(); i++)
      {
        m_hud.addParam( ((dmx::ArmMuscled*)m_arm)->getMuscle(i)->getName(), &m_excitation[i], "min=0.0 max=1.0 step=0.01" );
      }
    }  
  };
  
  bool m_hasMuscles;
  dmx::Arm* m_arm; 
  dmx::Arm3dView* m_armView;
  dmx::Plot* m_plot;  
  int32_t m_fixedFrameRate;
  float m_excitation[NUM_MUSCLES];
};

#endif