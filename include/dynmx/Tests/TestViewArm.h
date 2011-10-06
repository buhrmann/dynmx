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
#include "Scene.h"
#include "ArmViz.h"
#include "ArmMuscledViz.h"

#include "cinder/gl/gl.h"
#include "cinder/Matrix.h"
#include "SimpleGUI.h"

#define NUM_MUSCLES 6

#define HAS_REFLEX 1

// ---------------------------------------------------------------------------------------------------------------------
class TestViewArm : public dmx::View
{
  
public:
  
  //--------------------------------------------------------------------------------------------------------------------
  TestViewArm() : m_arm(0), m_hasMuscles(false), m_fixedFrameRate(100), m_selectedMuscle(0) {};
  TestViewArm(dmx::Arm* arm, bool hasMuscles = false) : m_arm(arm), m_hasMuscles(hasMuscles), 
    m_fixedFrameRate(DEFAULT_FRAME_RATE), m_selectedMuscle(0) {};
  
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
    m_armViz = m_hasMuscles ? new dmx::ArmMuscledViz((dmx::ArmMuscled*)m_arm) : new dmx::ArmViz(m_arm);
    m_armViz->rotate(ci::Vec4f(0,0,1,1), -PI_OVER_TWO);
    m_armViz->translate(ci::Vec4f(-0.25, 0, 0, 1));
    m_scene3d.m_children.push_back(m_armViz);
    
    // 2d view
    m_plotMuscles = new dmx::Plot(600.0, 180, NUM_MUSCLES, 200);
    if(m_hasMuscles)
    {    
      m_selectedMuscle = 0;
      m_selectedReflex = 0;
      std:string muscleName = ((dmx::ArmMuscled*)m_arm)->getMuscle(m_selectedMuscle)->getName();
      m_muscleLabel->setText(muscleName);      
      for(int i = 0; i < ((dmx::ArmMuscled*)m_arm)->getNumMuscles(); ++i)
      {
        m_plotMuscles->setLabel(i, ((dmx::ArmMuscled*)m_arm)->getMuscle(i)->getName());
      }
      m_plotMuscles->translate(ci::Vec4f(50, 50, 0, 1)); 
      m_scene2d.m_children.push_back(m_plotMuscles);
      
#if HAS_REFLEX    
      m_plotReflex1 = new dmx::Plot(600.0, 180, 4, 200);
      m_plotReflex1->translate(ci::Vec4f(50, 50 + 180 + 20, 0, 1));      
      m_plotReflex1->setLabel(0, "F1");
      m_plotReflex1->setLabel(1, "F2");
      m_plotReflex1->setLabel(2, "F3");      
      m_plotReflex2 = new dmx::Plot(600.0, 180, 4, 200);      
      m_plotReflex2->translate(ci::Vec4f(50, 50 + 180 + 20 + 180 + 20, 0, 1));            
      m_plotReflex2->setLabel(0, "E1");
      m_plotReflex2->setLabel(1, "E2");
      m_plotReflex2->setLabel(2, "E3");
      m_scene2d.m_children.push_back(m_plotReflex1);
      m_scene2d.m_children.push_back(m_plotReflex2);
#endif          
    }

    
    m_trackMouse = false;
  };
  
  
  //--------------------------------------------------------------------------------------------------------------------
  virtual int getDesiredFrameRate()
  {
    return m_fixedFrameRate;
  }

  //--------------------------------------------------------------------------------------------------------------------  
  virtual void update(float dt)
  {
    //m_app->setFixedTimeStep(1.0f / (float)m_fixedFrameRate);
    
    // Tracking cursor position
    if(m_trackMouse)
    {
      m_target = m_armViz->toLocalSpace(m_mouseWorld);
      //m_arm->setTarget(dmx::Pos(m_target));
    }    
    
    // Visual control of gravity
    m_arm->setGravity(m_gravity);
    
    // Visual control of muscles
    if(m_hasMuscles)
    {
      dmx::ArmMuscled* marm = (dmx::ArmMuscled*)m_arm;
      
      // Change which muscle the gui controls
      std:string muscleName = marm->getMuscle(m_selectedMuscle)->getName();
      if(m_muscleLabel->getText() != muscleName)
      {
        m_muscleLabel->setText(muscleName);
        m_fmaxControl->setControlledVariable(&marm->getMuscle(m_selectedMuscle)->m_maxForce);
        m_pGainControl->setControlledVariable(&marm->getEPController(m_selectedMuscle)->m_pGain);
        m_vGainControl->setControlledVariable(&marm->getEPController(m_selectedMuscle)->m_vGain);
        m_dGainControl->setControlledVariable(&marm->getEPController(m_selectedMuscle)->m_dGain);
      }
      
      // Visual control of muscle activation
      for(int i = 0; i < marm->getNumMuscles(); ++i)
      {
        marm->getMuscle(i)->setExcitation(m_excitation[i]);
      }
    }
    
    // Update plot data
    int r = m_selectedReflex;    
    if(m_hasMuscles)
    {
      dmx::ArmMuscled* marm =  (dmx::ArmMuscled*)m_arm;
      /*
       for(int i = 0; i < marm->getNumMuscles(); i++)
       {
       //double val = marm->getMuscle(i)->getNormalisedLength();
       //double val = ((dmx::ArmMuscled*)m_arm)->getMuscle(i)->getUnitLength();
       //double val = ((dmx::ArmMuscled*)m_arm)->getMuscle(i)->getNormalisedVelocity();
       m_plotMuscles->addPoint(val, i);
       }*/
      
      // Joint angles
      //m_plot->addPoint(m_arm->getJointAngle(dmx::JT_elbow), 0);
      //m_plot->addPoint(m_arm->getJointAngle(dmx::JT_shoulder), 1);    
      
      // Desired joint angles
      //m_plotMuscles->addPoint(marm->getDesiredJointAngle(dmx::JT_elbow), 0);
      //m_plotMuscles->addPoint(marm->getDesiredJointAngle(dmx::JT_shoulder), 1);
      
      // Desired muscle lengths
      m_plotMuscles->addPoint(marm->getReflex(r)->m_desiredLength[0], 0);
      m_plotMuscles->addPoint(marm->getReflex(r)->m_desiredLength[1], 1);
      
#if HAS_REFLEX      
      m_plotReflex1->addPoint(marm->getReflex(r)->m_spindlePri[0], 0);
      m_plotReflex1->addPoint(marm->getReflex(r)->m_IaInOut[0], 1);
      m_plotReflex1->addPoint(marm->getReflex(r)->m_alpha[0], 2);
      
      /*
       m_plotReflex->addPoint(0.5*(marm->getReflex(r)->getAlphaOutput(0)+marm->getReflex(r)->getAlphaOutput(1)), 0);
       m_plotReflex->addPoint(marm->getReflex(r)->getCoContraction(0), 1);
       m_plotReflex->addPoint(marm->getReflex(r)->getAlphaOutput(0), 2);
       m_plotReflex->addPoint(marm->getReflex(r)->getAlphaOutput(1), 3);
       */
      
      m_plotReflex2->addPoint(marm->getReflex(r)->m_spindlePri[1], 0);
      m_plotReflex2->addPoint(marm->getReflex(r)->m_IaInOut[1], 1);
      m_plotReflex2->addPoint(marm->getReflex(r)->m_alpha[1], 2);
      
      //m_plotReflex->addPoint(marm->getReflex(r)->m_desiredLength[0], 2);
      //m_plotReflex->addPoint(marm->getReflex(r)->m_desiredLength[1], 5);
      
      /*
       m_plotReflex->addPoint(marm->getMuscle(m_selectedMuscle)->getActiveForce(), 0);
       m_plotReflex->addPoint(marm->getMuscle(m_selectedMuscle)->getPassiveForce(), 1);
       m_plotReflex->addPoint(marm->getMuscle(m_selectedMuscle)->getVelocityForce(), 2);      
       */
#endif      
    }    
    
  }
  
  //--------------------------------------------------------------------------------------------------------------------
  virtual void draw3d()
  {
    //dmx::drawBasis(0.1f);   
  };
  
  //---------------------------------------------------------------------------------------------------------------------
  virtual void draw2d()
  {
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
      case 't':
        m_trackMouse = !m_trackMouse;
        break;
    }
    // default behaviour in base class
    View::keyDown(event);
  }
  
  //--------------------------------------------------------------------------------------------------------------------
  virtual void buildGui() 
  { 
#if !USE_SIMPLE_GUI
    m_gui.addParam( "FPS", &m_fixedFrameRate, "min=10 max=300 step=5");     
    m_gui.addSeparator("Muscle Controls");
    m_gui.addParam("Gravity", &m_gravity, "min=0.0 max=9.81 step=0.01");
    if(m_hasMuscles)
    {
      for(int i = 0; i < ((dmx::ArmMuscled*)m_arm)->getNumMuscles(); i++)
      {
        m_gui.addParam( ((dmx::ArmMuscled*)m_arm)->getMuscle(i)->getName(), &m_excitation[i], "min=0.0 max=1.0 step=0.01" );
      }
    }  
  };
#else
  m_gui->addParam("FPS", &m_fixedFrameRate, 10, 300, 100);   
  
  //m_gui->addColumn();
  m_gui->addPanel();
  m_gui->addLabel("Muscle Controls");
  m_gui->addParam("Gravity", &m_gravity, 0.0, 9.81, m_arm->getGravity());
  if(m_hasMuscles)
  {
    dmx::ArmMuscled* marm = (dmx::ArmMuscled*)m_arm;
    for(int i = 0; i < marm->getNumMuscles(); i++)
    {
      m_gui->addParam( marm->getMuscle(i)->getName(), &m_excitation[i], 0.0, 1.0, 0.0);
    }
                             
    m_gui->addPanel();
    m_gui->addParam("Muscle", &m_selectedMuscle, 0, 5, m_selectedMuscle);
    m_muscleLabel = m_gui->addLabel("Selected Muscle");
    m_fmaxControl = m_gui->addParam("Fmax", &(marm->getMuscle(m_selectedMuscle)->m_maxForce), 0.0, 3000.0, marm->getMuscle(m_selectedMuscle)->m_maxForce);
    m_pGainControl = m_gui->addParam("P", &(marm->getEPController(m_selectedMuscle)->m_pGain), 0.0, 100.0, marm->getEPController(m_selectedMuscle)->m_pGain);
    m_vGainControl = m_gui->addParam("V", &(marm->getEPController(m_selectedMuscle)->m_vGain), 0.0, 100.0, marm->getEPController(m_selectedMuscle)->m_vGain);
    m_dGainControl = m_gui->addParam("D", &(marm->getEPController(m_selectedMuscle)->m_dGain), 0.0, 100.0, marm->getEPController(m_selectedMuscle)->m_dGain);
    
    m_gui->addPanel();
    m_gui->addLabel("Reflex Controls");
    m_gui->addParam("Joint", &m_selectedReflex, 0, 1, m_selectedReflex);
  }
};
#endif
  
  bool m_hasMuscles;
  dmx::Arm* m_arm; 
  dmx::ArmViz* m_armViz;
  dmx::Plot* m_plotMuscles;
  dmx::Plot* m_plotReflex1;
  dmx::Plot* m_plotReflex2;
  int32_t m_fixedFrameRate;
  float m_excitation[NUM_MUSCLES];
  float m_gravity;
  int m_selectedMuscle;
  int m_selectedReflex;
  Vec3f m_target;
  bool m_trackMouse;
  mowa::sgui::LabelControl* m_muscleLabel;
  mowa::sgui::DoubleVarControl* m_fmaxControl;
  mowa::sgui::FloatVarControl* m_pGainControl;
  mowa::sgui::FloatVarControl* m_vGainControl;
  mowa::sgui::FloatVarControl* m_dGainControl;
};

#endif