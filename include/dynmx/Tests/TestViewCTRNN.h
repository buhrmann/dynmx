/*
 *  TestView.h
 *  openDynmx
 *
 *  Created by Thomas Buhrmann on 12/01/2011.
 *  Copyright 2011 __MyCompanyName__. All rights reserved.
 *
 */

#ifndef _DMX_TEST_VIEW_CTRNN_
#define _DMX_TEST_VIEW_CTRNN_

#include "View.h"
#include "CTRNN.h"
#include "CTRNNViz.h"

// ---------------------------------------------------------------------------------------------------------------------
class TestViewCTRNN : public dmx::View
{

public:

  //--------------------------------------------------------------------------------------------------------------------
  TestViewCTRNN() : m_ctrnn(0), m_activationFunction(CTRNN::kAF_Sigmoid) {};
  TestViewCTRNN(CTRNN* ctrnn) : m_ctrnn(ctrnn), m_activationFunction(CTRNN::kAF_Sigmoid) {};
  
  // functions to be implemented by subclasses
  //--------------------------------------------------------------------------------------------------------------------
  virtual void setupScene()
  {
    assert(m_ctrnn);
    
    m_ctrnnView = new dmx::CTRNNViz(m_ctrnn, 200);
    m_ctrnnView->translate(ci::Vec4f(50, 50, 0, 1));
    m_scene2d.m_children.push_back(m_ctrnnView);
  };
  
  //--------------------------------------------------------------------------------------------------------------------
  virtual void draw3d(){};
  
  //---------------------------------------------------------------------------------------------------------------------
  virtual void draw2d()
  {
    m_ctrnn->setActivationFunction(m_activationFunction);
  };
  
  //---------------------------------------------------------------------------------------------------------------------
  virtual void keyDown (ci::app::KeyEvent event)
  {
    View::keyDown(event);
    switch(event.getChar()) 
    {
      case 'r': 
        m_ctrnn->randomizeWeights(-10, 10);
        m_ctrnn->randomizeBiases(-10.0f, 10.0f);
        m_ctrnn->randomizeTimeConstants(2.0 / 30.0, 5.0f);         
        break;
      case 'c':
        m_ctrnn->setCenterCrossing();
        break;
    }
  }
  
  //--------------------------------------------------------------------------------------------------------------------
  virtual void buildGui()
  { 
    m_gui->addPanel();
    m_gui->addLabel("Muscle Controls");
    m_gui->addParam("ActivationFunction", &m_activationFunction, 0.0, CTRNN::kAF_NumFunctions, CTRNN::kAF_Sigmoid);
  };
  
  
  CTRNN* m_ctrnn; 
  dmx::CTRNNViz* m_ctrnnView;
  int m_activationFunction;
};

#endif
